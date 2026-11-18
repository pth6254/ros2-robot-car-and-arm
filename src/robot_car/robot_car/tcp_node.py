"""TCP/IP 노드: /cmd_vel -> WiFi(TCP) -> ESP32.   PC 키보드로 자동차를 무선 조종.

    [PC]  wasd_teleop --/cmd_vel--> tcp_node ==WiFi(TCP)==> [ESP32] --> L298N --> 모터
                                  (클라이언트)              (서버, 3333번 포트)

    구독: /cmd_vel (geometry_msgs/Twist)
    전송: "L,R\n"  (좌/우 PWM -255~255) 를 send_rate Hz 로 계속 전송
          -> 유선 serial_node 와 같은 형식이라 ESP32 스케치는 공통

파라미터
    host        ESP32 주소. 'robot-car.local' (스케치의 mDNS 이름) 또는 IP '192.168.0.50'
    port        ESP32 TCP 서버 포트 (스케치의 TCP_PORT, 기본 3333)
    min_pwm     0 이 아닌 명령의 최소 PWM (TT 모터가 낮은 PWM 에서 안 도는 문제 보정)

    ros2 run robot_car tcp_node --ros-args -p host:=192.168.0.50
"""
import select
import socket
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node

from robot_car.board_status import BoardStatus
from robot_car.imu_report import ImuReporter
from robot_car.ir_report import IrReporter
from robot_car.kinematics import twist_to_wheels, wheel_speed_to_pwm


class TcpNode(Node):
    def __init__(self):
        super().__init__('tcp_node')
        self.host = self.declare_parameter('host', 'robot-car.local').value
        self.port = self.declare_parameter('port', 3333).value
        self.cmd_timeout = self.declare_parameter('cmd_timeout', 0.5).value   # [s]
        send_rate = self.declare_parameter('send_rate', 20.0).value          # [Hz]
        self.min_pwm = self.declare_parameter('min_pwm', 60).value
        # ESP32 는 0.05~0.2초마다 상태(IMU, IR)를 보냄 -> 이 시간 동안 아무것도 안 오면 '응답 없음'으로 판단
        self.rx_timeout = float(self.declare_parameter('rx_timeout', 2.0, ParameterDescriptor(dynamic_typing=True)).value)
        self.last_rx = time.monotonic()

        self.sock = None
        self.rx_buf = b''
        self.left_pwm = self.right_pwm = 0
        self.last_cmd_time = self.get_clock().now()
        self.last_sent = None

        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.ir = IrReporter(self)   # ESP32 의 'IR,l,r' 보고 -> /ir/left, /ir/right
        self.imu = ImuReporter(self)  # ESP32 의 'IMU,...' 보고 -> /imu, /imu/yaw_deg
        self.status = BoardStatus(self)  # ESP32 의 'BOOT,...', 'STAT,...' -> 재부팅 원인, 워치독 경고 로그
        # 직선 주행 보정 세기 (ESP32 가 READY 를 보낼 때 'H,kp,kd' 로 전송). heading_kp:=0 이면 보정 끔
        self.heading_kp = float(self.declare_parameter('heading_kp', 6.0, ParameterDescriptor(dynamic_typing=True)).value)
        self.heading_kd = float(self.declare_parameter('heading_kd', 0.4, ParameterDescriptor(dynamic_typing=True)).value)
        self.create_timer(1.0 / send_rate, self.on_timer)
        self.create_timer(1.0, self.ensure_connected)    # 끊기면 1초마다 재연결 시도
        self.ensure_connected()

    # ---------------- TCP 연결 ----------------
    def ensure_connected(self):
        if self.sock is not None:
            return
        try:
            # TCP 클라이언트: ESP32(서버)의 IP:포트로 접속. 최대 1초 대기
            sock = socket.create_connection((self.host, self.port), timeout=1.0)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)   # 짧은 명령 즉시 전송
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)   # 끊김 감지 보조
            sock.settimeout(0.1)                                         # 전송이 0.1초 넘게 막히면 오류
            self.sock = sock
            self.last_rx = time.monotonic()
            self.rx_buf = b''
            self.last_sent = None
            peer = '%s:%d' % sock.getpeername()[:2]
            self.get_logger().info(f'ESP32 연결됨: {self.host} ({peer})')
        except OSError as e:
            self.get_logger().warn(f'ESP32 {self.host}:{self.port} 연결 실패: {e}',
                                   throttle_duration_sec=5.0)

    def close(self, reason):
        self.get_logger().error(f'연결 끊김: {reason} -> 재연결 시도')
        try:
            self.sock.close()
        except Exception:
            pass
        self.sock = None

    # ---------------- cmd_vel -> PWM ----------------
    def apply_deadzone(self, pwm):
        if pwm == 0:
            return 0
        sign = 1 if pwm > 0 else -1
        return sign * int(self.min_pwm + (abs(pwm) - 1) * (255 - self.min_pwm) / 254)

    def on_cmd_vel(self, msg):
        wl, wr = twist_to_wheels(msg.linear.x, msg.angular.z)
        self.left_pwm = self.apply_deadzone(wheel_speed_to_pwm(wl))
        self.right_pwm = self.apply_deadzone(wheel_speed_to_pwm(wr))
        self.last_cmd_time = self.get_clock().now()

    def on_timer(self):
        if self.sock is None:
            return
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if age > self.cmd_timeout:          # teleop 이 꺼지면 정지
            self.left_pwm = self.right_pwm = 0

        line = f'{self.left_pwm},{self.right_pwm}\n'
        try:
            # 같은 명령도 계속 보냄 -> ESP32 워치독(0.5초)이 정지시키지 않도록
            self.sock.sendall(line.encode())
            if line != self.last_sent:
                self.get_logger().info(f'-> {line.strip()}')
                self.last_sent = line
            self.receive()
        except OSError as e:                 # 전송 실패 = WiFi 끊김, ESP32 꺼짐 등
            self.close(e)

    def receive(self):
        """ESP32 가 보낸 줄(READY, ERR ...) 출력. 줄 중간에서 잘려 와도 버퍼에 모아서 처리."""
        while select.select([self.sock], [], [], 0)[0]:
            data = self.sock.recv(1024)
            if not data:                     # 상대가 연결을 닫음
                raise ConnectionResetError('ESP32 가 연결을 닫음')
            self.rx_buf += data
            self.last_rx = time.monotonic()
        # 전송은 되는 것처럼 보여도(보낸 데이터가 PC 버퍼에 쌓일 뿐) ESP32 가 꺼졌거나 재부팅 중일 수 있음
        # -> ESP32 가 보내오는 상태 보고가 끊긴 것으로 판단
        silent = time.monotonic() - self.last_rx
        if silent > self.rx_timeout:
            raise TimeoutError(f'ESP32 가 {silent:.1f}초 동안 응답 없음 '
                               '(전원 꺼짐, 전압 부족으로 재부팅, WiFi 끊김 가능)')
        *lines, self.rx_buf = self.rx_buf.split(b'\n')
        for raw in lines:
            text = raw.decode(errors='ignore').strip()
            if not text or self.ir.handle_line(text) or self.imu.handle_line(text) \
                    or self.status.handle_line(text):
                continue
            self.get_logger().info(f'<- esp32: {text}')
            if text == 'READY':              # 접속/재부팅 직후 -> 보정 설정 전송
                self.send_heading_config()
        if len(self.rx_buf) > 1024:
            self.rx_buf = b''

    def send_heading_config(self):
        line = f'H,{self.heading_kp:g},{self.heading_kd:g}\n'
        self.sock.sendall(line.encode())
        state = '끔' if self.heading_kp <= 0 else f'켬 (kp={self.heading_kp:g}, kd={self.heading_kd:g})'
        self.get_logger().info(f'직선 주행 보정 설정 전송: {state}')

    def stop(self):
        if self.sock is not None:
            try:
                self.sock.sendall(b'0,0\n')   # 종료 시 정지 명령
                self.sock.close()
            except OSError:
                pass


def main():
    rclpy.init()
    node = TcpNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):  # Ctrl+C 또는 SIGTERM
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
