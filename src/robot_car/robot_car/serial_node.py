"""시리얼 브릿지 노드: /cmd_vel -> 아두이노 / ESP32 (USB 유선 또는 WiFi 무선).

    구독: /cmd_vel (geometry_msgs/Twist)
    전송: "L,R\n"  (좌/우 PWM -255~255) 를 send_rate Hz 로 계속 전송

시뮬레이션의 pybullet_sim 노드 자리에 이 노드를 넣으면 실물이 움직입니다.
cmd_vel -> 바퀴 속도 계산은 시뮬레이션과 같은 robot_car/kinematics.py 를 사용합니다.

port 파라미터 (pyserial 의 serial_for_url 사용 -> 주소만 바꾸면 통신 방식이 바뀜)
    /dev/ttyACM0              Arduino Uno (USB)
    /dev/ttyUSB0              ESP32 DevKit, Uno 호환보드 CH340 (USB)
    socket://192.168.0.50:3333   ESP32 WiFi (TCP) -> 스케치에서 USE_WIFI 1

    ros2 run robot_car serial_node --ros-args -p port:=/dev/ttyACM0
"""
import serial
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node

from robot_car.board_status import BoardStatus
from robot_car.imu_report import ImuReporter
from robot_car.ir_report import IrReporter
from robot_car.kinematics import twist_to_wheels, wheel_speed_to_pwm


class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.port = self.declare_parameter('port', '/dev/ttyACM0').value
        self.baud = self.declare_parameter('baud', 115200).value
        self.cmd_timeout = self.declare_parameter('cmd_timeout', 0.5).value  # [s]
        send_rate = self.declare_parameter('send_rate', 20.0).value         # [Hz]
        # TT 모터는 PWM 이 낮으면 윙 소리만 나고 안 돎 -> 0이 아닌 명령은 최소 이 값 이상으로
        self.min_pwm = self.declare_parameter('min_pwm', 60).value

        self.ser = None
        self.left_pwm = self.right_pwm = 0
        self.last_cmd_time = self.get_clock().now()
        self.last_sent = None
        self.rx_buf = b''   # 보드에서 받은, 아직 줄바꿈이 오지 않은 데이터

        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.ir = IrReporter(self)   # ESP32 의 'IR,l,r' 보고 -> /ir/left, /ir/right
        self.imu = ImuReporter(self)  # ESP32 의 'IMU,...' 보고 -> /imu, /imu/yaw_deg
        self.status = BoardStatus(self)  # ESP32 의 'BOOT,...', 'STAT,...' -> 재부팅 원인, 워치독 경고 로그
        # 직선 주행 보정 세기 (ESP32 가 READY 를 보낼 때 'H,kp,kd' 로 전송). heading_kp:=0 이면 보정 끔
        self.heading_kp = float(self.declare_parameter('heading_kp', 6.0, ParameterDescriptor(dynamic_typing=True)).value)
        self.heading_kd = float(self.declare_parameter('heading_kd', 0.4, ParameterDescriptor(dynamic_typing=True)).value)
        self.create_timer(1.0 / send_rate, self.on_timer)
        self.create_timer(1.0, self.ensure_connected)   # 연결 끊기면 1초마다 재시도
        self.ensure_connected()

    # ---------- 시리얼 연결 ----------
    def ensure_connected(self):
        if self.ser is not None:
            return
        try:
            # serial_for_url: '/dev/tty...' 는 USB 시리얼, 'socket://IP:포트' 는 WiFi(TCP)
            # timeout=0: 읽기 논블로킹 (보드 응답이 없어도 노드가 멈추지 않음)
            self.ser = serial.serial_for_url(self.port, baudrate=self.baud,
                                             timeout=0, write_timeout=0.1)
            how = 'WiFi(TCP)' if self.port.startswith('socket://') else f'USB {self.baud}bps'
            self.get_logger().info(f'{self.port} 연결됨 ({how})')
        except (serial.SerialException, OSError) as e:
            self.get_logger().warn(f'{self.port} 연결 실패: {e}', throttle_duration_sec=5.0)
            self.ser = None

    def close(self, reason):
        self.get_logger().error(f'시리얼 끊김: {reason}')
        try:
            self.ser.close()
        except Exception:
            pass
        self.ser = None
        self.last_sent = None
        self.rx_buf = b''

    # ---------- cmd_vel -> PWM ----------
    def apply_deadzone(self, pwm):
        if pwm == 0:
            return 0
        sign = 1 if pwm > 0 else -1
        # 1~255 범위를 min_pwm~255 로 다시 매핑
        return sign * int(self.min_pwm + (abs(pwm) - 1) * (255 - self.min_pwm) / 254)

    def on_cmd_vel(self, msg):
        wl, wr = twist_to_wheels(msg.linear.x, msg.angular.z)
        self.left_pwm = self.apply_deadzone(wheel_speed_to_pwm(wl))
        self.right_pwm = self.apply_deadzone(wheel_speed_to_pwm(wr))
        self.last_cmd_time = self.get_clock().now()

    def on_timer(self):
        if self.ser is None:
            return
        # cmd_vel 이 끊기면 정지 명령 (teleop 이 죽었을 때 대비)
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if age > self.cmd_timeout:
            self.left_pwm = self.right_pwm = 0

        line = f'{self.left_pwm},{self.right_pwm}\n'
        try:
            # 같은 명령이라도 계속 보냄 -> 아두이노 워치독(0.5초)이 정지시키지 않도록
            self.ser.write(line.encode())
            if line != self.last_sent:
                self.get_logger().info(f'-> {line.strip()}')
                self.last_sent = line
            # 보드가 보낸 메시지(READY, ERR ...) 출력
            # 데이터는 줄 중간에서 잘려 도착할 수 있으므로 버퍼에 모았다가 '\n' 단위로 처리
            # (timeout=0 이라 read() 는 지금 와 있는 만큼만 즉시 돌려줌)
            self.rx_buf += self.ser.read(1024)
            *lines, self.rx_buf = self.rx_buf.split(b'\n')
            for raw in lines:
                text = raw.decode(errors='ignore').strip()
                if not text or self.ir.handle_line(text) or self.imu.handle_line(text) \
                        or self.status.handle_line(text):
                    continue
                self.get_logger().info(f'<- board: {text}')
                if text == 'READY':          # 보드 재부팅 직후 -> 보정 설정 전송
                    self.send_heading_config()
            if len(self.rx_buf) > 1024:   # 줄바꿈 없이 쓰레기만 쌓이는 경우
                self.rx_buf = b''
        except (serial.SerialException, OSError) as e:
            self.close(e)

    def send_heading_config(self):
        line = f'H,{self.heading_kp:g},{self.heading_kd:g}\n'
        self.ser.write(line.encode())
        state = '끔' if self.heading_kp <= 0 else f'켬 (kp={self.heading_kp:g}, kd={self.heading_kd:g})'
        self.get_logger().info(f'직선 주행 보정 설정 전송: {state}')

    def stop(self):
        if self.ser is not None:
            try:
                self.ser.write(b'0,0\n')
                self.ser.flush()
                self.ser.close()
            except Exception:
                pass


def main():
    rclpy.init()
    node = SerialNode()
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
