'''
/*MIT License

Copyright (c) 2024 JD edu. http://jdedu.kr author: conner.jeong@gmail.com

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN TH
SOFTWARE.*/
'''
# 로봇팔 디지털 트윈 브리지.
#
#   /command/joint_states (명령, 슬라이더 GUI 등)
#         │
#         ▼
#   [ArmTwinBridge] ── "a90b90c90d90e" ──▶ Arduino ──▶ SG90 x4
#         ▲                                   │
#         └──────── "s90.0,90.0,90.0,90.0" ◀──┘
#         │
#         ▼
#   /joint_states      (실제 상태 -> robot_state_publisher -> RViz 실물 모델)
#   /arm/tracking_error (명령 - 실제, rad)
#
# mode 파라미터
#   auto   : 시리얼이 있으면 실물, 없으면 시뮬레이션 (연결되면 자동 전환)
#   serial : 실물만. 연결이 없으면 /joint_states 를 발행하지 않는다
#   sim    : 항상 시뮬레이션 (Arduino 펌웨어와 같은 속도 제한 모델)
#

import glob
import math
import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

try:
    import serial
except ImportError:  # pyserial 미설치 시 sim 모드로만 동작
    serial = None

# URDF 관절 이름 (아두이노 명령 문자 a, b, c, d 순서)
JOINT_NAMES = ['base_shoulder', 'shoulder_arm1', 'arm1_arm2', 'arn2_end_arm']
CMD_CHARS = ['a', 'b', 'c', 'd']
# URDF 0 rad(팔이 똑바로 선 자세) = 서보 90도
SERVO_CENTER_DEG = 90.0


class ArmTwinBridge(Node):

    def __init__(self):
        super().__init__('ros_arm_bridge')

        self.declare_parameter('mode', 'auto')
        self.declare_parameter('port', 'auto')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('command_rate', 20.0)       # Hz, 아두이노로 명령 전송 주기
        self.declare_parameter('state_rate', 50.0)         # Hz, 시리얼 읽기 / sim 주기
        self.declare_parameter('state_timeout', 1.0)       # s, 이 시간 동안 상태가 없으면 끊김으로 판단
        self.declare_parameter('sim_speed_deg_s', 120.0)   # 아두이노 MAX_SPEED_DEG_S 와 맞출 것
        # 서보 장착 방향 보정: 실물이 RViz와 반대로 돌면 해당 관절을 -1 로
        self.declare_parameter('joint_directions', [1, 1, 1, 1])
        # 영점 보정(도): 똑바로 선 자세에서 서보 각도가 90이 아니면 그 차이
        self.declare_parameter('joint_offsets_deg', [0.0, 0.0, 0.0, 0.0])

        self.mode = self.get_parameter('mode').value
        self.port_param = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.state_timeout = self.get_parameter('state_timeout').value
        self.sim_speed = math.radians(self.get_parameter('sim_speed_deg_s').value)
        self.directions = [float(d) for d in self.get_parameter('joint_directions').value]
        self.offsets = [float(o) for o in self.get_parameter('joint_offsets_deg').value]
        if len(self.directions) != 4 or len(self.offsets) != 4:
            raise ValueError('joint_directions / joint_offsets_deg 는 4개 값이어야 합니다')
        if self.mode not in ('auto', 'serial', 'sim'):
            raise ValueError(f'mode 는 auto/serial/sim 중 하나여야 합니다: {self.mode}')
        if serial is None and self.mode != 'sim':
            self.get_logger().warn('pyserial 이 없어 sim 모드로 동작합니다 (pip install pyserial)')
            self.mode = 'sim'

        self.command = None                 # 최신 명령 [rad] x4
        self.actual = [0.0] * 4             # 최신 실제 상태 [rad] x4
        self.have_actual = False
        self.last_sent = None
        self.last_sent_time = 0.0
        self.last_state_time = 0.0
        self.ser = None
        self.rx_buf = b''
        self.next_connect_try = 0.0
        self.source = None                  # 'serial' / 'sim' / None
        # 로봇팔 펌웨어("s..." 상태)를 확인한 포트에만 명령을 보낸다.
        # (ESP32 등 다른 보드가 같은 PC에 꽂혀 있어도 건드리지 않기 위함)
        self.verified = False
        self.probe_deadline = 0.0
        self.rejected = {}                  # port -> 장치 식별값 (다시 꽂으면 바뀜)

        self.create_subscription(JointState, 'command/joint_states', self.command_callback, 10)
        self.state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.error_pub = self.create_publisher(JointState, 'arm/tracking_error', 10)
        self.status_pub = self.create_publisher(String, 'arm/status', 10)

        self.dt = 1.0 / self.get_parameter('state_rate').value
        self.create_timer(self.dt, self.state_loop)
        self.create_timer(1.0 / self.get_parameter('command_rate').value, self.command_loop)
        self.create_timer(1.0, self.publish_status)

        self.get_logger().info(f'디지털 트윈 브리지 시작 (mode={self.mode})')

    # ------------------------------------------------------------------ 변환
    def rad_to_servo(self, i, rad):
        deg = SERVO_CENTER_DEG + self.directions[i] * math.degrees(rad) + self.offsets[i]
        return int(round(max(0.0, min(180.0, deg))))

    def servo_to_rad(self, i, deg):
        return math.radians((deg - SERVO_CENTER_DEG - self.offsets[i]) * self.directions[i])

    # ------------------------------------------------------------------ 명령 입력
    def command_callback(self, msg):
        positions = dict(zip(msg.name, msg.position))
        if any(name not in positions for name in JOINT_NAMES):
            return
        self.command = [positions[name] for name in JOINT_NAMES]

    # ------------------------------------------------------------------ 시리얼
    @staticmethod
    def device_id(port):
        try:
            st = os.stat(port)
            return (st.st_ino, st.st_ctime_ns)
        except OSError:
            return None

    @staticmethod
    def port_in_use(port):
        """다른 프로세스(시리얼 모니터, 업로드 등)가 이미 연 포트인지 확인."""
        me = str(os.getpid())
        for fd_dir in glob.glob('/proc/[0-9]*/fd'):
            if fd_dir.split('/')[2] == me:
                continue
            try:
                for fd in os.listdir(fd_dir):
                    if os.readlink(os.path.join(fd_dir, fd)) == port:
                        return True
            except OSError:
                continue
        return False

    def find_port(self):
        if self.port_param != 'auto':
            return self.port_param
        ports = sorted(glob.glob('/dev/ttyUSB*')) + sorted(glob.glob('/dev/ttyACM*'))
        for p in ports:
            if p in self.rejected and self.rejected[p] == self.device_id(p):
                continue  # 이미 확인했는데 로봇팔이 아니었던 장치
            if self.port_in_use(p):
                continue  # 다른 프로그램이 쓰는 중 (예: ESP32 업로드/모니터)
            return p
        return None

    def try_connect(self):
        now = time.monotonic()
        if now < self.next_connect_try:
            return
        self.next_connect_try = now + 2.0
        port = self.find_port()
        if port is None:
            return
        if self.port_in_use(port):
            # 다른 bridge/시리얼 모니터와 동시에 열면 명령이 섞이고 상태 메시지가 깨진다
            self.get_logger().error(
                f'{port} 를 다른 프로그램이 사용 중입니다. 이미 실행 중인 launch나 '
                '시리얼 모니터를 먼저 종료하세요.', throttle_duration_sec=10.0)
            return
        try:
            ser = serial.Serial()
            ser.port = port
            ser.baudrate = self.baudrate
            ser.timeout = 0
            ser.write_timeout = 0.1
            ser.open()
        except (serial.SerialException, OSError) as e:
            self.get_logger().warn(f'{port} 열기 실패: {e}', throttle_duration_sec=10.0)
            return
        self.ser = ser
        self.rx_buf = b''
        self.last_sent = None
        self.verified = False
        # 포트를 열면 Arduino가 리셋된다 (부팅 약 2초) -> 넉넉히 기다린 뒤 판정
        self.probe_deadline = now + 4.0
        self.get_logger().info(f'{port} 확인 중... (로봇팔 펌웨어 응답 대기)')

    def disconnect(self, reason, warn=True):
        if warn:
            self.get_logger().warn(f'Arduino 연결 끊김: {reason}')
        try:
            self.ser.close()
        except Exception:
            pass
        self.ser = None
        self.verified = False

    def check_probe(self, now):
        """포트를 연 뒤 제한 시간 안에 상태가 안 오면 로봇팔이 아니라고 보고 놓아준다."""
        if self.ser is None or self.verified or now < self.probe_deadline:
            return
        port = self.ser.port
        self.disconnect('', warn=False)
        if self.port_param == 'auto':
            self.rejected[port] = self.device_id(port)
            self.get_logger().warn(
                f'{port}: 로봇팔 펌웨어 응답 없음 -> 이 장치는 다시 열지 않습니다 '
                '(펌웨어 업로드 후 다시 꽂으면 재시도)')
        else:
            self.next_connect_try = now + 5.0
            self.get_logger().warn(
                f'{port}: 로봇팔 펌웨어 응답 없음 (ros_control.ino 업로드 확인). 5초 후 재시도',
                throttle_duration_sec=30.0)

    def read_serial(self):
        try:
            n = self.ser.in_waiting
            if n:
                self.rx_buf += self.ser.read(n)
        except (serial.SerialException, OSError) as e:
            self.disconnect(e)
            return False

        got_state = False
        while b'\n' in self.rx_buf:
            line, self.rx_buf = self.rx_buf.split(b'\n', 1)
            line = line.strip().decode(errors='ignore')
            if line.startswith('s'):
                try:
                    degs = [float(v) for v in line[1:].split(',')]
                except ValueError:
                    continue
                if len(degs) != 4:
                    continue
                self.actual = [self.servo_to_rad(i, d) for i, d in enumerate(degs)]
                got_state = True
                if not self.verified:
                    self.verified = True
                    self.get_logger().info(f'로봇팔 Arduino 연결 확인: {self.ser.port}')
            elif line and (self.verified or 'Robot Arm' in line or 'pose' in line.lower()):
                self.get_logger().info(f'[Arduino] {line}')
        if len(self.rx_buf) > 1024:
            self.rx_buf = b''
        return got_state

    # ------------------------------------------------------------------ 시뮬레이션
    def step_sim(self):
        target = self.command if self.command is not None else [0.0] * 4
        max_step = self.sim_speed * self.dt
        for i in range(4):
            # 실물과 같은 0~180도 범위 제한
            lo, hi = self.servo_to_rad(i, 0), self.servo_to_rad(i, 180)
            t = max(min(lo, hi), min(max(lo, hi), target[i]))
            diff = t - self.actual[i]
            self.actual[i] += max(-max_step, min(max_step, diff))

    # ------------------------------------------------------------------ 메인 루프
    def state_loop(self):
        if not rclpy.ok():  # 종료 중
            return
        now = time.monotonic()
        updated = False

        if self.mode != 'sim':
            if self.ser is None:
                self.try_connect()
            if self.ser is not None and self.read_serial():
                self.last_state_time = now
                updated = True
                self.set_source('serial')
            self.check_probe(now)

        serial_alive = (self.ser is not None and self.verified
                        and now - self.last_state_time < self.state_timeout)
        if self.mode == 'sim' or (self.mode == 'auto' and not serial_alive and not updated):
            self.step_sim()
            updated = True
            self.set_source('sim')
        elif not serial_alive:
            self.set_source(None)

        if updated:
            self.have_actual = True
            self.publish_state()

    def set_source(self, source):
        if source != self.source:
            msg = {'serial': '실물(Arduino) 상태로 트윈 구동',
                   'sim': '시뮬레이션 상태로 트윈 구동 (Arduino 없음)',
                   None: 'Arduino 상태 수신 없음 - 트윈 갱신 중지'}[source]
            self.get_logger().info(msg)
            self.source = source

    def command_loop(self):
        # 로봇팔 펌웨어로 확인된 포트에만 명령 전송
        if self.ser is None or not self.verified or self.command is None:
            return
        cmd = [self.rad_to_servo(i, r) for i, r in enumerate(self.command)]
        now = time.monotonic()
        # 바뀌었을 때 + 0.5초마다 재전송 (아두이노 리셋 직후 유실 대비)
        if cmd == self.last_sent and now - self.last_sent_time < 0.5:
            return
        line = ''.join(f'{c}{v}' for c, v in zip(CMD_CHARS, cmd)) + 'e\n'
        try:
            self.ser.write(line.encode())
        except (serial.SerialException, OSError) as e:
            self.disconnect(e)
            return
        self.last_sent = cmd
        self.last_sent_time = now

    def publish_state(self):
        stamp = self.get_clock().now().to_msg()
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = list(JOINT_NAMES)
        msg.position = list(self.actual)
        self.state_pub.publish(msg)

        if self.command is not None:
            err = JointState()
            err.header.stamp = stamp
            err.name = list(JOINT_NAMES)
            err.position = [c - a for c, a in zip(self.command, self.actual)]
            self.error_pub.publish(err)

    def publish_status(self):
        s = String()
        port = self.ser.port if self.ser else None
        s.data = f'source={self.source} port={port} verified={self.verified}'
        self.status_pub.publish(s)

    def close(self):
        if self.ser is not None:
            self.ser.close()


def main(args=None):
    rclpy.init(args=args)
    node = ArmTwinBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
