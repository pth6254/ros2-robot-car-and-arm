"""WASD 키보드 -> /cmd_vel (geometry_msgs/Twist) 발행 노드.

    w : 전진        s : 후진
    a : 좌회전      d : 우회전   (제자리 회전)
    q / e : 전진하면서 좌/우 커브
    space 또는 x : 정지
    + / - : 속도 증가/감소
    Ctrl+C : 종료

실행 방법:
    ros2 launch robot_car bringup.launch.py   # 조종 창(WASD-teleop)이 자동으로 따로 열림
    ros2 run robot_car wasd_teleop            # 직접 실행할 때는 일반 터미널에서
키 입력을 받아야 하므로 터미널(창)이 꼭 필요합니다.
"""
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

HELP = __doc__.split('실행 방법:')[0].replace('\n', '\r\n')  # 키 안내 부분만 화면에 표시

# 키: (선속도 방향, 각속도 방향)
BINDINGS = {
    'w': (1, 0), 's': (-1, 0),
    'a': (0, 1), 'd': (0, -1),
    'q': (1, 1), 'e': (1, -1),
    ' ': (0, 0), 'x': (0, 0),
}


class WasdTeleop(Node):
    def __init__(self):
        super().__init__('wasd_teleop')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.declare_parameter('linear_speed', 0.3)   # m/s
        self.declare_parameter('angular_speed', 2.0)  # rad/s
        self.lin = self.get_parameter('linear_speed').value
        self.ang = self.get_parameter('angular_speed').value
        self.dir = (0, 0)

    def publish(self):
        msg = Twist()
        msg.linear.x = self.dir[0] * self.lin
        msg.angular.z = self.dir[1] * self.ang
        self.pub.publish(msg)

    def handle_key(self, key):
        if key in BINDINGS:
            self.dir = BINDINGS[key]
        elif key in ('+', '='):
            self.lin, self.ang = self.lin * 1.1, self.ang * 1.1
        elif key in ('-', '_'):
            self.lin, self.ang = self.lin * 0.9, self.ang * 0.9
        else:
            return
        print(f'\r방향={self.dir}  linear={self.lin:.2f} m/s  angular={self.ang:.2f} rad/s   ',
              end='', flush=True)


def main():
    if not sys.stdin.isatty():
        print('키보드 입력을 받을 수 없습니다. 별도 터미널에서 `ros2 run robot_car wasd_teleop` 로 실행하세요.')
        return

    rclpy.init()
    node = WasdTeleop()
    settings = termios.tcgetattr(sys.stdin)
    print(HELP)
    try:
        tty.setraw(sys.stdin.fileno())
        while rclpy.ok():
            # 0.1초마다 키 확인 + 현재 명령을 계속 발행(10Hz)
            # -> 로봇 쪽 타임아웃(명령 끊기면 정지)이 동작하지 않게 유지
            ready, _, _ = select.select([sys.stdin], [], [], 0.1)
            if ready:
                key = sys.stdin.read(1).lower()
                if key == '\x03':  # Ctrl+C (raw 모드에서는 신호가 아니라 글자로 들어옴)
                    break
                node.handle_key(key)
            node.publish()
    except KeyboardInterrupt:
        pass  # launch 종료 등으로 외부에서 SIGINT 를 받은 경우
    finally:
        # 터미널 복구를 가장 먼저 (아래에서 오류가 나도 터미널이 망가지지 않게)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        try:
            node.dir = (0, 0)
            node.publish()  # 종료 시 정지 명령
        except Exception:
            pass  # 이미 ROS 가 종료된 경우
        node.destroy_node()
        rclpy.try_shutdown()
        print()


if __name__ == '__main__':
    main()
