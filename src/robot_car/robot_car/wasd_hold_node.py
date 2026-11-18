"""키를 '누르고 있는 동안만' 움직이는 키보드 조종 노드 -> /cmd_vel.

    w / s : 전진 / 후진        (누르는 동안만)
    a / d : 좌회전 / 우회전     (누르는 동안만)
    w + a 처럼 동시에 누르면 커브 주행
    방향키(↑ ↓ ← →)도 같은 동작
    + / - : 속도 증가 / 감소
    손을 떼면 즉시 정지. 조종 창 밖을 클릭해도 정지 (안전장치)

터미널은 '키를 뗐다'는 신호를 받을 수 없어서, 작은 GUI 창(tkinter)에서 키를 받습니다.
조종 창을 클릭해서 선택한 상태에서 키를 누르세요.

    ros2 run robot_car wasd_hold
"""
import signal

import rclpy
import tkinter as tk
from geometry_msgs.msg import Twist
from rclpy.node import Node

# 키 -> (선속도 방향, 각속도 방향)
KEYMAP = {
    'w': (1, 0), 'Up': (1, 0),
    's': (-1, 0), 'Down': (-1, 0),
    'a': (0, 1), 'Left': (0, 1),
    'd': (0, -1), 'Right': (0, -1),
}
RELEASE_DEBOUNCE_MS = 40   # 키 자동반복의 '가짜 뗌'을 걸러내는 시간
PUBLISH_MS = 50            # 20Hz 로 /cmd_vel 발행


class WasdHold(Node):
    def __init__(self, root):
        super().__init__('wasd_hold')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.lin = self.declare_parameter('linear_speed', 0.3).value    # m/s
        self.ang = self.declare_parameter('angular_speed', 2.0).value   # rad/s

        self.pressed = set()       # 지금 눌려 있는 키
        self.pending = {}          # 뗌 확인 대기 중인 키 -> after id
        self.root = root

        root.title('WASD 조종 (누르는 동안만)')
        root.geometry('420x230')
        root.configure(bg='#202124')
        tk.Label(root, text='이 창을 클릭한 뒤 키를 누르고 있으세요', fg='#bdc1c6', bg='#202124',
                 font=('Sans', 11)).pack(pady=(12, 2))
        tk.Label(root, text='w/s 전진·후진   a/d 회전   w+a 커브   +/- 속도', fg='#9aa0a6',
                 bg='#202124', font=('Sans', 10)).pack()
        self.keys_label = tk.Label(root, text='', fg='#8ab4f8', bg='#202124', font=('Sans', 26, 'bold'))
        self.keys_label.pack(pady=8)
        self.cmd_label = tk.Label(root, text='', fg='#e8eaed', bg='#202124', font=('Monospace', 11))
        self.cmd_label.pack()
        self.focus_label = tk.Label(root, text='', fg='#f28b82', bg='#202124', font=('Sans', 10))
        self.focus_label.pack(pady=4)

        root.bind('<KeyPress>', self.on_press)
        root.bind('<KeyRelease>', self.on_release)
        root.bind('<FocusOut>', self.on_focus_out)
        root.bind('<FocusIn>', lambda e: self.focus_label.config(text=''))
        root.protocol('WM_DELETE_WINDOW', self.shutdown)
        root.after(PUBLISH_MS, self.tick)
        root.focus_force()

    # ---------- 키 이벤트 ----------
    @staticmethod
    def key_name(event):
        k = event.keysym
        return k.lower() if len(k) == 1 else k

    def on_press(self, event):
        k = self.key_name(event)
        if k in self.pending:                 # 자동반복: 방금 '뗌'이 왔지만 실제로는 계속 누르는 중
            self.root.after_cancel(self.pending.pop(k))
        if k in KEYMAP:
            self.pressed.add(k)
        elif k in ('plus', 'equal', 'KP_Add') and k not in self.pressed:
            self.lin, self.ang = self.lin * 1.1, self.ang * 1.1
            self.pressed.add(k)
        elif k in ('minus', 'underscore', 'KP_Subtract') and k not in self.pressed:
            self.lin, self.ang = self.lin * 0.9, self.ang * 0.9
            self.pressed.add(k)

    def on_release(self, event):
        k = self.key_name(event)
        if k in self.pressed and k not in self.pending:
            # 40ms 안에 다시 눌리지 않으면 진짜로 뗀 것
            self.pending[k] = self.root.after(RELEASE_DEBOUNCE_MS, self.confirm_release, k)

    def confirm_release(self, k):
        self.pending.pop(k, None)
        self.pressed.discard(k)

    def on_focus_out(self, event):
        # 안전장치: 다른 창을 클릭하면 뗌 이벤트를 못 받으므로 전부 뗀 것으로 처리
        for aid in self.pending.values():
            self.root.after_cancel(aid)
        self.pending.clear()
        self.pressed.clear()
        self.focus_label.config(text='창 선택 안 됨 → 정지. 이 창을 클릭하세요')

    # ---------- 발행 ----------
    def current_twist(self):
        lin_dir = sum(KEYMAP[k][0] for k in self.pressed if k in KEYMAP)
        ang_dir = sum(KEYMAP[k][1] for k in self.pressed if k in KEYMAP)
        lin_dir = max(-1, min(1, lin_dir))    # w 와 ↑ 를 같이 눌러도 1배
        ang_dir = max(-1, min(1, ang_dir))
        msg = Twist()
        msg.linear.x = lin_dir * self.lin
        msg.angular.z = ang_dir * self.ang
        return msg

    def tick(self):
        if not rclpy.ok():
            self.root.quit()
            return
        msg = self.current_twist()
        self.pub.publish(msg)
        shown = sorted(k for k in self.pressed if k in KEYMAP)
        self.keys_label.config(text=' + '.join(shown) if shown else '정지')
        self.cmd_label.config(text=f'linear {msg.linear.x:+.2f} m/s   angular {msg.angular.z:+.2f} rad/s\n'
                                   f'(최대 {self.lin:.2f} m/s, {self.ang:.2f} rad/s)')
        self.root.after(PUBLISH_MS, self.tick)

    def shutdown(self):
        self.pressed.clear()
        try:
            self.pub.publish(Twist())   # 종료 시 정지 명령
        except Exception:
            pass
        self.root.quit()


def main():
    rclpy.init()
    root = tk.Tk()
    node = WasdHold(root)
    # Ctrl+C / launch 종료(SIGINT, SIGTERM) -> 창 닫기
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: root.after(0, node.shutdown))
    root.after(200, lambda: None)   # 신호 처리를 위해 이벤트 루프가 주기적으로 깨어나게 함
    try:
        root.mainloop()
    finally:
        try:
            node.pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()
        try:
            root.destroy()
        except tk.TclError:
            pass


if __name__ == '__main__':
    main()
