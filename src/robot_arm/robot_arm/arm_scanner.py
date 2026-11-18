# 로봇팔 ToF 스캐너 (간이 2D LiDAR).
#
#   1) 팔을 스캔 자세로 보낸다 (기본: 팔을 세우고 end +90도 -> 센서가 수평을 봄)
#   2) base 를 base_min_deg ~ base_max_deg 사이에서 step_deg 씩 왕복
#   3) 각 스텝마다 트윈(/joint_states)이 목표에 도착 -> settle_time 대기 -> /tof/range 한 개 수집
#   4) 그 순간의 TF(world -> tof_sensor)로 측정점의 3D 좌표를 계산
#
#   발행
#     /command/joint_states : 팔 명령 (슬라이더 GUI 대신 이 노드가 팔을 움직인다)
#     /scan                 : sensor_msgs/LaserScan, frame 'scan_plane' (센서 높이의 수평면)
#     /tof/points           : sensor_msgs/PointCloud2, 최근 keep_sweeps 회 스캔의 3D 점
#
#   /tof/range 는 시뮬레이션(tof_sim)이든 실물(bridge)이든 상관없이 동작한다.

import math
import time

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState, LaserScan, PointCloud2, Range
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener

from robot_arm.tof_sim import quat_to_matrix

JOINT_NAMES = ['base_shoulder', 'shoulder_arm1', 'arm1_arm2', 'arn2_end_arm']


class ArmScanner(Node):

    def __init__(self):
        super().__init__('arm_scanner')
        self.declare_parameter('scan_pose_deg', [0.0, 0.0, 90.0])  # shoulder, arm1, end
        self.declare_parameter('base_min_deg', -90.0)
        self.declare_parameter('base_max_deg', 90.0)
        self.declare_parameter('step_deg', 2.0)
        self.declare_parameter('settle_time', 0.08)       # s, 도착 후 진동이 멎을 때까지
        self.declare_parameter('tolerance_deg', 1.0)      # 도착 판정
        self.declare_parameter('step_timeout', 3.0)       # s, 도착 못 해도 측정하고 넘어감
        self.declare_parameter('range_timeout', 0.5)      # s, 거리값이 안 오면 측정 실패
        self.declare_parameter('keep_sweeps', 2)
        self.declare_parameter('world_frame', 'base_sub_assembly')
        self.declare_parameter('sensor_frame', 'tof_sensor')
        self.declare_parameter('scan_frame', 'scan_plane')
        self.declare_parameter('plane_tolerance', 0.05)   # m, 이 높이 차 안의 점만 /scan 에 포함

        gp = self.get_parameter
        pose = [math.radians(v) for v in gp('scan_pose_deg').value]
        if len(pose) != 3:
            raise ValueError('scan_pose_deg 는 [shoulder, arm1, end] 3개 값이어야 합니다')
        self.pose = pose
        self.bmin = math.radians(gp('base_min_deg').value)
        self.bmax = math.radians(gp('base_max_deg').value)
        self.step = math.radians(gp('step_deg').value)
        self.settle = gp('settle_time').value
        self.tol = math.radians(gp('tolerance_deg').value)
        self.step_timeout = gp('step_timeout').value
        self.range_timeout = gp('range_timeout').value
        self.keep = max(1, gp('keep_sweeps').value)
        self.world = gp('world_frame').value
        self.sensor = gp('sensor_frame').value
        self.scan_frame = gp('scan_frame').value
        self.plane_tol = gp('plane_tolerance').value

        self.actual = None
        self.last_range = None        # (수신 시각, Range)
        self.target_base = self.bmin
        self.direction = 1
        self.state = 'to_start'
        self.state_time = time.monotonic()
        self.plane_z = None
        self.sweep_points = []
        self.old_sweeps = []
        self.sweep_start = None
        n_bins = int(round(2 * math.pi / self.step))
        self.scan_ranges = [math.inf] * n_bins

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.cmd_pub = self.create_publisher(JointState, 'command/joint_states', 10)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, 'tof/points', 10)
        self.create_subscription(JointState, 'joint_states', self.on_joint_states, 10)
        self.create_subscription(Range, 'tof/range', self.on_range, 10)
        self.tick = 0
        self.create_timer(0.01, self.loop)

        self.get_logger().info(
            f'스캐너 시작: base {math.degrees(self.bmin):.0f}~{math.degrees(self.bmax):.0f}도, '
            f'{math.degrees(self.step):.1f}도 간격, 자세 {gp("scan_pose_deg").value}')

    # ---------------------------------------------------------------- 입력
    def on_joint_states(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if all(n in pos for n in JOINT_NAMES):
            self.actual = [pos[n] for n in JOINT_NAMES]

    def on_range(self, msg):
        self.last_range = (time.monotonic(), msg)

    # ---------------------------------------------------------------- 명령
    def target(self):
        return [self.target_base] + self.pose

    def publish_command(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = self.target()
        self.cmd_pub.publish(msg)

    def arrived(self):
        return self.actual is not None and all(
            abs(a - t) <= self.tol for a, t in zip(self.actual, self.target()))

    def set_state(self, state):
        self.state = state
        self.state_time = time.monotonic()

    # ---------------------------------------------------------------- 메인 루프
    def loop(self):
        if not rclpy.ok():
            return
        self.tick += 1
        if self.tick % 4 == 0:
            self.publish_command()
        now = time.monotonic()
        elapsed = now - self.state_time

        if self.state in ('to_start', 'moving'):
            if self.arrived() or elapsed > self.step_timeout:
                if not self.arrived():
                    self.get_logger().warn(
                        f'목표 도착 실패 (base {math.degrees(self.target_base):.0f}도) '
                        '- 범위 밖 관절이 있는지 확인. 그대로 측정합니다',
                        throttle_duration_sec=5.0)
                if self.state == 'to_start':
                    self.start_sweep()
                self.set_state('settling')
        elif self.state == 'settling':
            if elapsed >= self.settle:
                self.set_state('measuring')
        elif self.state == 'measuring':
            if self.last_range is not None and self.last_range[0] >= self.state_time:
                self.record(self.last_range[1])
                self.next_step()
            elif elapsed > self.range_timeout:
                self.get_logger().warn('거리값 수신 없음 (/tof/range)', throttle_duration_sec=5.0)
                self.record(None)
                self.next_step()

    def start_sweep(self):
        self.sweep_points = []
        self.sweep_start = time.monotonic()
        self.update_scan_plane()

    def next_step(self):
        nxt = self.target_base + self.direction * self.step
        if nxt > self.bmax + 1e-6 or nxt < self.bmin - 1e-6:
            self.finish_sweep()
            self.direction *= -1
            nxt = self.target_base + self.direction * self.step
            self.start_sweep()
        self.target_base = nxt
        self.publish_command()
        self.set_state('moving')

    def finish_sweep(self):
        hits = sum(1 for p in self.sweep_points if p is not None)
        self.get_logger().info(
            f'스캔 1회 완료: {len(self.sweep_points)}스텝 중 {hits}개 측정점, '
            f'{time.monotonic() - self.sweep_start:.1f}초')
        self.old_sweeps.append([p for p in self.sweep_points if p is not None])
        self.old_sweeps = self.old_sweeps[-(self.keep - 1):] if self.keep > 1 else []

    # ---------------------------------------------------------------- 측정 처리
    def sensor_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.world, self.sensor, Time(),
                                                 timeout=Duration(seconds=0.0))
        except TransformException:
            return None
        t = tf.transform.translation
        return (t.x, t.y, t.z), quat_to_matrix(tf.transform.rotation)

    def update_scan_plane(self):
        pose = self.sensor_pose()
        if pose is None:
            return
        self.plane_z = pose[0][2]
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.world
        tf.child_frame_id = self.scan_frame
        tf.transform.translation.z = self.plane_z
        tf.transform.rotation.w = 1.0
        self.static_tf.sendTransform(tf)

    def record(self, rng):
        point = None
        pose = self.sensor_pose()
        if rng is not None and pose is not None and math.isfinite(rng.range):
            (ox, oy, oz), R = pose
            r = rng.range
            point = (ox + R[0][0] * r, oy + R[1][0] * r, oz + R[2][0] * r)
        self.sweep_points.append(point)

        # LaserScan: scan_plane 원점에서 본 각도 칸에 수평 거리 기록
        if pose is not None and self.plane_z is not None:
            ox, oy, _ = pose[0]
            if point is not None and abs(point[2] - self.plane_z) <= self.plane_tol:
                ang, dist = math.atan2(point[1], point[0]), math.hypot(point[0], point[1])
            else:  # 측정 실패/평면 밖 -> 센서가 보던 방향 칸을 비운다
                ang = math.atan2(pose[1][1][0], pose[1][0][0])
                dist = math.inf
            idx = int(round((ang + math.pi) / self.step)) % len(self.scan_ranges)
            self.scan_ranges[idx] = dist
            self.publish_scan()
        self.publish_cloud()

    def publish_scan(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.scan_frame
        msg.angle_min = -math.pi
        msg.angle_increment = self.step
        msg.angle_max = -math.pi + self.step * (len(self.scan_ranges) - 1)
        msg.range_min = 0.0
        msg.range_max = 2.0
        msg.ranges = [float(r) for r in self.scan_ranges]
        self.scan_pub.publish(msg)

    def publish_cloud(self):
        pts = [p for s in self.old_sweeps for p in s]
        pts += [p for p in self.sweep_points if p is not None]
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.world
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, pts))


def main(args=None):
    rclpy.init(args=args)
    node = ArmScanner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
