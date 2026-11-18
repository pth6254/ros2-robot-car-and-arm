# 가상 VL53L0X ToF 거리 센서.
#
#   TF(world -> tof_sensor) 로 센서 위치/방향을 구하고, 가상 물체(축 정렬 상자 + 바닥)까지
#   광선을 쏴서 거리를 계산해 /tof/range (sensor_msgs/Range) 로 발행한다.
#   실물 센서를 달면 bridge 가 같은 토픽을 발행하므로, 이 노드만 빼면 된다.
#
#   VL53L0X 흉내:
#     - 측정 범위 0.03 ~ 1.2 m (범위 밖: +inf / -inf, REP 117)
#     - 시야각 약 25도 -> 중심 광선 + 원뿔 둘레 광선 중 가장 가까운 값
#     - 노이즈: 표준편차 3 mm + 거리의 1 %
#     - 약 30 Hz

import math
import random

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import Range
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


def quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def ray_box(origin, d, center, size):
    """광선-상자(축 정렬) 교차 거리. 없으면 None."""
    tmin, tmax = -math.inf, math.inf
    for i in range(3):
        lo = center[i] - size[i] / 2.0
        hi = center[i] + size[i] / 2.0
        if abs(d[i]) < 1e-12:
            if origin[i] < lo or origin[i] > hi:
                return None
            continue
        t1 = (lo - origin[i]) / d[i]
        t2 = (hi - origin[i]) / d[i]
        if t1 > t2:
            t1, t2 = t2, t1
        tmin, tmax = max(tmin, t1), min(tmax, t2)
        if tmin > tmax:
            return None
    if tmax < 0:
        return None
    return tmin if tmin >= 0 else 0.0  # 센서가 상자 안이면 0


class ToFSim(Node):

    def __init__(self):
        super().__init__('tof_sim')
        self.declare_parameter('world_frame', 'base_sub_assembly')
        self.declare_parameter('sensor_frame', 'tof_sensor')
        self.declare_parameter('rate', 30.0)
        self.declare_parameter('min_range', 0.03)
        self.declare_parameter('max_range', 1.2)
        self.declare_parameter('field_of_view_deg', 25.0)
        self.declare_parameter('cone_rays', 8)          # 0 이면 중심 광선 하나만
        self.declare_parameter('noise_std_m', 0.003)
        self.declare_parameter('noise_ratio', 0.01)
        self.declare_parameter('ground_plane', True)
        self.declare_parameter('box_names', [''])
        self.declare_parameter('box_centers', [0.0])
        self.declare_parameter('box_sizes', [0.0])

        gp = self.get_parameter
        self.world = gp('world_frame').value
        self.sensor = gp('sensor_frame').value
        self.min_r = gp('min_range').value
        self.max_r = gp('max_range').value
        self.fov = math.radians(gp('field_of_view_deg').value)
        self.noise_std = gp('noise_std_m').value
        self.noise_ratio = gp('noise_ratio').value
        self.ground = gp('ground_plane').value

        names = [n for n in gp('box_names').value if n]
        c, s = gp('box_centers').value, gp('box_sizes').value
        if len(c) != 3 * len(names) or len(s) != 3 * len(names):
            raise ValueError('box_centers / box_sizes 는 상자 개수 x 3 개여야 합니다')
        self.boxes = [(names[i], c[3 * i:3 * i + 3], s[3 * i:3 * i + 3])
                      for i in range(len(names))]

        # 센서 프레임 기준 광선 방향들 (+x 가 정면)
        self.rays = [(1.0, 0.0, 0.0)]
        n = gp('cone_rays').value
        half = self.fov / 2.0
        for k in range(n):
            a = 2 * math.pi * k / n
            self.rays.append((math.cos(half),
                              math.sin(half) * math.cos(a),
                              math.sin(half) * math.sin(a)))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.range_pub = self.create_publisher(Range, 'tof/range', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(MarkerArray, 'sim_world/markers', latched)
        self.publish_world()
        self.create_timer(1.0 / gp('rate').value, self.update)
        self.get_logger().info(
            f'가상 ToF 센서 시작: 상자 {len(self.boxes)}개, 바닥={self.ground}, '
            f'범위 {self.min_r}~{self.max_r} m, 시야각 {math.degrees(self.fov):.0f}도')

    def cast(self, origin, d):
        best = math.inf
        for _, center, size in self.boxes:
            t = ray_box(origin, d, center, size)
            if t is not None and t < best:
                best = t
        if self.ground and d[2] < -1e-9:
            t = -origin[2] / d[2]
            if 0 <= t < best:
                best = t
        return best

    def update(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.world, self.sensor, Time(),
                                                 timeout=Duration(seconds=0.0))
        except TransformException:
            return  # 아직 TF 없음 (트윈 시작 전)
        t = tf.transform.translation
        origin = (t.x, t.y, t.z)
        R = quat_to_matrix(tf.transform.rotation)

        dist = math.inf
        for r in self.rays:
            d = tuple(R[i][0] * r[0] + R[i][1] * r[1] + R[i][2] * r[2] for i in range(3))
            dist = min(dist, self.cast(origin, d))

        if math.isfinite(dist):
            dist += random.gauss(0.0, self.noise_std + self.noise_ratio * dist)
            if dist < self.min_r:
                dist = -math.inf
            elif dist > self.max_r:
                dist = math.inf

        msg = Range()
        msg.header.stamp = tf.header.stamp
        msg.header.frame_id = self.sensor
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = self.fov
        msg.min_range = self.min_r
        msg.max_range = self.max_r
        msg.range = float(dist)
        self.range_pub.publish(msg)

    def publish_world(self):
        arr = MarkerArray()
        for i, (name, center, size) in enumerate(self.boxes):
            m = Marker()
            m.header.frame_id = self.world
            m.ns = 'sim_world'
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = center
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = size
            m.color.r, m.color.g, m.color.b, m.color.a = 0.9, 0.6, 0.2, 0.35
            arr.markers.append(m)
            label = Marker()
            label.header.frame_id = self.world
            label.ns = 'sim_world_labels'
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x, label.pose.position.y = center[0], center[1]
            label.pose.position.z = center[2] + size[2] / 2.0 + 0.02
            label.pose.orientation.w = 1.0
            label.scale.z = 0.025
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = name
            arr.markers.append(label)
        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = ToFSim()
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
