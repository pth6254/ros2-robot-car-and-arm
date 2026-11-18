"""PyBullet 물리 시뮬레이션 노드 = '가상의 아두이노 + 모터 + 차체'.

    구독: /cmd_vel        (geometry_msgs/Twist)
    발행: /joint_states   (바퀴 회전각 -> RViz에서 바퀴가 도는 모습)
          /odom           (nav_msgs/Odometry, 시뮬레이터의 실제 위치)
          TF odom -> base_link

나중에 실물을 만들면 이 노드 자리에 시리얼 노드만 바꿔 끼우면 됩니다.
teleop 과 /cmd_vel 인터페이스는 그대로입니다.
"""
import os

try:
    import pybullet as p
    import pybullet_data
except ImportError:
    raise SystemExit('pybullet 을 찾을 수 없습니다. venv 를 활성화했는지 확인하세요:\n'
                     '    source <프로젝트 폴더>/env.sh')
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from robot_car.kinematics import twist_to_wheels

PHYSICS_HZ = 240     # PyBullet 권장 물리 스텝
PUBLISH_HZ = 60      # ROS 발행 주기
CMD_TIMEOUT = 0.5    # [s] 이 시간 동안 cmd_vel 이 없으면 정지 (아두이노에도 똑같이 넣을 안전장치)
MOTOR_TORQUE = 0.3   # [N·m] 바퀴당 최대 토크


class PybulletSim(Node):
    def __init__(self):
        super().__init__('pybullet_sim')
        self.declare_parameter('gui', True)
        self.declare_parameter('urdf_path', os.path.join(
            get_package_share_directory('robot_car'), 'urdf', 'robot_car.urdf'))
        gui = self.get_parameter('gui').value
        urdf_path = self.get_parameter('urdf_path').value

        # ---- PyBullet 월드 구성 ----
        p.connect(p.GUI if gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1.0 / PHYSICS_HZ)
        p.loadURDF('plane.urdf')
        self.robot = p.loadURDF(urdf_path, [0, 0, 0.04],
                                flags=p.URDF_USE_INERTIA_FROM_FILE)

        # 조인트 이름 -> 인덱스
        joints = {p.getJointInfo(self.robot, i)[1].decode(): i
                  for i in range(p.getNumJoints(self.robot))}
        self.left = joints['left_wheel_joint']
        self.right = joints['right_wheel_joint']
        caster = joints['caster_joint']   # fixed joint 의 child link 인덱스 = joint 인덱스

        # 바퀴는 잘 붙고, 볼캐스터는 마찰 없이 미끄러지게
        for w in (self.left, self.right):
            p.changeDynamics(self.robot, w, lateralFriction=1.0)
        p.changeDynamics(self.robot, caster, lateralFriction=0.0,
                         spinningFriction=0.0, rollingFriction=0.0)

        # PyBullet 은 base 위치를 '무게중심' 기준으로 돌려주므로
        # URDF 의 base_link 원점으로 되돌리는 변환을 미리 구해둠
        info = p.getDynamicsInfo(self.robot, -1)
        self.com_to_link = p.invertTransform(info[3], info[4])

        self.gui = gui
        self.wl = self.wr = 0.0
        self.last_cmd_time = self.get_clock().now()

        # ---- ROS 인터페이스 ----
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_pub = TransformBroadcaster(self)
        self.create_timer(1.0 / PUBLISH_HZ, self.step)
        self.get_logger().info(f'PyBullet 시뮬레이션 시작 (gui={gui}), /cmd_vel 대기 중')

    def on_cmd_vel(self, msg):
        self.wl, self.wr = twist_to_wheels(msg.linear.x, msg.angular.z)
        self.last_cmd_time = self.get_clock().now()

    def step(self):
        now = self.get_clock().now()
        if (now - self.last_cmd_time).nanoseconds * 1e-9 > CMD_TIMEOUT:
            self.wl = self.wr = 0.0

        # 모터 = 속도제어 (목표 각속도, 최대 토크)
        for j, vel in ((self.left, self.wl), (self.right, self.wr)):
            p.setJointMotorControl2(self.robot, j, p.VELOCITY_CONTROL,
                                    targetVelocity=vel, force=MOTOR_TORQUE)
        for _ in range(PHYSICS_HZ // PUBLISH_HZ):
            p.stepSimulation()

        self.publish_state(now.to_msg())

    def publish_state(self, stamp):
        # 바퀴 조인트 상태
        js = JointState()
        js.header.stamp = stamp
        js.name = ['left_wheel_joint', 'right_wheel_joint']
        states = [p.getJointState(self.robot, j) for j in (self.left, self.right)]
        js.position = [s[0] for s in states]
        js.velocity = [s[1] for s in states]
        self.joint_pub.publish(js)

        # 차체 위치 (무게중심 -> base_link 원점으로 변환)
        com_pos, com_orn = p.getBasePositionAndOrientation(self.robot)
        pos, orn = p.multiplyTransforms(com_pos, com_orn, *self.com_to_link)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = pos
        (tf.transform.rotation.x, tf.transform.rotation.y,
         tf.transform.rotation.z, tf.transform.rotation.w) = orn
        self.tf_pub.sendTransform(tf)

        # 속도는 월드 기준 -> 로봇 기준으로 변환
        lin_w, ang_w = p.getBaseVelocity(self.robot)
        _, inv_orn = p.invertTransform([0, 0, 0], orn)
        lin_b, _ = p.multiplyTransforms([0, 0, 0], inv_orn, lin_w, [0, 0, 0, 1])

        odom = Odometry()
        odom.header = tf.header
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = pos
        odom.pose.pose.orientation = tf.transform.rotation
        odom.twist.twist.linear.x = lin_b[0]
        odom.twist.twist.linear.y = lin_b[1]
        odom.twist.twist.angular.z = ang_w[2]
        self.odom_pub.publish(odom)

        # GUI 카메라가 로봇을 따라다니게
        if self.gui:
            cam = p.getDebugVisualizerCamera()
            p.resetDebugVisualizerCamera(cam[10], cam[8], cam[9], pos)

    def destroy_node(self):
        if p.isConnected():
            p.disconnect()
        super().destroy_node()


def main():
    rclpy.init()
    node = PybulletSim()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):  # Ctrl+C 또는 SIGTERM
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
