"""ESP32 가 보내는 MPU-6050 보고("IMU,yaw,gz,qw,qx,qy,qz,hold") -> ROS 토픽.

tcp_node(WiFi) 와 serial_node(USB) 가 공통으로 사용합니다.

    /imu                 sensor_msgs/Imu   (frame_id = imu_link) 자세(쿼터니언) + Z축 회전 속도
    /imu/yaw_deg         std_msgs/Float32  방향 [도], 반시계 + (켰을 때 방향이 0)
    /imu/heading_hold    std_msgs/Bool     ESP32 가 지금 직선 주행 보정 중인지

rqt_plot /imu/yaw_deg/data 로 직진할 때 방향이 얼마나 유지되는지 그래프로 볼 수 있습니다.
"""
import math

from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32


class ImuReporter:
    def __init__(self, node):
        self.node = node
        self.pub_imu = node.create_publisher(Imu, 'imu', 10)
        self.pub_yaw = node.create_publisher(Float32, 'imu/yaw_deg', 10)
        self.pub_hold = node.create_publisher(Bool, 'imu/heading_hold', 10)
        self.first = True

    def handle_line(self, text):
        """'IMU,...' 형식이면 발행하고 True, 아니면 False."""
        if not text.startswith('IMU,'):
            return False
        try:
            yaw, gz, qw, qx, qy, qz, hold = text.split(',')[1:8]
            yaw, gz, qw, qx, qy, qz = (float(v) for v in (yaw, gz, qw, qx, qy, qz))
            hold = hold.strip() == '1'
        except ValueError:
            return True                       # 형식이 깨진 줄은 버림
        msg = Imu()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        msg.orientation.w, msg.orientation.x = qw, qx
        msg.orientation.y, msg.orientation.z = qy, qz
        msg.orientation_covariance[0] = msg.orientation_covariance[4] = 0.01
        msg.orientation_covariance[8] = 0.05  # yaw 는 나침반이 없어 천천히 흐를 수 있음
        msg.angular_velocity.z = math.radians(gz)
        msg.angular_velocity_covariance[8] = 0.01
        msg.linear_acceleration_covariance[0] = -1.0   # 가속도는 보내지 않음 (ROS 관례: -1 = 데이터 없음)
        self.pub_imu.publish(msg)
        self.pub_yaw.publish(Float32(data=yaw))
        self.pub_hold.publish(Bool(data=hold))
        if self.first:
            self.first = False
            self.node.get_logger().info(f'IMU 데이터 수신 시작 (yaw={yaw:.1f}°)')
        return True
