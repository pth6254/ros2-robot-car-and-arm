# 로봇팔 ToF 스캔 (간이 2D LiDAR) 실행.
#
#   팔:   mode   = sim | serial | auto   (display.launch.py 와 같음)
#   센서: sensor = sim  -> 가상 VL53L0X (tof_sim, config/sim_world.yaml)
#                  real -> 실물 VL53L0X (bridge 가 /tof/range 발행, 펌웨어 지원 후)
#
#   슬라이더 GUI 대신 arm_scanner 가 팔을 움직인다.
#
# 예)
#   ros2 launch robot_arm scan.launch.py                         # 팔 sim + 센서 sim
#   ros2 launch robot_arm scan.launch.py mode:=serial port:=/dev/ttyUSB0   # 실물 팔 + 가상 센서
#   ros2 launch robot_arm scan.launch.py step_deg:=1.0

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('robot_arm')
    world = os.path.join(pkg, 'config', 'sim_world.yaml')
    rviz_config = os.path.join(pkg, 'rviz', 'scan.rviz')

    mode = LaunchConfiguration('mode')
    port = LaunchConfiguration('port')
    sensor = LaunchConfiguration('sensor')
    step_deg = LaunchConfiguration('step_deg')

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='sim', description='팔: sim | serial | auto'),
        DeclareLaunchArgument('port', default_value='auto'),
        DeclareLaunchArgument('sensor', default_value='sim', description='센서: sim | real'),
        DeclareLaunchArgument('step_deg', default_value='2.0', description='스캔 각도 간격'),
        DeclareLaunchArgument('rviz', default_value='true'),

        # 트윈 본체 + 명령 고스트 + bridge (슬라이더/기본 RViz 는 끔)
        # GroupAction 으로 감싸서 gui/rviz:=false 가 이 파일의 rviz 인자로 새지 않게 한다
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'display.launch.py')),
                launch_arguments={'mode': mode, 'port': port, 'gui': 'false',
                                  'rviz': 'false'}.items(),
            ),
        ], scoped=True),

        Node(
            package='robot_arm',
            executable='tof_sim',
            name='tof_sim',
            output='screen',
            parameters=[world],
            condition=IfCondition(PythonExpression(["'", sensor, "' == 'sim'"])),
        ),

        Node(
            package='robot_arm',
            executable='arm_scanner',
            name='arm_scanner',
            output='screen',
            parameters=[{'step_deg': step_deg}],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
