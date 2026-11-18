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
# 로봇팔 디지털 트윈 실행.
#
#   [command 네임스페이스]  슬라이더 GUI -> /command/joint_states -> 반투명 '명령' 고스트 (TF: cmd/*)
#   [기본 네임스페이스]      bridge(Arduino/sim) -> /joint_states -> 진한 '실제' 모델  (TF: *)
#
# 예)
#   ros2 launch robot_arm display.launch.py                 # Arduino 있으면 실물, 없으면 sim
#   ros2 launch robot_arm display.launch.py mode:=serial port:=/dev/ttyACM0
#   ros2 launch robot_arm display.launch.py mode:=sim
#

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('robot_arm')
    urdf = os.path.join(pkg, 'urdf', 'ros_arm.urdf')
    rviz_config = os.path.join(pkg, 'rviz', 'ros_arm.rviz')
    calibration = os.path.join(pkg, 'config', 'arm_calibration.yaml')
    with open(urdf, 'r') as f:
        robot_desc = f.read()

    use_sim_time = LaunchConfiguration('use_sim_time')
    mode = LaunchConfiguration('mode')
    port = LaunchConfiguration('port')
    use_gui = LaunchConfiguration('gui')
    use_rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('mode', default_value='auto',
                              description='auto | serial | sim'),
        DeclareLaunchArgument('port', default_value='auto',
                              description='시리얼 포트 (auto = ttyUSB*/ttyACM* 자동 탐색)'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='슬라이더 GUI로 명령 입력'),
        DeclareLaunchArgument('rviz', default_value='true'),

        # ---------- 실제 상태 (트윈 본체) ----------
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
        ),

        # ---------- 명령 고스트 ----------
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace='command',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'robot_description': robot_desc,
                         'frame_prefix': 'cmd/'}],
        ),
        # 고스트의 루트 프레임을 실제 모델의 루트에 겹쳐 놓는다
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='cmd_root_tf',
            arguments=['--frame-id', 'base_sub_assembly',
                       '--child-frame-id', 'cmd/base_sub_assembly'],
        ),
        # 슬라이더 0 = 팔이 똑바로 선 자세 = 서보 90도 (범위는 URDF limit ±90도)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            namespace='command',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_gui),
        ),

        # ---------- 실물 <-> ROS 브리지 ----------
        Node(
            package='robot_arm',
            executable='ros_arm_control',
            name='ros_arm_bridge',
            output='screen',
            parameters=[
                # 관절 방향/영점 보정: config/arm_calibration.yaml
                calibration,
                {'use_sim_time': use_sim_time, 'mode': mode, 'port': port},
            ],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
        ),
    ])
