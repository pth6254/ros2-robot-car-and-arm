"""시뮬레이터 실행: PyBullet 가상 차량 + RViz + 키보드 조종 창 (실제 차량 없이).

    ros2 launch robot_car display.launch.py                  # ★ 시뮬레이터 (기본)
    ros2 launch robot_car display.launch.py gui:=false       # PyBullet 물리 창 없이 RViz 만
    ros2 launch robot_car display.launch.py mode:=model      # 모델 확인만 (물리 없음, 슬라이더로 바퀴 돌려보기)

선택 인자 (sim):
    gui:=false      PyBullet 물리 창 숨김
    rviz:=false     RViz 끄기
    teleop:=false   키보드 조종 창 끄기
    keys:=toggle    조종 방식: hold(기본) = 누르는 동안만 이동 / toggle = 한 번 누르면 계속 이동

※ 실제 차량은 bringup.launch.py 가 기본입니다:  ros2 launch robot_car bringup.launch.py

구성:
    sim   : bringup.launch.py 를 mode:=sim 으로 불러옴 (시뮬레이터 설정을 한 곳에서만 관리)
            -> robot_state_publisher + pybullet_sim + RViz(odom 기준) + 키보드 조종 창
    model : robot_state_publisher + joint_state_publisher_gui(슬라이더) + RViz(base_link 기준)
            -> URDF(robot_layout.yaml)를 고친 뒤 모양과 좌표축 확인용
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('robot_car')
    with open(os.path.join(share, 'urdf', 'robot_car.urdf')) as f:
        robot_description = f.read()

    mode = LaunchConfiguration('mode')
    is_sim = IfCondition(EqualsSubstitution(mode, 'sim'))
    is_model = IfCondition(EqualsSubstitution(mode, 'model'))

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='sim', choices=['sim', 'model'],
                              description='sim = PyBullet 시뮬레이터 + 키보드 조종 (기본), '
                                          'model = 모델 확인만 (슬라이더)'),
        DeclareLaunchArgument('gui', default_value='true', description='PyBullet 물리 창 표시 (sim)'),
        DeclareLaunchArgument('rviz', default_value='true', description='RViz 실행 (sim)'),
        DeclareLaunchArgument('teleop', default_value='true', description='키보드 조종 창 실행 (sim)'),
        DeclareLaunchArgument('keys', default_value='hold', choices=['hold', 'toggle'],
                              description='hold = 누르는 동안만 이동, toggle = 한 번 누르면 계속 이동 (sim)'),

        # ---------------- sim: bringup 을 시뮬레이션 모드로 ----------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'bringup.launch.py')),
            launch_arguments={'mode': 'sim',
                              'gui': LaunchConfiguration('gui'),
                              'rviz': LaunchConfiguration('rviz'),
                              'teleop': LaunchConfiguration('teleop'),
                              'keys': LaunchConfiguration('keys')}.items(),
            condition=is_sim),

        # ---------------- model: 모델 확인만 (물리 없음) ----------------
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}], condition=is_model),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             condition=is_model),
        Node(package='rviz2', executable='rviz2', condition=is_model,
             arguments=['-d', os.path.join(share, 'rviz', 'robot_car.rviz'), '-f', 'base_link']),
    ])
