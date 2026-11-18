"""실제 차량 실행: 명령 하나로 차량 연결 + RViz + 키보드 조종 창까지 전부 띄움.

    ros2 launch robot_car bringup.launch.py                              # ★ 실제 차량, ESP32 무선 (기본)
    ros2 launch robot_car bringup.launch.py host:=192.168.0.200          # 무선, IP 직접 지정
    ros2 launch robot_car bringup.launch.py mode:=real port:=/dev/ttyUSB0  # ESP32 유선 (USB)
    ros2 launch robot_car bringup.launch.py mode:=real                   # Arduino Uno 유선 (/dev/ttyACM0)
    ros2 launch robot_car bringup.launch.py mode:=sim                    # 시뮬레이터 (= display.launch.py)

※ 시뮬레이터는 display.launch.py 가 기본입니다:  ros2 launch robot_car display.launch.py

선택 인자:
    gui:=false      PyBullet 물리 창 숨김 (sim)
    rviz:=false     RViz 끄기
    teleop:=false   키보드 조종 창 끄기 (직접 실행할 때)
    keys:=toggle    조종 방식: hold(기본) = 누르는 동안만 이동 / toggle = 한 번 누르면 계속 이동
    min_pwm:=80     모터 최소 PWM (real / wifi)
    heading_kp:=0   자이로 직선 주행 보정 끄기 (기본 6.0). 보정 켬/끔 비교할 때 사용 (real / wifi)
    heading_kd:=0.4 보정 흔들림 억제 세기 (real / wifi)

키보드 조종 창이 따로 열립니다. 그 창을 클릭해서 선택한 상태에서 w/a/s/d 를 누르세요.
    keys:=hold   -> 'WASD 조종 (누르는 동안만)' GUI 창 (wasd_hold)
    keys:=toggle -> 'WASD-teleop' 터미널 창 (wasd_teleop)

구성:
    sim  : robot_state_publisher + pybullet_sim + RViz(odom 기준) + teleop
    wifi : robot_state_publisher + tcp_node    + joint_state_publisher + RViz(base_link 기준) + teleop
    real : robot_state_publisher + serial_node + joint_state_publisher + RViz(base_link 기준) + teleop
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import (AndSubstitution, EqualsSubstitution, LaunchConfiguration,
                                  NotSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('robot_car')
    urdf_path = os.path.join(share, 'urdf', 'robot_car.urdf')
    rviz_config = os.path.join(share, 'rviz', 'robot_car.rviz')
    with open(urdf_path) as f:
        robot_description = f.read()

    mode = LaunchConfiguration('mode')
    sim = EqualsSubstitution(mode, 'sim')
    is_sim = IfCondition(sim)
    is_real = IfCondition(EqualsSubstitution(mode, 'real'))
    is_wifi = IfCondition(EqualsSubstitution(mode, 'wifi'))
    is_hw = IfCondition(NotSubstitution(sim))          # real 또는 wifi (실제 차량)
    rviz = LaunchConfiguration('rviz')
    min_pwm = ParameterValue(LaunchConfiguration('min_pwm'), value_type=int)
    heading = {'heading_kp': ParameterValue(LaunchConfiguration('heading_kp'), value_type=float),
               'heading_kd': ParameterValue(LaunchConfiguration('heading_kd'), value_type=float)}

    return LaunchDescription([
        # ---------------- 인자 ----------------
        DeclareLaunchArgument('mode', default_value='wifi', choices=['wifi', 'real', 'sim'],
                              description='wifi = 실제 차량 ESP32 무선(TCP, 기본), '
                                          'real = 실제 차량 유선(USB 시리얼), sim = PyBullet 시뮬레이션'),
        DeclareLaunchArgument('gui', default_value='true', description='PyBullet 물리 창 표시 (sim)'),
        DeclareLaunchArgument('rviz', default_value='true', description='RViz 실행'),
        DeclareLaunchArgument('teleop', default_value='true', description='키보드 조종 창 실행'),
        DeclareLaunchArgument('keys', default_value='hold', choices=['hold', 'toggle'],
                              description='hold = 누르는 동안만 이동, toggle = 한 번 누르면 계속 이동'),
        DeclareLaunchArgument('host', default_value='robot-car.local',
                              description='ESP32 주소 (wifi): robot-car.local 또는 IP'),
        DeclareLaunchArgument('tcp_port', default_value='3333', description='ESP32 TCP 포트 (wifi)'),
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0',
                              description='USB 포트 (real): /dev/ttyACM0 Uno, /dev/ttyUSB0 ESP32'),
        DeclareLaunchArgument('min_pwm', default_value='60', description='모터 최소 PWM (real/wifi)'),
        DeclareLaunchArgument('heading_kp', default_value='6.0',
                              description='자이로 직선 보정 세기 (0 = 끔) (real/wifi)'),
        DeclareLaunchArgument('heading_kd', default_value='0.4', description='직선 보정 흔들림 억제 (real/wifi)'),

        LogInfo(msg=['[bringup] mode=', mode]),

        # ---------------- 공통: 로봇 모델 -> TF ----------------
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),

        # ---------------- sim: 가상 로봇 ----------------
        Node(package='robot_car', executable='pybullet_sim', output='screen', condition=is_sim,
             parameters=[{'gui': ParameterValue(LaunchConfiguration('gui'), value_type=bool),
                          'urdf_path': urdf_path}]),

        # ---------------- wifi: ESP32 무선 (TCP/IP) ----------------
        Node(package='robot_car', executable='tcp_node', output='screen', condition=is_wifi,
             parameters=[{'host': LaunchConfiguration('host'),
                          'port': ParameterValue(LaunchConfiguration('tcp_port'), value_type=int),
                          'min_pwm': min_pwm, **heading}]),

        # ---------------- real: 보드 유선 (USB 시리얼) ----------------
        Node(package='robot_car', executable='serial_node', output='screen', condition=is_real,
             parameters=[{'port': LaunchConfiguration('port'), 'min_pwm': min_pwm, **heading}]),

        # 실제 차량은 바퀴 엔코더가 없어서 바퀴 각도를 0으로 발행 (RViz 에 모델을 띄우기 위해)
        Node(package='joint_state_publisher', executable='joint_state_publisher', condition=is_hw),

        # ---------------- RViz ----------------
        # sim: 시뮬레이터가 위치(odom)를 알려주므로 odom 기준 -> 로봇이 움직이는 게 보임
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config, '-f', 'odom'],
             condition=IfCondition(AndSubstitution(rviz, sim))),
        # 실제 차량: 위치 정보가 없으므로 로봇 기준(base_link) -> 모델만 표시
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config, '-f', 'base_link'],
             condition=IfCondition(AndSubstitution(rviz, NotSubstitution(sim)))),

        # ---------------- 키보드 조종 ----------------
        # hold: 누르는 동안만 이동 (GUI 창, 키 뗌을 감지할 수 있음)
        Node(package='robot_car', executable='wasd_hold', name='wasd_hold',
             condition=IfCondition(AndSubstitution(
                 LaunchConfiguration('teleop'), EqualsSubstitution(LaunchConfiguration('keys'), 'hold')))),
        # toggle: 한 번 누르면 계속 이동 (터미널 창)
        # run_in_terminal.sh: 새 창에서 실행 + launch 종료 시 창도 함께 닫음
        Node(package='robot_car', executable='wasd_teleop', name='wasd_teleop',
             prefix=f'bash {os.path.join(share, "scripts", "run_in_terminal.sh")} WASD-teleop',
             condition=IfCondition(AndSubstitution(
                 LaunchConfiguration('teleop'), EqualsSubstitution(LaunchConfiguration('keys'), 'toggle')))),
    ])
