from glob import glob

from setuptools import setup

package_name = 'robot_car'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/scripts', glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pth6254',
    maintainer_email='117342307+pth6254@users.noreply.github.com',
    description='Arduino 2WD differential drive robot simulation',
    license='MIT',
    entry_points={
        'console_scripts': [
            # 실행 이름(ros2 run robot_car ○○○) = 파일(모듈):함수
            'pybullet_sim = robot_car.pybullet_sim_node:main',
            'wasd_teleop = robot_car.wasd_teleop_node:main',
            'wasd_hold = robot_car.wasd_hold_node:main',
            'serial_node = robot_car.serial_node:main',
            'tcp_node = robot_car.tcp_node:main',
        ],
    },
)
