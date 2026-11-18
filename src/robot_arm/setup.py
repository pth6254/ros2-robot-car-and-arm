from setuptools import find_packages, setup
import os 
from glob import glob 

package_name = 'robot_arm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pth6254',
    maintainer_email='117342307+pth6254@users.noreply.github.com',
    description='4-DOF SG90 robot arm digital twin (Arduino serial bridge, RViz) and ToF-based 2D scanner',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ros_arm_control = robot_arm.ros_arm_bridge:main',
            'tof_sim = robot_arm.tof_sim:main',
            'arm_scanner = robot_arm.arm_scanner:main',
        ],
    },
)
