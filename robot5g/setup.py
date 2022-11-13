from setuptools import setup
import os
from glob import glob

package_name = 'robot5g'

setup(
    name=package_name,
    version='0.2.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.lua')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.pgm')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nitipon',
    maintainer_email='nitipon2544@gmail.com',
    description='DPU Thesis 2022',
    license='DPU Thesis 2022',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bringup = robot5g.Robot_Bringup:main',
            'mqtt_pub = robot5g.Robot_MQTT_Pub:main',
            'mqtt_sub = robot5g.Robot_MQTT_Sub:main',
            'joystick = robot5g.Robot_Joystick:main',
            'core = robot5g.Robot_Core:main',
            'power = robot5g.Robot_Power:main',
            'waypoint = robot5g.Waypoint:main',
        ],
    },
)
