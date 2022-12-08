from setuptools import setup
import os
from glob import glob

package_name = 'delta'

setup(
    name=package_name,
    version='0.3.2',
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
            'data_pub = delta.Robot_Pub:main',
            'data_sub = delta.Robot_Sub:main',
            'joystick = delta.Robot_Joystick:main',
            'core = delta.Robot_Core:main',
            'power = delta.Robot_Power:main',
            'waypoint = delta.Robot_Waypoint:main',
        ],
    },
)
