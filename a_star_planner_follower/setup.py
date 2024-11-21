from setuptools import find_packages, setup
import os
from glob import glob
from setuptools import setup

package_name = 'a_star_planner_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
        glob(os.path.join('launch', 'turtlebot4_navigator.launch.py')) +
        glob(os.path.join('launch', 'view_robot.launch.py')))
        ,
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', 'robot.rviz'))),
        (os.path.join('share', package_name, 'maps'), [os.path.join('maps', 'sync_classroom_map.pgm'), os.path.join('maps', 'sync_classroom_map.yaml')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shashwat',
    maintainer_email='smudugur@purdue.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'auto_navigator = task_7.a_star_planner_follower:main'
        ],
    },
)



