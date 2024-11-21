from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    namespace = LaunchConfiguration('namespace', default='/robot')

    # SLAM
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot4_navigation'), 'launch', 'slam.launch.py')
        ),
        launch_arguments={'namespace': namespace}.items()
    )

    # RViz Visualization
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot4_viz'), 'launch', 'view_robot.launch.py')
        ),
        launch_arguments={'namespace': namespace}.items()
    )

    # Teleoperation
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        remappings=[('/cmd_vel', [namespace, '/cmd_vel'])]
    )

    return LaunchDescription([
        slam_launch,
        rviz_launch,
        teleop_node
    ])