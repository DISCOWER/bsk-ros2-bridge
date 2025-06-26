from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value=os.path.join(
            get_package_share_directory('bsk_ros_bridge'),
            'config',
            'topics.yaml'
        ),
        description='Path to the topics configuration file'
    )

    # Bridge node only - handles all namespaces dynamically
    bridge_node = Node(
        package='bsk_ros_bridge',
        executable='bsk_ros_bridge',
        parameters=[{
            'config_path': LaunchConfiguration('config_path')
        }],
        output='screen'
    )

    return LaunchDescription([
        config_path_arg,
        bridge_node
    ])