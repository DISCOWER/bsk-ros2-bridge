from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Bridge node with automatic BSK message handling
    bridge_node = Node(
        package='bsk_ros_bridge',
        executable='bsk_ros_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': False
        }]
    )

    return LaunchDescription([
        bridge_node
    ])