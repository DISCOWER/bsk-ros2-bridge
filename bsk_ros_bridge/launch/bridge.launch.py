from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments for port configuration
    sub_port_arg = DeclareLaunchArgument(
        'sub_port', default_value='5550',
        description='ZMQ subscriber port for receiving from Basilisk'
    )
    pub_port_arg = DeclareLaunchArgument(
        'pub_port', default_value='5551', 
        description='ZMQ publisher port for sending to Basilisk'
    )
    heartbeat_port_arg = DeclareLaunchArgument(
        'heartbeat_port', default_value='5552',
        description='ZMQ heartbeat port'
    )

    # Bridge node with configurable ports
    bridge_node = Node(
        package='bsk_ros_bridge',
        executable='bsk_ros_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'sub_port': LaunchConfiguration('sub_port'),
            'pub_port': LaunchConfiguration('pub_port'), 
            'heartbeat_port': LaunchConfiguration('heartbeat_port')
        }]
    )

    return LaunchDescription([
        sub_port_arg,
        pub_port_arg, 
        heartbeat_port_arg,
        bridge_node
    ])