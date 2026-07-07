"""
Launch file for BSK-ROS2 Bridge.
Configures ZMQ ports for communication with Basilisk astrodynamics simulator.
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Configure namespace for this bridge instance
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for this bridge instance'
    )
    
    # Configure ZMQ communication ports
    # These must match the ports used by the Basilisk simulation
    pub_port_arg = DeclareLaunchArgument(
        'pub_port', 
        default_value='5550',
        description='ZMQ subscriber port for receiving data from Basilisk'
    )
    
    sub_port_arg = DeclareLaunchArgument(
        'sub_port', 
        default_value='5551', 
        description='ZMQ publisher port for sending commands to Basilisk'
    )
    
    heartbeat_port_arg = DeclareLaunchArgument(
        'heartbeat_port', 
        default_value='5552',
        description='ZMQ heartbeat port for connection monitoring'
    )

    publish_clock_arg = DeclareLaunchArgument(
        'publish_clock',
        default_value='True',
        description='Whether to publish /clock at all'
    )

    clock_rate_arg = DeclareLaunchArgument(
        'clock_rate',
        default_value='1000.0',
        description='Rate (Hz, real/wall-clock time) at which /clock is published, independent of simulation speed'
    )

    # Main bridge node - handles all BSK<->ROS2 message translation
    bridge_node = Node(
        package='bsk-ros2-bridge',
        executable='bsk-ros2-bridge',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{
            'use_sim_time': False,  # BSK manages its own simulation time
            'pub_port': LaunchConfiguration('pub_port'),
            'sub_port': LaunchConfiguration('sub_port'), 
            'heartbeat_port': LaunchConfiguration('heartbeat_port'),
            'publish_clock': LaunchConfiguration('publish_clock'),
            'clock_rate': LaunchConfiguration('clock_rate')
        }]
    )

    return LaunchDescription([
        namespace_arg,
        pub_port_arg,
        sub_port_arg, 
        heartbeat_port_arg,
        publish_clock_arg,
        clock_rate_arg,
        bridge_node
    ])