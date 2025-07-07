# Basilisk-ROS2 Bridge

A ROS2 package that bridges the open-source astrodynamics simulator [Basilisk (BSK)](https://hanspeterschaub.info/basilisk/) with ROS2, enabling seamless communication between spacecraft simulation and ROS2-based control systems.

The bridge uses the [ZMQ Protocol](https://zguide.zeromq.org/docs/chapter2/#Handling-Errors-and-ETERM) for high-performance, low-latency communication, similar to Basilisk's Vizard visualization tool.

## Package Components

This package contains three main components:

1. **ROS2 Bridge Node** (`bsk_ros_bridge.py`) - **Main component** that handles ZMQ↔ROS2 message conversion and routing
2. **Basilisk Module** (`rosBridgeHandler.py`) - Basilisk-side module for bridge communication  
3. **Example Data Processor** (`ex_data_processor.py`) - Demonstrates ROS2-side spacecraft control (dummy controller)

## Prerequisites

### Required Dependencies

**Custom ROS2 Messages:**
This bridge requires custom ROS2 message definitions that mirror Basilisk's internal message structure. These messages are automatically converted from Basilisk's `.h` files to ROS2 `.msg` files, ensuring compatibility with Basilisk's internal data structures.

```bash
cd your_ros2_workspace/src
git clone https://github.com/E-Krantz/bsk_msgs.git
cd ..
colcon build --packages-select bsk_msgs
source install/setup.bash
```

### For Bridge and Data Processor
Ensure you have ROS2 installed and sourced:
```bash
source /opt/ros/humble/setup.bash  # or your ROS2 distribution
```

### For Basilisk Module Only
The Basilisk module requires Basilisk's Python environment:
```bash
cd $BSK_PATH  # replace with your Basilisk repository path
source .venv/bin/activate
```

## Installation

Clone the bridge to your ROS2 workspace and build:
```bash
cd your_ros2_workspace/src
git clone https://github.com/Thomas-Chan-2019/srl-ros2-BSK-bridge.git bsk_ros_bridge
cd ..
colcon build --packages-select bsk_ros_bridge
source install/setup.bash
```

## Usage

### Start the Bridge
The bridge must be running before starting any Basilisk simulations:
```bash
ros2 launch bsk_ros_bridge bridge.launch.py
```

This creates the core communication infrastructure between Basilisk and ROS2.

## Examples

### Single Spacecraft

**Terminal 1 - Start the bridge:**
```bash
ros2 launch bsk_ros_bridge bridge.launch.py
```

**Terminal 2 - Run Basilisk simulation (requires BSK environment):**
```bash
source $BSK_PATH/.venv/bin/activate
python examples/ex_ros_bridge_handler.py
```

**Terminal 3 - Run dummy controller:**
```bash
ros2 run bsk_ros_bridge example_data_processor
```

### Multiple Spacecraft

**Terminal 1 - Start the bridge:**
```bash
ros2 launch bsk_ros_bridge bridge.launch.py
```

**Terminal 2 - Run multi-spacecraft Basilisk simulation (requires BSK environment):**
```bash
source $BSK_PATH/.venv/bin/activate
python examples/ex_ros_bridge_handler_multi.py
```

**Terminal 3 - Run dummy controller for test_sat1:**
```bash
ros2 run bsk_ros_bridge example_data_processor --ros-args -r __ns:=/test_sat1
```

**Terminal 4 - Run dummy controller for test_sat2:**
```bash
ros2 run bsk_ros_bridge example_data_processor --ros-args -r __ns:=/test_sat2
```

**Terminal 5 - Run dummy controller for test_sat3:**
```bash
ros2 run bsk_ros_bridge example_data_processor --ros-args -r __ns:=/test_sat3
```

The multi-spacecraft example (`ex_ros_bridge_handler_multi.py`) demonstrates different spacecraft operating at different frequencies:
- **test_sat1**: 100 Hz (0.01s update rate)
- **test_sat2**: 50 Hz (0.02s update rate)  
- **test_sat3**: 200 Hz (0.005s update rate)

All spacecraft share the same ZMQ ports (5550-5552) through the bridge, avoiding port conflicts while maintaining independent control loops.

## Configuration

### Port Configuration

**Default ZMQ Ports:**
- **5550** - Basilisk → Bridge (simulation data)
- **5551** - Bridge → Basilisk (control commands)  
- **5552** - Bridge heartbeat (health monitoring)

**Changing Bridge Ports:**
```bash
# Custom ports for the bridge
ros2 launch bsk_ros_bridge bridge.launch.py sub_port:=6550 pub_port:=6551 heartbeat_port:=6552
```

**Changing Basilisk Module Ports:**
In your Basilisk script, configure the `RosBridgeHandler`:
```python
# Default ports (5550, 5551, 5552)
module = RosBridgeHandler(namespace="test_sat1")

# Custom ports
module = RosBridgeHandler(
    namespace="test_sat1",
    send_port=6550,      # Must match bridge sub_port
    receive_port=6551,   # Must match bridge pub_port
    heartbeat_port=6552  # Must match bridge heartbeat_port
)
```

### Topics

**Published Topics (Basilisk → ROS2):**
- `/[namespace]/bsk/out/sc_states` - Spacecraft state data
- `/bsk_sim_time` - Simulation time synchronization

**Subscribed Topics (ROS2 → Basilisk):**
- `/[namespace]/bsk/in/cmd_force_body` - Control forces
- `/[namespace]/bsk/in/cmd_torque_body` - Control torques

## Features

- **Dynamic Namespace Support** - Automatically creates ROS2 topics for multiple spacecraft
- **Real-time Communication** - Optimized for low-latency spacecraft control
- **Heartbeat Monitoring** - Ensures connection health between Basilisk and ROS2
- **Automatic Topic Creation** - Bridge discovers and creates topics based on Basilisk messages
- **Context Manager Support** - Automatic resource cleanup on shutdown
- **Custom Message Types** - Uses `bsk_msgs` package that mirrors Basilisk's internal message structure
- **Configurable Ports** - Runtime port configuration for multiple bridge instances

## Troubleshooting

### Missing Message Types
If you encounter errors about missing message types:

```bash
# Ensure bsk_msgs is built and sourced
cd your_ros2_workspace
colcon build --packages-select bsk_msgs
source install/setup.bash

# Verify message types are available
ros2 interface list | grep bsk_msgs
```

### Port Conflicts
If you encounter ZMQ port conflicts:

```bash
# Check for occupied ports
lsof -i :5550 :5551 :5552

# Kill conflicting processes if needed
kill -9 ${PID}  # Replace ${PID} with the actual process ID
```

### Bridge Connection Issues
- Ensure the bridge is running before starting Basilisk
- Check that Basilisk's virtual environment is activated
- Verify port configuration matches between bridge and Basilisk module
- Ensure `bsk_msgs` package is built and sourced
- Monitor bridge logs for heartbeat and connection status

## Contributing

When contributing to this package:
1. Follow ROS2 Python style guidelines
2. Add corresponding conversion functions in the bridge for new message types
3. Update `bsk_msgs` package if new Basilisk message types are needed
4. Update tests and documentation

## License

This package is licensed under BSD-3-Clause.

## References

- [Basilisk Astrodynamics Simulation](https://hanspeterschaub.info/basilisk/)
- [ROS2 Documentation](https://www.ros.org/)
- [ZeroMQ Messaging Library](https://zeromq.org/)
- [BSK ROS2 Messages](https://github.com/E-Krantz/bsk_msgs.git)
