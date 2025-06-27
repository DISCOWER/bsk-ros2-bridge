# Basilisk-ROS2 Bridge

A ROS2 package that bridges the open-source astrodynamics simulator [Basilisk (BSK)](https://hanspeterschaub.info/basilisk/) with ROS2, enabling seamless communication between spacecraft simulation and ROS2-based control systems.

The bridge uses the [ZMQ Protocol](https://zguide.zeromq.org/docs/chapter2/#Handling-Errors-and-ETERM) for high-performance, low-latency communication, similar to Basilisk's Vizard visualization tool.

## Package Components

This package contains three main components:

1. **ROS2 Bridge Node** (`bsk_ros_bridge.py`) - **Main component** that handles ZMQ↔ROS2 message conversion and routing
2. **Basilisk Module** (`ros_bridge_handler.py`) - Basilisk-side module for bridge communication  
3. **Example Data Processor** (`ex_data_processor.py`) - Demonstrates ROS2-side spacecraft control (MPC placeholder)

## Prerequisites

### For Bridge and Data Processor
Ensure you have ROS2 installed and sourced:
```bash
source /opt/ros/humble/setup.bash  # or your ROS2 distribution
```

### For Basilisk Module Only
The Basilisk module requires Basilisk's Python environment:
```bash
cd $BSK_REPO  # replace with your Basilisk repository path
source .venv/bin/activate
```

## Installation

Build the package using colcon:
```bash
cd your_ros2_workspace
colcon build --packages-select bsk_ros_bridge
source install/setup.bash
```

## Usage

### Launch the Bridge
Start the main bridge node that handles all ZMQ↔ROS2 communication:
```bash
ros2 launch bsk_ros_bridge bridge.launch.py
```

This is the core component that enables communication between Basilisk and ROS2. Once running, it will automatically create ROS2 topics for any namespaces that Basilisk modules connect with.

## Examples

### Example Data Processor
Launch the example spacecraft controller (as a stand-in for proper MPC):

**Single spacecraft (default namespace):**
```bash
ros2 run bsk_ros_bridge example_data_processor
```

**Multiple spacecraft with different namespaces:**
```bash
# Terminal 1 - Spacecraft 1
ros2 run bsk_ros_bridge example_data_processor --ros-args -r __ns:=/test_sat1

# Terminal 2 - Spacecraft 2  
ros2 run bsk_ros_bridge example_data_processor --ros-args -r __ns:=/test_sat2

# Terminal 3 - Spacecraft 3
ros2 run bsk_ros_bridge example_data_processor --ros-args -r __ns:=/my_satellite
```

Each instance will automatically create its own topic tree under the specified namespace.

### Basilisk Module Example
Import and use the `ros_bridge_handler.py` module in your Basilisk scenarios. 

**Note:** This requires Basilisk's Python environment to be activated first:
```bash
cd $BSK_REPO && source .venv/bin/activate
python examples/ex_ros_bridge_handler.py
```

## Configuration

Topics are configured in `config/topics.yaml`. The bridge currently supports:

**Published Topics (Basilisk → ROS2):**
- `/[namespace]/state` - Spacecraft position and attitude (PoseStamped)
- `/[namespace]/velocity` - Linear and angular velocity (TwistStamped) 
- `/[namespace]/mass_properties` - Mass and inertia properties (InertiaStamped)

**Subscribed Topics (ROS2 → Basilisk):**
- `/[namespace]/control_wrench` - Control forces and torques (WrenchStamped)
- `/[namespace]/attitude_setpoint` - Desired attitude (PoseStamped)
- `/[namespace]/thrust_command` - Thrust vector commands (Vector3Stamped)

**ZMQ Port Configuration:**
- **5550** - Basilisk → Bridge (simulation data)
- **5551** - Bridge → Basilisk (control commands)  
- **5552** - Bridge heartbeat (health monitoring)
- **5553** - Kill request (graceful shutdown)

## Architecture

```
┌──────────────────┐    ZMQ     ┌──────────────────┐    ROS2    ┌──────────────────┐
│                  │   Sockets  │                  │   Topics   │                  │
│     Basilisk     │◄──────────►│    Bridge Node   │◄──────────►│       ROS2       │
│    Simulation    │            │ (bsk_ros_bridge) │            │     Ecosystem    │
│                  │            │                  │            │                  │
└──────────────────┘            └──────────────────┘            └──────────────────┘
         ▲                               ▲                               ▲
         │                               │                               │
         ▼                               ▼                               ▼
┌──────────────────┐            ┌──────────────────┐            ┌──────────────────┐
│   ros_bridge_    │            │  Configuration   │            │    ROS2 Nodes    │
│   handler.py     │            │  (topics.yaml)   │            │ (data processors,│
└──────────────────┘            └──────────────────┘            │ controllers, ...)│
                                                                └──────────────────┘
```

## Features

- **Dynamic Namespace Support** - Automatically creates ROS2 topics for multiple spacecraft
- **Real-time Communication** - Optimized QoS profiles for low-latency spacecraft control
- **Heartbeat Monitoring** - Ensures connection health between Basilisk and ROS2
- **Message Caching** - Efficient message type loading and conversion
- **Context Manager Support** - Automatic resource cleanup on shutdown

## Development and Testing

### Testing the Basilisk Module
The `ros_bridge_handler.py` module should be imported into your Basilisk scenarios. 

**Note:** Testing requires Basilisk's Python environment:
```bash
cd $BSK_REPO && source .venv/bin/activate
python examples/ex_ros_bridge_handler.py
```

### Custom Configuration
Modify `config/topics.yaml` to add custom message types or topics for your specific spacecraft configuration.

### Multiple Spacecraft
The bridge supports multiple spacecraft through namespacing. Each Basilisk instance can specify its own namespace, and the bridge will automatically create separate ROS2 topic trees.

## Troubleshooting

### Port Conflicts
If you encounter ZMQ port conflicts, check for occupied ports:
```bash
# Check for occupied ports (default: 5550, 5551, 5552, 5553)
lsof -i :5550
lsof -i :5551
lsof -i :5552
lsof -i :5553
```

Kill conflicting processes:
```bash
kill -9 ${PID}  # Replace ${PID} with the actual process ID
```

### Bridge Connection Issues
- Ensure Basilisk's virtual environment is activated
- Check that both bridge and Basilisk module are using the same port configuration
- Monitor bridge logs for heartbeat and connection status

### Message Type Errors
- Verify `topics.yaml` configuration matches your message requirements
- Check that all required ROS2 message packages are installed
- Ensure message type strings follow the format: `package_name::msg::MessageType`

## Contributing

When contributing to this package:
1. Follow ROS2 Python style guidelines
2. Update `topics.yaml` for new message types
3. Add corresponding conversion functions in the bridge
4. Update tests and documentation

## License

This package is licensed under BSD-3-Clause.

## References

- [Basilisk Astrodynamics Simulation](https://hanspeterschaub.info/basilisk/)
- [ZeroMQ Messaging Library](https://zeromq.org/)
- [ROS2 Documentation](https://docs.ros.org/)
