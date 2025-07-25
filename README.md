# Basilisk-ROS2 Bridge

A ROS2 package that bridges the [Basilisk astrodynamics simulator](https://hanspeterschaub.info/basilisk/) with ROS2, enabling real-time spacecraft simulation and control. Uses ZeroMQ for high-performance, low-latency communication.

## Components

- **Bridge Node** (`bsk_ros_bridge.py`) - ROS2 node that handles ZMQ<->ROS2 message conversion
- **Handler Module** (`bsk_module/rosBridgeHandler.py`) - Basilisk module for scenario integration
- **Example Scenarios** - Four example configurations demonstrating single/multi-spacecraft control

## Quick Start

### 1. Install Dependencies

```bash
# Install custom ROS2 messages
cd your_ros2_workspace/src
git clone https://github.com/E-Krantz/bsk_msgs.git
cd ..
colcon build --packages-select bsk_msgs
source install/setup.bash

# Install this package
cd your_ros2_workspace/src
git clone https://github.com/Thomas-Chan-2019/bsk_ros_bridge.git bsk_ros_bridge
cd ..
colcon build --packages-select bsk_ros_bridge
source install/setup.bash

# Install required packages from `requirements.txt`
# This snippet has to be run as GLOBAL pip installation, it might not work within virtual environments (e.g., python venv, conda env).
cd your_ros2_workspace/src/bsk_ros_bridge
pip install -r requirements.txt

```

### 2. Run Example

```bash
# Terminal 1: Start bridge
ros2 launch bsk_ros_bridge bridge.launch.py

# Terminal 2: Start Basilisk simulation (requires BSK environment)
source $BSK_PATH/.venv/bin/activate
python examples/scenarioRosBasic_TH.py

# Terminal 3: Start controller (uses default namespace 'bskSat')
ros2 run bsk_ros_bridge example_data_processor --ros-args -p mode:=direct_allocation
```

## Examples

| Scenario | Type | Control Mode |
|----------|------|--------------|
| `scenarioRosBasic_TH.py` | Single spacecraft | Direct thruster allocation |
| `scenarioRosBasic_wrench.py` | Single spacecraft | Force/torque commands |
| `scenarioRosFormation_TH.py` | Multi-spacecraft | Direct thruster allocation |
| `scenarioRosFormation_wrench.py` | Multi-spacecraft | Force/torque commands |

### Control Modes

**Direct Allocation (`direct_allocation`)**
- Commands 12 individual thrusters (0-1.5N each)
- Topic: `/[namespace]/bsk/in/thr_array_cmd_force`

**Wrench Control (`wrench`)**
- Commands 3D forces and torques, mapped to thrusters by Basilisk
- Topics: `/[namespace]/bsk/in/cmd_force`, `/[namespace]/bsk/in/cmd_torque`

### Multi-Spacecraft Usage

```bash
# Start formation scenario
source $BSK_PATH/.venv/bin/activate
python examples/scenarioRosFormation_TH.py

# Control each spacecraft separately
ros2 run bsk_ros_bridge example_data_processor --ros-args -r __ns:=/bskSat0 -p mode:=direct_allocation
ros2 run bsk_ros_bridge example_data_processor --ros-args -r __ns:=/bskSat1 -p mode:=direct_allocation
```

### Time Synchronization

The bridge automatically publishes Basilisk simulation time to `/clock` using `rosgraph_msgs/Clock`. To enable ROS2 nodes to use simulation time:

```bash
ros2 run your_node --ros-args -p use_sim_time:=true
```

This ensures all ROS2 nodes are synchronized with the Basilisk simulation time instead of system time.

## Development

### Integrating RosBridgeHandler in Scenarios

```python
# Import and setup
from bsk_module.rosBridgeHandler import RosBridgeHandler
ros_bridge = RosBridgeHandler()
ros_bridge.ModelTag = "ros_bridge"

# Add publishers (Basilisk -> ROS2)
ros_bridge.add_ros_publisher('SCStatesMsgPayload', 'SCStatesMsgIn', 'sc_states', 'bskSat')

# Add subscribers (ROS2 -> Basilisk)  
ros_bridge.add_ros_subscriber('THRArrayCmdForceMsgPayload', 'THRArrayCmdForceMsgOut', 'thr_array_cmd_force', 'bskSat')

# Connect to Basilisk messages
ros_bridge.bskSat.SCStatesMsgIn.subscribeTo(scObject.scStateOutMsg)
thrFiringSchmittObj.thrForceInMsg.subscribeTo(ros_bridge.bskSat.THRArrayCmdForceMsgOut)

# Add to simulation task
scSim.AddModelToTask(simTaskName, ros_bridge)
```

### Method Signatures

```python
# Publisher: Sends Basilisk data to ROS2
ros_bridge.add_ros_publisher(msg_type_name, handler_name, topic_name, namespace)

# Subscriber: Receives ROS2 commands in Basilisk
ros_bridge.add_ros_subscriber(msg_type_name, handler_name, topic_name, namespace)
```

**Parameters:**
- `msg_type_name` - Basilisk message type (e.g., `'SCStatesMsgPayload'`, `'CmdForceBodyMsgPayload'`)
- `handler_name` - Internal message handler (e.g., `'SCStatesMsgIn'`, `'CmdForceBodyMsgOut'`)
- `topic_name` - ROS2 topic name (e.g., `'sc_states'`, `'cmd_force'`)
- `namespace` - Spacecraft identifier (e.g., `'bskSat'`, `'bskSat0'`)

### Topic Structure

Topics follow the pattern: `/[namespace]/bsk/[direction]/[topic_name]`

**Common Topics:**
- `/clock` - Simulation time synchronization
- `/[namespace]/bsk/out/sc_states` - Spacecraft states  
- `/[namespace]/bsk/in/thr_array_cmd_force` - Thruster commands
- `/[namespace]/bsk/in/cmd_force` - Force commands
- `/[namespace]/bsk/in/cmd_torque` - Torque commands

### Port Configuration

Default ZMQ ports: 5550 (Basilisk->Bridge), 5551 (Bridge->Basilisk), 5552 (heartbeat)

**Custom ports:**
```bash
ros2 launch bsk_ros_bridge bridge.launch.py sub_port:=6550 pub_port:=6551 heartbeat_port:=6552
```

```python
ros_bridge = RosBridgeHandler(send_port=6550, receive_port=6551, heartbeat_port=6552)
```

## Features

- **Dynamic Namespace Support** - Automatically creates ROS2 topics for multiple spacecraft
- **Real-time Communication** - Optimized for low-latency spacecraft control
- **Heartbeat Monitoring** - Ensures connection health between Basilisk and ROS2
- **Automatic Topic Creation** - Bridge discovers and creates topics based on Basilisk messages
- **Context Manager Support** - Automatic resource cleanup on shutdown
- **Custom Message Types** - Uses `bsk_msgs` package that mirrors Basilisk's internal message structure
- **Configurable Ports** - Runtime port configuration for multiple bridge instances

## Troubleshooting

**Missing message types:** Ensure `bsk_msgs` is built and sourced

**Port conflicts:** Check ports status with: 
```bash
lsof -i :5550 && lsof -i :5551 && lsof -i :5552
# Kill occupied ports via PID if they are not closed properly:
kill -9 {port PID}
```

**Connection issues:** Ensure bridge is running, verify BSK environment is activated

## References

- [Basilisk Astrodynamics Simulation](https://hanspeterschaub.info/basilisk/)
- [ROS2 Documentation](https://www.ros.org/)
- [ZeroMQ Documentation](https://zeromq.org/)
- [BSK ROS2 Messages](https://github.com/E-Krantz/bsk_msgs.git)
