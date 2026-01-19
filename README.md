# Basilisk-ROS 2 Bridge

A lightweight, open-source communication bridge between the Basilisk astrodynamics simulation framework and ROS 2, enabling real-time, bidirectional data exchange.

![bsk_bridge_rviz_demo_compressed](https://github.com/user-attachments/assets/11cca7af-d8b4-443f-aed6-494e996d0d6c)

This repository provides a ROS 2 interface for the [Basilisk astrodynamics framework](https://hanspeterschaub.info/basilisk/) that enables spacecraft simulations to interact directly with ROS 2 nodes without requiring any modifications to Basilisk's core. The bridge connects Basilisk's high-fidelity spacecraft dynamics and simulation timing with ROS 2's distributed middleware.

Spacecraft states generated in Basilisk are published as ROS 2 topics, while commands such as forces, torques, or thruster states can be sent from ROS 2 back into the simulation. This allows standard ROS 2 tools and workflows (e.g., rosbag, RViz, PlotJuggler) to be applied directly to spacecraft simulations, and supports modular autonomy, estimation, control, and monitoring software running externally in ROS 2.

The bridge supports single- and multi-spacecraft scenarios using namespace-aware topic conventions, enabling scalable setups such as formation flying and coordinated control.

## Citation (preprint)
If you use this package in academic work, please cite:

E. Krantz, N. N. Chan, G. Tibert, H. Mao, C. Fuglesang,  
*Bridging the Basilisk Astrodynamics Framework with ROS 2 for Modular Spacecraft Simulation and Hardware Integration*,  
arXiv preprint arXiv:2512.09833, 2025.  
https://arxiv.org/abs/2512.09833

The paper introduces the bridge architecture and demonstrates it in a leader–follower formation-flying scenario using ROS 2-based NMPC, deployed identically in both Basilisk simulation and on a physical spacecraft testbed.

```
@article{krantz2025bridging,
  title={Bridging the Basilisk Astrodynamics Framework with ROS 2 for Modular Spacecraft Simulation and Hardware Integration},
  author={Krantz, Elias and Chan, Ngai Nam and Tibert, Gunnar and Mao, Huina and Fuglesang, Christer},
  journal={arXiv preprint arXiv:2512.09833},
  year={2025}
}
```

## Components

- **Bridge Node** (`bsk-ros2-bridge.py`) - ROS 2 node that handles ZMQ<->ROS 2 message conversion
- **Handler Module** (`bsk_module/rosBridgeHandler.py`) - Basilisk module for scenario integration
- **Example Scenarios** - Four example configurations demonstrating single/multi-spacecraft control

## Tested Configurations

This package has been successfully built and exercised on the following configurations:

| Environment                                | ROS 2 Distro             | Basilisk Build  | Details & Notes                                                     |
|--------------------------------------------|--------------------------|-----------------|----------------------------------------------------------------------|
| Ubuntu 22.04 LTS                           | ROS 2 Humble             | Linux           | Native desktop─full build, CLI tools, and launch files tested        |
| Ubuntu 24.04 LTS                           | ROS 2 Jazzy              | Linux           | Native desktop─full build, CLI tools, and launch files tested        |
| Windows (via WSL, Ubuntu 24.04)            | ROS 2 Rolling            | Linux           | Runs within WSL; build, topic communication, and launch verified     |

**Known limitations**  
- WSL environment requires correction for ROS 2 on [locale settings, e.g. from ROS 2 Rolling](https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debs.html#set-locale).
- Minor latency observed with GUI tools (e.g., BSK-Vizard) over WSL; CLI tools operate normally.  

## Quick Start

### 0) Prerequisites

- [Basilisk](https://avslab.github.io/basilisk/Install.html)
- ROS 2 (e.g. [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html), [Rolling](https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debs.html))

After installation of **Basilisk**, it is recommended to export the user path of Basilisk to `.bashrc` as:
```bash
export BSK_PATH=your_BSK_path
```

### 1) Install dependencies

```bash
# Build and source the custom ROS 2 messages
cd your_ros2_workspace/src
git clone https://github.com/DISCOWER/bsk-msgs.git
cd ..
colcon build --packages-select bsk-msgs
source install/setup.bash

# Build and source this package
cd your_ros2_workspace/src
git clone https://github.com/DISCOWER/bsk-ros2-bridge.git
cd ..
colcon build --packages-select bsk-ros2-bridge
source install/setup.bash

# Python dependencies for the bridge + examples
cd your_ros2_workspace/src/bsk-ros2-bridge
pip install -r requirements.txt

```

### 2) Run an example

```bash
# Terminal 1: start the bridge
ros2 launch bsk-ros2-bridge bridge.launch.py

# Terminal 2: start a Basilisk scenario (requires the BSK environment)
source $BSK_PATH/.venv/bin/activate
python examples/scenarioRosBasic_da.py

# Terminal 3: send commands to Basilisk
# The included `dummy-data-processor` is a minimal test tool (not a real controller).
# For closed-loop control, see the BSK-ROS 2 MPC Controller:
# https://github.com/DISCOWER/bsk-ros2-mpc
ros2 run bsk-ros2-bridge dummy-data-processor --ros-args -r __ns:=/bskSat0 -p mode:=da
```

## Examples

| Scenario | Description | Control mode |
|----------|-------------|--------------|
| `scenarioRosBasic_da.py` | Single spacecraft (no Earth) | `da` |
| `scenarioRosBasic_wrench.py` | Single spacecraft (no Earth) | `wrench` |
| `scenarioRosOrbit_da.py` | Spacecraft orbiting Earth | `da` |
| `scenarioRosOrbit_wrench.py` | Spacecraft orbiting Earth | `wrench` |
| `scenarioRosLeaderFollowerBasic_wrench.py` | 1 leader + 2 followers (no Earth) | `wrench` |
| `scenarioRosLeaderFollowerOrbit_wrench.py` | 1 leader + 2 followers (orbit) | `wrench` |

### Control Modes

**Direct allocation (`da`)**
- Commands individual thrusters
- Topic: `/<namespace>/bsk/in/thr_array_cmd_force`

**Wrench control (`wrench`)**
- Commands 3D forces and torques, mapped to thrusters by Basilisk
- Topics: `/<namespace>/bsk/in/cmd_force`, `/<namespace>/bsk/in/cmd_torque`

### Multi-Spacecraft Usage

The scenarios `scenarioRosOrbit_da.py` and `scenarioRosOrbit_wrench.py` support any number of spacecraft. Example (run an orbit scenario with multiple spacecraft):

```bash
# Terminal 1: start the bridge
ros2 launch bsk-ros2-bridge bridge.launch.py

# Terminal 2: start the Basilisk scenario
source $BSK_PATH/.venv/bin/activate
python examples/scenarioRosOrbit_da.py

# Terminal 3: start one controller per spacecraft
ros2 run bsk-ros2-bridge dummy-data-processor --ros-args -r __ns:=/bskSat0 -p mode:=da
ros2 run bsk-ros2-bridge dummy-data-processor --ros-args -r __ns:=/bskSat1 -p mode:=da
```

### Time Synchronization

The bridge publishes Basilisk simulation time to `/clock` using `rosgraph_msgs/Clock` when the simulation runs in non-realtime mode (i.e., `accelFactor` is not 1). This allows ROS 2 nodes to synchronize with accelerated or slowed simulations. In these cases, your ROS 2 nodes must be initialized with `use_sim_time=true`.

The clock update interval can be configured via the `clock_timestep` launch argument (default: 0.01s):

```bash
ros2 launch bsk-ros2-bridge bridge.launch.py clock_timestep:=0.1
```

**Note:** When running in realtime mode (`accelFactor=1`), the `/clock` topic is not published and nodes should use system time (`use_sim_time:=false`).

## Development

### Integrating RosBridgeHandler in Scenarios

```python
# Import and setup
from bsk_module.rosBridgeHandler import RosBridgeHandler
ros_bridge = RosBridgeHandler()
ros_bridge.ModelTag = "ros_bridge"

# Add publishers (Basilisk -> ROS 2)
ros_bridge.add_ros_publisher('SCStatesMsgPayload', 'SCStatesMsgIn', 'sc_states', 'bskSat', max_rate=100.0)

# Add subscribers (ROS 2 -> Basilisk)  
ros_bridge.add_ros_subscriber('THRArrayCmdForceMsgPayload', 'THRArrayCmdForceMsgOut', 'thr_array_cmd_force', 'bskSat')

# Connect to Basilisk messages
ros_bridge.bskSat.SCStatesMsgIn.subscribeTo(scObject.scStateOutMsg)
thrFiringSchmittObj.thrForceInMsg.subscribeTo(ros_bridge.bskSat.THRArrayCmdForceMsgOut)

# Finally, add module to simulation task
scSim.AddModelToTask(simTaskName, ros_bridge)
```

### Registering ROS 2 Publishers and Subscribers

```python
# Publisher: Sends Basilisk data to ROS 2
ros_bridge.add_ros_publisher(msg_type_name, handler_name, topic_name, namespace, max_rate=None)

# Subscriber: Receives ROS 2 commands in Basilisk
ros_bridge.add_ros_subscriber(msg_type_name, handler_name, topic_name, namespace)
```

**Parameters:**
- `msg_type_name` - Basilisk message type (e.g., `'SCStatesMsgPayload'`, `'CmdForceBodyMsgPayload'`)
- `handler_name` - Internal message handler (e.g., `'SCStatesMsgIn'`, `'CmdForceBodyMsgOut'`)
- `topic_name` - ROS 2 topic name (e.g., `'sc_states'`, `'cmd_force'`)
- `namespace` - Spacecraft identifier (e.g., `'bskSat'`, `'bskSat0'`)
- `max_rate` - (Publishers only) optional maximum publishing rate (Hz). If not specified, publishes at the task rate.

### Topic Structure

Topics follow the pattern: `/<namespace>/bsk/<in|out>/<topic_name>`

**Common Topics:**
- `/clock` - Simulation time synchronization (only in non-realtime mode)
- `/<namespace>/bsk/out/sc_states` - Spacecraft states  
- `/<namespace>/bsk/in/thr_array_cmd_force` - Thruster commands
- `/<namespace>/bsk/in/cmd_force` - Force commands
- `/<namespace>/bsk/in/cmd_torque` - Torque commands

### Required QoS

All topics exchanged by the bridge use the following ROS 2 Quality of Service (QoS) settings:

- Reliability: **BEST_EFFORT**
- Durability: **VOLATILE**
- History: **KEEP_LAST**
- Queue depth: **1**

This configuration prioritizes low-latency communication and ensures that only the most recent message is delivered, suitable for high-rate spacecraft simulation data.

### Port Configuration

Default ZMQ ports:
- 5550: Basilisk → Bridge
- 5551: Bridge → Basilisk
- 5552: Heartbeat

**Launch arguments:**
| Argument | Default | Description |
|----------|---------|-------------|
| `sub_port` | 5550 | ZMQ subscriber port for receiving data from Basilisk |
| `pub_port` | 5551 | ZMQ publisher port for sending commands to Basilisk |
| `heartbeat_port` | 5552 | ZMQ heartbeat port for connection monitoring |
| `clock_timestep` | 0.01 | ROS clock update interval in seconds |
| `namespace` | '' | Namespace for this bridge instance |

**Custom ports:**
```bash
ros2 launch bsk-ros2-bridge bridge.launch.py sub_port:=6550 pub_port:=6551 heartbeat_port:=6552
```

```python
ros_bridge = RosBridgeHandler(send_port=6550, receive_port=6551, heartbeat_port=6552)
```

**RosBridgeHandler parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `ModelTag` | `'ros_bridge'` | Module identifier for logging |
| `send_port` | 5550 | ZMQ port for sending data to bridge (BSK → ROS 2) |
| `receive_port` | 5551 | ZMQ port for receiving data from bridge (ROS 2 → BSK) |
| `heartbeat_port` | 5552 | ZMQ port for heartbeat monitoring |
| `accelFactor` | NaN | Simulation speed factor; set to enable `/clock` publishing |

## Features

- **Dynamic Namespace Support** - Automatically creates ROS 2 topics for multiple spacecraft
- **Real-time Communication** - Optimized for low-latency spacecraft control
- **Heartbeat Monitoring** - Ensures connection health between Basilisk and ROS 2
- **Automatic Topic Creation** - Bridge discovers and creates topics based on Basilisk messages
- **Context Manager Support** - Automatic resource cleanup on shutdown
- **Custom Message Types** - Uses `bsk_msgs` package that mirrors Basilisk's internal message structure
- **Configurable Ports** - Runtime port configuration for multiple bridge instances

## Troubleshooting

**Missing message types**: ensure `bsk_msgs` is built and sourced

**Port conflicts**: check ports status with: 
```bash
lsof -i :5550 && lsof -i :5551 && lsof -i :5552
```
Kill occupied ports via PID if needed:
```bash
kill -9 {port PID}
```

**Connection issues**: ensure bridge is running, verify BSK environment is activated

## References

- [Basilisk Astrodynamics Simulation](https://hanspeterschaub.info/basilisk/)
- [ROS 2 Documentation](https://www.ros.org/)
- [ZeroMQ Documentation](https://zeromq.org/)
- [BSK ROS 2 Messages](https://github.com/DISCOWER/bsk-msgs.git)
- [BSK ROS 2 MPC Controller](https://github.com/DISCOWER/bsk-ros2-mpc.git)
