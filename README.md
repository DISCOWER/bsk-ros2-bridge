# Basilisk-ROS 2 Bridge

A lightweight, open-source communication bridge between the Basilisk astrodynamics simulation framework and ROS 2, enabling real-time, bidirectional data exchange.

![bsk_bridge_rviz_demo_compressed](https://github.com/user-attachments/assets/11cca7af-d8b4-443f-aed6-494e996d0d6c)

This repository provides a ROS 2 interface for the [Basilisk astrodynamics framework](https://hanspeterschaub.info/basilisk/) that enables spacecraft simulations to interact directly with ROS 2 nodes without requiring any modifications to Basilisk's core. The bridge connects Basilisk's high-fidelity spacecraft dynamics and simulation timing with ROS 2's distributed middleware.

Spacecraft states generated in Basilisk are published as ROS 2 topics, while commands such as forces, torques, or thruster states can be sent from ROS 2 back into the simulation. This allows standard ROS 2 tools and workflows (e.g., rosbag, RViz, PlotJuggler) to be applied directly to spacecraft simulations, and supports modular autonomy, estimation, control, and monitoring software running externally in ROS 2.

The bridge supports single- and multi-spacecraft scenarios using namespace-aware topic conventions, enabling scalable setups such as formation flying and coordinated control.

## Citation

If you use this package in academic work, please cite:

E. Krantz, N. N. Chan, G. Tibert, H. Mao, C. Fuglesang,
*Bridging the Basilisk Astrodynamics Framework with ROS 2 for Modular Spacecraft Simulation and Hardware Integration*,
arXiv preprint arXiv:2512.09833, 2025.
https://arxiv.org/abs/2512.09833

<details>
<summary>BibTeX</summary>

```bibtex
@article{krantz2025bridging,
  title={Bridging the Basilisk Astrodynamics Framework with ROS 2 for Modular Spacecraft Simulation and Hardware Integration},
  author={Krantz, Elias and Chan, Ngai Nam and Tibert, Gunnar and Mao, Huina and Fuglesang, Christer},
  journal={arXiv preprint arXiv:2512.09833},
  year={2025}
}
```
</details>

## Tested Configurations

| Environment | ROS 2 Distro | Notes |
|---|---|---|
| Ubuntu 22.04 LTS | Humble | Full build, CLI tools, and launch files tested |
| Ubuntu 24.04 LTS | Jazzy | Full build, CLI tools, and launch files tested |
| Windows (WSL, Ubuntu 24.04) | Rolling | Build and topic communication verified |

> **WSL note:** Locale settings may need correction ([see ROS 2 docs](https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debs.html#set-locale)). Minor GUI latency (e.g., BSK-Vizard) is expected; CLI tools work normally.

## Quick Start

### Prerequisites

- [Basilisk](https://avslab.github.io/basilisk/Install.html)
- ROS 2 (e.g. [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html), [Rolling](https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debs.html))

If you want to run the MuJoCo example scenario, Basilisk must be built with MuJoCo support:
```bash
python3 conanfile.py --mujoco True
```

After installation of **Basilisk**, it is recommended to export the user path of Basilisk to `.bashrc` as:
```bash
export BSK_PATH=your_BSK_path
```

In order to visualize the results of the simulations make sure you install [Vizard](https://hanspeterschaub.info/basilisk/Vizard/VizardDownload.html).

### Install

```bash
cd your_ros2_workspace/src
git clone https://github.com/DISCOWER/bsk-msgs.git
git clone https://github.com/DISCOWER/bsk-ros2-bridge.git
git clone https://github.com/DISCOWER/bsk-ros2-mpc.git
cd ..
colcon build bsk-ros2-bridge bsk-msgs bsk-ros2-mpc
source install/setup.bash

pip install -r src/bsk-ros2-bridge/requirements.txt
```

### Run an Example

```bash
# Terminal 1: start the bridge
ros2 launch bsk-ros2-bridge bridge.launch.py

# Terminal 2: start a Basilisk scenario (requires the BSK environment)
source $BSK_PATH/.venv/bin/activate
python examples/scenarioRosOrbit_wrench.py

# Terminal 3: start a controller (e.g., BSK-ROS2-MPC)
ros2 launch bsk-ros2-mpc mpc.launch.py namespace:=bskSat0 type:=wrench use_hill:=True use_rviz:=True
```
Note: This example uses an orbit scenario, hence the MPC launch argument `use_hill:=True` (Hill frame). For the basic no-orbit scenarios, set `use_hill:=False`.

For closed-loop control, see the [BSK-ROS 2 MPC Controller](https://github.com/DISCOWER/bsk-ros2-mpc).

### Example Scenarios

| Scenario | Description | Control Mode |
|---|---|---|
| `scenarioRosBasic_da.py` | Single spacecraft (no Earth) | `da` |
| `scenarioRosBasic_wrench.py` | Single spacecraft (no Earth) | `wrench` |
| `scenarioRosOrbit_da.py` | Spacecraft orbiting Earth | `da` |
| `scenarioRosOrbit_wrench.py` | Spacecraft orbiting Earth | `wrench` |
| `scenarioRosLeaderFollowerBasic_wrench.py` | 1 leader + 2 followers (no Earth) | `wrench` |
| `scenarioRosLeaderFollowerOrbit_wrench.py` | 1 leader + 2 followers (orbit) | `wrench` |
| `scenarioRosMujoco_wrench.py` | Two satellites on a collision course with MuJoCo multibody dynamics | `wrench` |

**Control modes:** `da` (direct allocation) commands individual thrusters via `/<ns>/bsk/in/thr_array_cmd_force`. `wrench` commands 3D forces and torques via `/<ns>/bsk/in/cmd_force` and `/<ns>/bsk/in/cmd_torque`, mapped to thrusters by Basilisk.

The orbit scenarios support any number of spacecraft. Launch one controller per namespace (e.g., `/bskSat0`, `/bskSat1`).

#### MuJoCo Scenario

The MuJoCo example (`scenarioRosMujoco_wrench.py`) is a scenario that simulates two satellites on a collision course and uses MuJoCo for multi-joint and contact dynamics.

To run this scenario, ensure Basilisk was built with MuJoCo support:
```bash
python3 conanfile.py --mujoco True
```

Just like the other examples, the spacecraft in this MuJoCo scenario are actuated, and can be controlled with, e.g., [bsk-ros2-mpc](https://github.com/DISCOWER/bsk-ros2-mpc).

## Development Guide

### Integrating `RosBridgeHandler` in a Scenario

```python
from bsk_module.rosBridgeHandler import RosBridgeHandler

ros_bridge = RosBridgeHandler()
ros_bridge.ModelTag = "ros_bridge"

# Add publishers (Basilisk → ROS 2)
ros_bridge.add_ros_publisher('SCStatesMsgPayload', 'SCStatesMsgIn', 'sc_states', 'bskSat', max_rate=100.0)

# Add subscribers (ROS 2 → Basilisk)  
ros_bridge.add_ros_subscriber('THRArrayCmdForceMsgPayload', 'THRArrayCmdForceMsgOut', 'thr_array_cmd_force', 'bskSat')

# Connect to Basilisk messages
ros_bridge.bskSat.SCStatesMsgIn.subscribeTo(scObject.scStateOutMsg)
thrFiringSchmittObj.thrForceInMsg.subscribeTo(ros_bridge.bskSat.THRArrayCmdForceMsgOut)

# Add the module to a simulation task
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

### Time Synchronization

In non-realtime mode (`accelFactor is not 1`), the bridge publishes simulation time to `/clock`. For time synchronization, ROS 2 nodes must use `use_sim_time:=true` in this case. In realtime mode, `/clock` is not published and nodes use system time.

The clock update interval can be configured via the `clock_timestep`:
```bash
ros2 launch bsk-ros2-bridge bridge.launch.py clock_timestep:=0.1
```

### QoS Settings

All topics use the following ROS 2 Quality of Service (QoS):

- Reliability: **BEST_EFFORT**
- Durability: **VOLATILE**
- History: **KEEP_LAST**
- Queue depth: **1**

This configuration prioritizes low-latency communication and ensures that only the most recent message is delivered, suitable for high-rate spacecraft simulation data.

### Port Configuration

**Bridge launch arguments:**
| Argument | Default | Description |
|---|---|---|
| `sub_port` | 5550 | ZMQ port: Basilisk → Bridge |
| `pub_port` | 5551 | ZMQ port: Bridge → Basilisk |
| `heartbeat_port` | 5552 | ZMQ heartbeat port |
| `clock_timestep` | 0.01 | Clock update interval (s) |
| `namespace` | `''` | Bridge namespace |

To set custom ports:

```bash
ros2 launch bsk-ros2-bridge bridge.launch.py sub_port:=6550 pub_port:=6551 heartbeat_port:=6552
```

```python
ros_bridge = RosBridgeHandler(send_port=6550, receive_port=6551, heartbeat_port=6552)
```

**RosBridgeHandler arguments:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `ModelTag` | `'ros_bridge'` | Module identifier for logging |
| `send_port` | 5550 | ZMQ port for sending data to bridge (BSK → ROS 2) |
| `receive_port` | 5551 | ZMQ port for receiving data from bridge (ROS 2 → BSK) |
| `heartbeat_port` | 5552 | ZMQ port for heartbeat monitoring |
| `accelFactor` | NaN | Simulation speed factor; set to enable `/clock` publishing |

## Troubleshooting

**Missing message types**: ensure `bsk_msgs` is built and sourced.

**Port conflicts**: check with `lsof -i :5550` and kill occupied ports if needed.

**Connection issues**: ensure bridge is running, verify BSK environment is activated.

## References

- [Basilisk Astrodynamics Simulation](https://hanspeterschaub.info/basilisk/)
- [ROS 2 Documentation](https://www.ros.org/)
- [ZeroMQ Documentation](https://zeromq.org/)
- [Basilisk-ROS 2 Messages](https://github.com/DISCOWER/bsk-msgs)
- [Basilisk-ROS 2 MPC](https://github.com/DISCOWER/bsk-ros2-mpc)