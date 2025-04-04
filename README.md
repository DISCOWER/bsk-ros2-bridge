# SRL ROS2-to-Basilisk Bridge
Based on [ZMQ Protocol](https://zguide.zeromq.org/docs/chapter2/#Handling-Errors-and-ETERM) used also by Vizard.

## To run test in Basilisk:
### Pre-requisites: Source from Basilisk (recommended):
Locate your Basilisk repo and source from the Basilisk Python virtual environment:
```
cd $BSK_REPO # replace BSK_REPO with the actual path to Basilisk/fork.
source .venv/bin/activate
```

### Shortcut command (for local testing only, ensure you __exported__ the paths for **Basilisk** as `BSK_FORK` and **ROS2Bridge** as `ROS2_Bridge`)
```
cd $BSK_FORK && . .venv/bin/activate && cd $ROS2_Bridge
```

Optionally, you can copy [`bsk-module/ROS2Handler.py`](bsk-module/ROS2Handler.py) and [`bsk-module/test_ROS2Handler.py`](bsk-module/test_ROS2Handler.py) into the Basilisk repo/forks, e.g. inside the `/dev` folder.

### Run test via Basilisk modules (within this repo):

1. In this repo, run ROS2 run to subscribe BSK-to-ZMQ-to-ROS2 converted message (ROS2 topic) `/basilisk_data` while publishing topic `/ros_to_basilisk`:
```
python3 dev/bsk_data_processor.py
```

2. Start ZMQ BSK-to-ROS2 bridge and published converted message as topic `/basilisk_data`, while subscribing topic `/ros_to_basilisk` to send to Basilisk:
```
python3 dev/zmq_to_ros_bridge.py
```
3. Run `test_ROS2Handler.py` to start a unit test case for the `ROS2Handler.py` Basilisk module by:
```
python3 bsk-module/test_ROS2Handler.py
```

Note that `ROS2Handler.py` and `test_ROS2Handler.py` are **not synchronized with Basilisk** and might not be fully up-to-date, check the developed module at the Basilisk repo side regularly.


## Handling blocked/occupied localhost ports:
### Linux:
1. Get the `PID` for the occupied ports (8888/5555/7070 for all possible default-used ports for the ZMQ Bridge):
```bash
lsof -i :8888
```
2. Kill the port-specific process:
```bash
kill -9 ${PID}
```

## (Archive) To run a Proof-Of-Concept scenario:
Run on 4 command terminals:

0. Go to the development folder by `cd /dev`;
1. Send fake BSK message in JSON format and receive fake ROS2 command message `/ros_to_basilisk` via bridge:
```
python3 try_fake_send_receive.py
```
2. Start ZMQ BSK-to-ROS2 bridge and publish converted message as topic `/basilisk_data`:
```
python3 zmq_to_ros_bridge_original.py
```
3. Echo ROS2 topic `/basilisk_data` converted by the bridge from BSK side `try_fake_send_receive.py`:
```
ros2 topic echo /basilisk_data
```
4. Publish fake ROS2 command message `/ros_to_basilisk`:
```
ros2 topic pub /ros_to_basilisk geometry_msgs/msg/Point '{x: 1.0, y: 2.0, z: 4.0}'
```
