# SRL ROS2-to-Basilisk Bridge
Based on [ZMQ Protocol](https://zguide.zeromq.org/docs/chapter2/#Handling-Errors-and-ETERM) used also by Vizard.

## To run a Proof-Of-Concept scenario:
Run on 4 command terminals:

0. Go to the development folder by `cd /dev`;
1. Send fake BSK message in JSON format and receive fake ROS2 command message `/ros_to_basilisk` via bridge:
```
python3 try_fake_send_receive.py
```
2. Start ZMQ BSK-to-ROS2 bridge and publish converted message as topic `/basilisk_data`:
```
python3 zmq_to_ros_bridge.py
```
3. Echo ROS2 topic `/basilisk_data` converted by the bridge from BSK side `try_fake_send_receive.py`:
```
ros2 topic echo /basilisk_data
```
4. Publish fake ROS2 command message `/ros_to_basilisk`:
```
ros2 topic pub /ros_to_basilisk geometry_msgs/msg/Point '{x: 1.0, y: 2.0, z: 4.0}'
```

## To run test in Basilisk:


### Pre-requisites: Source from Basilisk (recommended):
Locate your Basilisk repo and source from the Basilisk Python virtual environment:
```
cd $BSK_REPO # replace BSK_REPO with the actual path to Basilisk/fork.
source .venv/bin/activate
```

Optionally, you can copy [`bsk-module/ROS2Handler.py`](bsk-module/ROS2Handler.py) and [`bsk-module/test_ROS2Handler.py`](bsk-module/test_ROS2Handler.py) into the Basilisk repo/forks, e.g. inside the `/dev` folder.

### Run test via Basilisk modules (within this repo):

1. In this repo, `cd /dev` and start ZMQ BSK-to-ROS2 bridge and published converted message as topic `/basilisk_data`:
```
python3 zmq_to_ros_bridge.py
```
2. Run `test_ROS2Handler.py` to start a unit test case for the `ROS2Handler.py` Basilisk module by:
```
python3 test_ROS2Handler.py
```
3. TODO - Add ROS2 subscriber run to catch BSK-to-ZMQ-to-ROS2 converted message.

Note that `ROS2Handler.py` and `test_ROS2Handler.py` are **not synchronized with Basilisk** and might not be fully up-to-date, check the developed module at the Basilisk repo side regularly.