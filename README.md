# SRL ROS2-to-Basilisk Bridge
Based on [ZMQ Protocol](https://zguide.zeromq.org/docs/chapter2/#Handling-Errors-and-ETERM) used also by Vizard.

Run on 4 command terminals:

1. Send fake BSK message in JSON format and receive fake ROS2 command message `/ros_to_basilisk` via bridge:
```
python3 try_fake_send_receive.py
```
2. Start ZMQ BSK-to-ROS2 bridge and published converted message as topic `/basilisk_data`:
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
