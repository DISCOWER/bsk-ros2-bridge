import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import zmq
import json
import time

class ZMQToROSBridge(Node):
    def __init__(self):
        super().__init__("zmq_to_ros_bridge")
        self.publisher = self.create_publisher(Point, "basilisk_data", 10)

        # ZMQ setup done outside the class constructor
        # This eliminates the error where you try to set 'context' as an attribute
        self.socket = None

    def setup_zmq(self):
        # ZMQ setup (can be called after initializing the ROS2 node)
        context = zmq.Context()
        self.socket = context.socket(zmq.SUB)
        self.socket.connect("tcp://localhost:5555")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")

    def receive_and_publish(self):
        while rclpy.ok():
            try:
                msg = self.socket.recv_string(flags=zmq.NOBLOCK)
                msg_data = json.loads(msg)

                ros_msg = Point(
                    x=msg_data["position"][0], 
                    y=msg_data["position"][1], 
                    z=msg_data["position"][2]
                )
                self.publisher.publish(ros_msg)
                self.get_logger().info(f"Published to ROS2: {ros_msg}")

            except zmq.Again:
                time.sleep(0.1)  # Avoid busy-waiting

def main():
    rclpy.init()
    node = ZMQToROSBridge()
    node.setup_zmq()  # Call the ZMQ setup after initializing the node
    try:
        node.receive_and_publish()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()