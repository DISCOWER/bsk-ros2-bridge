import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import numpy as np

class BasiliskDataProcessor(Node):
    def __init__(self):
        super().__init__("bsk_data_processor")
        self.subscription = self.create_subscription(Point, "basilisk_data", self.callback, 10)
        self.publication = self.create_publisher(Point, "ros_to_basilisk", 10)
        self.get_logger().info("Initiated `BasiliskDataProcessor()`. Start listening and publishing ROS2 Messages...")

    def callback(self, msg):
        self.get_logger().info(f"Received Basilisk Data: {msg}")
        fake_processed_msg = Point(
            x=np.random.randn(),
            y=np.random.randn(),
            z=np.random.randn()
        )
        self.publication.publish(fake_processed_msg)
        self.get_logger().info(f"Published to ROS2: {fake_processed_msg}")

def main():
    rclpy.init()
    node = BasiliskDataProcessor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
