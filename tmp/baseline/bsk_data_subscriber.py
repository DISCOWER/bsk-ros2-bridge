import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class BasiliskDataSubscriber(Node):
    def __init__(self):
        super().__init__("bsk_data_subscriber")
        self.subscription = self.create_subscription(Point, "basilisk_data", self.callback, 10)

    def callback(self, msg):
        self.get_logger().info(f"Received Basilisk Data: {msg}")

def main():
    rclpy.init()
    node = BasiliskDataSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
