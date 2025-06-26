import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, WrenchStamped, TwistStamped
import numpy as np

class BasiliskDataProcessor(Node):
    def __init__(self):
        super().__init__("bsk_data_processor")
        
        # Get namespace from node (can be set via ros2 run --ros-args -r __ns:=/namespace)
        self.namespace = self.get_namespace()
        if self.namespace == '/':
            self.namespace = '/test_sat1'  # Default namespace if none provided
        
        self.get_logger().info(f"Starting BasiliskDataProcessor for namespace: {self.namespace}")
        
        # Subscribers
        self.state_sub = self.create_subscription(
            PoseStamped, 
            f"{self.namespace}/state", 
            self.state_callback, 
            10
        )
        self.get_logger().info(f"Subscribed to: {self.namespace}/state")

        self.velocity_sub = self.create_subscription(
            TwistStamped, 
            f"{self.namespace}/velocity", 
            self.velocity_callback, 
            10
        )
        self.get_logger().info(f"Subscribed to: {self.namespace}/velocity")
        
        # Publishers
        self.wrench_pub = self.create_publisher(
            WrenchStamped, 
            f"{self.namespace}/control_wrench", 
            10
        )
        timer_cmd_period = 0.1  # seconds
        self.create_timer(timer_cmd_period, self.cmd_callback)
        self.get_logger().info(f"Publishing to: {self.namespace}/control_wrench")
        
        self.get_logger().info("Waiting for Basilisk data...")

    def state_callback(self, msg: PoseStamped):
        self.get_logger().info(
            f"Received Basilisk state data from {self.namespace}: {msg}"
        )

    def velocity_callback(self, msg: TwistStamped):
        self.get_logger().info(
            f"Received Basilisk velocity data from {self.namespace}: {msg}"
        )

    def cmd_callback(self):        
        # Create a WrenchStamped message with dummy force and torque values
        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = self.get_clock().now().to_msg()
        wrench_msg.header.frame_id = f"{self.namespace.strip('/')}_body"
        
        # Set dummy force values (Newtons)
        wrench_msg.wrench.force.x = np.random.uniform(-10.0, 10.0)
        wrench_msg.wrench.force.y = np.random.uniform(-10.0, 10.0)
        wrench_msg.wrench.force.z = np.random.uniform(-10.0, 10.0)
        
        # Set dummy torque values (Newton-meters)
        wrench_msg.wrench.torque.x = np.random.uniform(-1.0, 1.0)
        wrench_msg.wrench.torque.y = np.random.uniform(-1.0, 1.0)
        wrench_msg.wrench.torque.z = np.random.uniform(-1.0, 1.0)
        
        # Publish the combined wrench message
        self.wrench_pub.publish(wrench_msg)
        self.get_logger().info(
            f"Sent control to {self.namespace}: "
            f"F=[{wrench_msg.wrench.force.x:.2f}, {wrench_msg.wrench.force.y:.2f}, {wrench_msg.wrench.force.z:.2f}], "
            f"T=[{wrench_msg.wrench.torque.x:.2f}, {wrench_msg.wrench.torque.y:.2f}, {wrench_msg.wrench.torque.z:.2f}]"
        )

def main():
    rclpy.init()
    node = BasiliskDataProcessor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
