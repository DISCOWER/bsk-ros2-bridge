import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from geometry_msgs.msg import Point, PoseStamped
import numpy as np

from px4_msgs.msg import VehicleThrustSetpoint, VehicleTorqueSetpoint
class BasiliskDataProcessor(Node):
    def __init__(self):
        super().__init__("bsk_data_processor")
        
        self.namespace_prefix = '' # hardcoded for now
        
        # Subscribe BSK data from ZMQ Bridge:
        self.subscription = self.create_subscription(PoseStamped, "basilisk_data", self.callback, 10)
        # self.subscription = self.create_subscription(Point, "basilisk_data", self.callback, 10)
        
        # Publish ROS2 data after "control":
        self.thrust_publication = self.create_publisher(VehicleThrustSetpoint, "ros_to_bsk_F", 10)
        self.torque_publication = self.create_publisher(VehicleTorqueSetpoint, "ros_to_bsk_L", 10)
        # self.publication = self.create_publisher(Point, "ros_to_basilisk", 10)
        self.get_logger().info("Initiated `BasiliskDataProcessor()`. Start listening and publishing ROS2 Messages...")

    def callback(self, msg):
        self.get_logger().info(f"Received Basilisk Data: {msg}")
        # fake_processed_msg = Point(
        #     x=np.random.randn(),
        #     y=np.random.randn(),
        #     z=np.random.randn()
        # )
        
        thrust_outputs_msg = VehicleThrustSetpoint()
        thrust_outputs_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        thrust_outputs_msg.xyz = [np.random.randn(), np.random.randn(), np.random.randn()]        
        
        torque_outputs_msg = VehicleTorqueSetpoint()
        torque_outputs_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        torque_outputs_msg.xyz = [np.random.randn(), np.random.randn(), np.random.randn()]
        
        self.thrust_publication.publish(thrust_outputs_msg)
        self.torque_publication.publish(torque_outputs_msg)
        self.get_logger().info(f"Published Force msg to ROS2: {thrust_outputs_msg}")        
        self.get_logger().info(f"Published Torque msg to ROS2: {torque_outputs_msg}")

def main():
    rclpy.init()
    node = BasiliskDataProcessor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
