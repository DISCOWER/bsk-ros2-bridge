import rclpy
from rclpy.node import Node
from bsk_msgs.msg import CmdForceBodyMsgPayload, CmdTorqueBodyMsgPayload, SCStatesMsgPayload
from std_msgs.msg import Float64
import numpy as np

class BasiliskDataProcessor(Node):
    def __init__(self):
        super().__init__("bsk_data_processor")
        
        # Get namespace from node (can be set via ros2 run --ros-args -r __ns:=/namespace)
        self.namespace = self.get_namespace()
        if self.namespace == '/':
            self.namespace = '/test_sat1'  # Default namespace if none provided
        
        self.get_logger().info(f"Starting BasiliskDataProcessor for namespace: {self.namespace}")
        
        # Track simulation time
        self.sim_time = 0.0
        
        # Subscribe to simulation time
        self.sim_time_sub = self.create_subscription(
            Float64,
            '/bsk_sim_time',
            self.sim_time_callback,
            10
        )
        
        # Subscribers - Listen to Basilisk output messages
        self.state_sub = self.create_subscription(
            SCStatesMsgPayload, 
            f"{self.namespace}/bsk/out/sc_states", 
            self.state_callback, 
            10
        )
        self.get_logger().info(f"Subscribed to: {self.namespace}/bsk/out/sc_states")
        
        # Publishers - Send commands to Basilisk input topics
        self.force_pub = self.create_publisher(
            CmdForceBodyMsgPayload, 
            f"{self.namespace}/bsk/in/cmd_force_body", 
            10
        )
        self.torque_pub = self.create_publisher(
            CmdTorqueBodyMsgPayload,
            f"{self.namespace}/bsk/in/cmd_torque_body",
            10
        )
        timer_cmd_period = 0.1  # seconds
        self.create_timer(timer_cmd_period, self.cmd_callback)
        self.get_logger().info(f"Publishing to: {self.namespace}/bsk/in/cmd_force_body and {self.namespace}/bsk/in/cmd_torque_body")
        
        self.get_logger().info("Waiting for Basilisk data...")

    def sim_time_callback(self, msg: Float64):
        """Update simulation time from Basilisk."""
        self.sim_time = msg.data

    def state_callback(self, msg: SCStatesMsgPayload):
        """Process received spacecraft state - implement your control logic here."""
        # Log basic state info to observe the effects of commands
        pos = msg.r_bn_n
        vel = msg.v_bn_n
        att = msg.sigma_bn
        omega = msg.omega_bn_b

    def cmd_callback(self):        
        # Create a CmdForceBodyMsgPayload message
        force_msg = CmdForceBodyMsgPayload()
        # Use simulation time for timestamp synchronization
        force_msg.stamp.sec = int(self.sim_time)
        force_msg.stamp.nanosec = int((self.sim_time - int(self.sim_time)) * 1e9)
        
        # Set dummy force values (Newtons) - replace with actual control law
        force_msg.force_request_body[0] = 10.0 #np.random.uniform(-10.0, 10.0)
        force_msg.force_request_body[1] = 0.0 #np.random.uniform(-10.0, 10.0)
        force_msg.force_request_body[2] = 0.0 #np.random.uniform(-10.0, 10.0)
        
        # Create a CmdTorqueBodyMsgPayload message
        torque_msg = CmdTorqueBodyMsgPayload()
        # Use simulation time for timestamp synchronization
        torque_msg.stamp.sec = int(self.sim_time)
        torque_msg.stamp.nanosec = int((self.sim_time - int(self.sim_time)) * 1e9)

        # Set dummy torque values (Newton-meters) - replace with actual control law
        torque_msg.torque_request_body[0] = 1. #np.random.uniform(-1.0, 1.0)
        torque_msg.torque_request_body[1] = 0. #np.random.uniform(-1.0, 1.0)
        torque_msg.torque_request_body[2] = 0. #np.random.uniform(-1.0, 1.0)
        
        # Publish the messages
        self.force_pub.publish(force_msg)
        self.torque_pub.publish(torque_msg)
        
        # Log sent commands
        self.get_logger().info(
            f"Sent commands: F=[{force_msg.force_request_body[0]:.2f}, {force_msg.force_request_body[1]:.2f}, {force_msg.force_request_body[2]:.2f}] N, "
            f"T=[{torque_msg.torque_request_body[0]:.2f}, {torque_msg.torque_request_body[1]:.2f}, {torque_msg.torque_request_body[2]:.2f}] Nm"
        )

def main():
    rclpy.init()
    node = BasiliskDataProcessor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
