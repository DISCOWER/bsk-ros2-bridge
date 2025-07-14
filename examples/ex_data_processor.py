import rclpy
from rclpy.node import Node
from bsk_msgs.msg import CmdForceBodyMsgPayload, CmdTorqueBodyMsgPayload, SCStatesMsgPayload, THRArrayCmdForceMsgPayload
from std_msgs.msg import Float64
import numpy as np

class BasiliskDataProcessor(Node):
    def __init__(self):
        super().__init__("bsk_data_processor")
        
        # Declare ROS2 parameter for thruster mode (direct allocation or wrench)
        self.declare_parameter('mode', 'direct_allocation')
        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        
        # Get namespace from parameter (can be set via ros2 run --ros-args -p namespace:=/bskSat0)
        self.declare_parameter('namespace', '/bskSat')
        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value
        
        self.get_logger().info(f"Starting BasiliskDataProcessor for namespace: {self.namespace}")
        self.get_logger().info(f"Mode: {self.mode}")
        
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
        if self.mode == 'direct_allocation':
            self.force_pub = self.create_publisher(
                THRArrayCmdForceMsgPayload, 
                f"{self.namespace}/bsk/in/thr_array_cmd_force", 
                10
            )
        elif self.mode == 'wrench':
            self.force_pub = self.create_publisher(
                CmdForceBodyMsgPayload, 
                f"{self.namespace}/bsk/in/cmd_force", 
                10
            )
            self.torque_pub = self.create_publisher(
                CmdTorqueBodyMsgPayload,
                f"{self.namespace}/bsk/in/cmd_torque",
                10
            )
        else:
            self.get_logger().error(f"Unknown mode: {self.mode}. Use 'direct_allocation' or 'wrench'.")
            return

        timer_cmd_period = 0.1  # seconds
        self.create_timer(timer_cmd_period, self.cmd_callback)

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
        if self.mode == 'direct_allocation':
            # Create a THRArrayCmdForceMsgPayload message for 12 thrusters
            force_msg = THRArrayCmdForceMsgPayload()
            # Use simulation time for timestamp synchronization
            force_msg.stamp.sec = int(self.sim_time)
            force_msg.stamp.nanosec = int((self.sim_time - int(self.sim_time)) * 1e9)
            
            # Set thruster force values (0-1.5 N for each of 12 thrusters)
            for i in range(12):
                force_msg.thrforce[i] = np.random.uniform(0.0, 1.5)
            # force_msg.thrforce[0] = 0.75
            # force_msg.thrforce[2] = 0.75
            # force_msg.thrforce[3] = 0*0.25

            # Publish the messages
            self.force_pub.publish(force_msg)

            thruster_forces = [f"{force_msg.thrforce[i]:.2f}" for i in range(12)]
            self.get_logger().info(
                f"Sent thruster commands:"
                f"F=[{', '.join(thruster_forces)}] N"
            )

        elif self.mode == 'wrench':
            # Create a CmdForceBodyMsgPayload message
            force_msg = CmdForceBodyMsgPayload()
            # Use simulation time for timestamp synchronization
            force_msg.stamp.sec = int(self.sim_time)
            force_msg.stamp.nanosec = int((self.sim_time - int(self.sim_time)) * 1e9)
            
            # Set dummy force values (Newtons) - replace with actual control law
            force_msg.forcerequestbody[0] = np.random.uniform(-3.0, 3.0)
            force_msg.forcerequestbody[1] = np.random.uniform(-3.0, 3.0)
            force_msg.forcerequestbody[2] = np.random.uniform(-3.0, 3.0)
        
            # Create a CmdTorqueBodyMsgPayload message
            torque_msg = CmdTorqueBodyMsgPayload()
            # Use simulation time for timestamp synchronization
            torque_msg.stamp.sec = int(self.sim_time)
            torque_msg.stamp.nanosec = int((self.sim_time - int(self.sim_time)) * 1e9)

            # Set dummy torque values (Newton-meters) - replace with actual control law
            torque_msg.torquerequestbody[0] = np.random.uniform(-0.36, 0.36)
            torque_msg.torquerequestbody[1] = np.random.uniform(-0.36, 0.36)
            torque_msg.torquerequestbody[2] = np.random.uniform(-0.36, 0.36)
            
            # Publish the messages
            self.force_pub.publish(force_msg)
            self.torque_pub.publish(torque_msg)
        
            self.get_logger().info(
                f"Sent commands:"
                f"F=[{force_msg.forcerequestbody[0]:.2f}, {force_msg.forcerequestbody[1]:.2f}, {force_msg.forcerequestbody[2]:.2f}] N"
                f"T=[{torque_msg.torquerequestbody[0]:.2f}, {torque_msg.torquerequestbody[1]:.2f}, {torque_msg.torquerequestbody[2]:.2f}] Nm"
            )
        else:
            self.get_logger().error(f"Unknown mode: {self.mode}. Use 'direct_allocation' or 'wrench'.")

def main():
    rclpy.init()
    node = BasiliskDataProcessor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
