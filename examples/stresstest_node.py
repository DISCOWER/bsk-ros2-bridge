import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from bsk_msgs.msg import THRArrayConfigMsgPayload
# from builtin_interfaces.msg import Time
from rclpy.time import Time
from std_msgs.msg import Float64

class BasiliskStresstestNode(Node):
    def __init__(self):
        super().__init__("bsk_stresstest_node",
                        parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        
        # Declare parameters
        self.declare_parameter('num_sc', 1)  # Default to 1 spacecraft
        self.declare_parameter('w', 100)  # Default window size of 100
        self.declare_parameter('period', 1.) # Publish period in milliseconds
        self.declare_parameter('sim_period', 1.) # Simulation time step in microseconds

        # Get parameters
        self.num_spacecraft = self.get_parameter('num_sc').get_parameter_value().integer_value
        self.window_size = self.get_parameter('w').get_parameter_value().integer_value
        self.period = self.get_parameter('period').get_parameter_value().double_value
        self.sim_period = self.get_parameter('sim_period').get_parameter_value().double_value
        
        self.get_logger().info(f"Starting BasiliskStresstestNode with {self.num_spacecraft} spacecraft")

        # Dictionary to store publishers and subscribers for each spacecraft
        self.msg_pubs = {}
        self.msg_subs = {}

        # Initialize message at zero
        self.msg_idx = 0
        self.message = THRArrayConfigMsgPayload()
        self.message.numthrusters = self.msg_idx

        # Create latency statistics publishers for bskSat0 only
        self.latency_mean_pub = self.create_publisher(Float64, '/bsk_bridge/latency_mean_us', 100)
        self.latency_std_pub = self.create_publisher(Float64, '/bsk_bridge/latency_std_us', 100)
        self.dropped_msgs_pub = self.create_publisher(Float64, '/bsk_bridge/dropped_msgs', 100)

        # Store send timestamps for messages (for bskSat0 only)
        self.send_times = {}  # Dictionary to store send time for each message index
        self.highest_received_idx = 0  # Track highest received message index for cleanup
        
        # Store measurements for statistics
        self.latency_window = []  # Will collect up to window_size measurements
        self.dropped_msgs_window = 0  # Count of dropped messages in current window
        self.total_msgs_window = 0  # Total messages that should have been received in window
        self.last_processed_idx = 0  # Last processed message index to track gaps
        
        # Maximum number of pending messages to store timestamps for
        self.max_pending_messages = 1000  # Prevent memory growth from lost messages

        self.create_timer(self.period / 1e3, self.pub_callback)

        self.get_logger().info(f"Period  is set to: {self.period} ms")

        # Create publishers and subscribers for each spacecraft
        for i in range(self.num_spacecraft):
            namespace = f'bskSat{i}'
            
            # Subscribers for each spacecraft
            self.msg_subs[namespace] = self.create_subscription(
                THRArrayConfigMsgPayload,
                '/bskSat0/bsk/out/msg_out',
                self.message_callback,
                100
            )
            self.get_logger().info(f"Subscribed to: /{namespace}/bsk/out/msg_out")

            # Publishers for each spacecraft
            self.msg_pubs[namespace] = self.create_publisher(
                THRArrayConfigMsgPayload,
                f'/{namespace}/bsk/in/msg_in',
                100
            )
            self.get_logger().info(f"Publishing to: /{namespace}/bsk/in/msg_in")

        # Check if use_sim_time is enabled
        if self.get_parameter('use_sim_time').get_parameter_value().bool_value:
            self.get_logger().info("Using simulation time, waiting for /clock...")
            self.wait_for_clock()

    def wait_for_clock(self):
        """Wait for the /clock topic to start publishing."""
        rate = self.create_rate(10)  # 10 Hz
        warned = False
        while rclpy.ok():
            topics = dict(self.get_topic_names_and_types())
            if '/clock' in topics:
                return
            if not warned:
                self.get_logger().info("Waiting for /clock topic...")
                warned = True
            rate.sleep()

    def message_callback(self, msg: THRArrayConfigMsgPayload):
        # Only measure latency for bskSat0
        recv_time = Time.from_msg(msg.stamp)
        # recv_time = self.get_clock().now()
        received_idx = msg.numthrusters
        
        # Update highest received index and clean up old timestamps
        if received_idx > self.highest_received_idx:
            # Count dropped messages and clean up timestamps
            for idx in list(self.send_times.keys()):
                if idx < received_idx:
                    if idx > self.highest_received_idx:
                        # This message was skipped/dropped
                        self.dropped_msgs_window += 1
                    del self.send_times[idx]
            self.highest_received_idx = received_idx
        
        # Count messages in the gap between last processed and current
        if received_idx > self.last_processed_idx:
            gap_size = received_idx - self.last_processed_idx
            self.total_msgs_window += gap_size - 1  # -1 because current message is counted separately
            self.dropped_msgs_window += gap_size - 1  # All messages in gap were dropped
        
        # Add current message to total
        self.total_msgs_window += 1
        
        # Calculate latency if we have the send time for this message
        if received_idx in self.send_times:
            # Calculate latency in microseconds
            latency_us = (recv_time - self.send_times[received_idx]).nanoseconds / 1e3
            # Subtract simulation period (both in microseconds)
            latency_us = latency_us - self.sim_period
            latency_us = max(latency_us, 0.0)
            
            # Add to window
            self.latency_window.append(latency_us)
            
            # Remove the timestamp as we've processed it
            del self.send_times[received_idx]
            
        # Update last processed index
        self.last_processed_idx = received_idx
            
        # If window is full, publish statistics and reset
        if len(self.latency_window) >= self.window_size:
            # Calculate statistics
            mean_us = np.mean(self.latency_window)
            std_us = np.std(self.latency_window)
            
            # Publish mean
            mean_msg = Float64()
            mean_msg.data = mean_us
            self.latency_mean_pub.publish(mean_msg)
            
            # Publish std
            std_msg = Float64()
            std_msg.data = std_us
            self.latency_std_pub.publish(std_msg)
            
            # Publish dropped messages percentage
            dropped_msg = Float64()
            if self.total_msgs_window > 0:
                dropped_percentage = (self.dropped_msgs_window / self.total_msgs_window) * 100.0
            else:
                dropped_percentage = 0.0
            dropped_msg.data = dropped_percentage
            self.dropped_msgs_pub.publish(dropped_msg)
            
            # Reset window
            self.latency_window = []
            self.dropped_msgs_window = 0
            self.total_msgs_window = 0

    def pub_callback(self):
        # Increment message index and update message
        self.msg_idx += 1
        self.message.numthrusters = self.msg_idx

        # Publish messages for all spacecraft
        namespace = 'bskSat0'
        
        # Store send time for bskSat0 only
        if namespace == 'bskSat0':
            if len(self.send_times) < self.max_pending_messages:
                self.send_times[self.msg_idx] = self.get_clock().now()
        
        # Publish the message
        self.msg_pubs[namespace].publish(self.message)

def main():
    rclpy.init()
    node = BasiliskStresstestNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
