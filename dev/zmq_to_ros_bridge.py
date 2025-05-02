import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Twist, Vector3
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
import zmq
import json, time, threading, signal, sys

class ZMQToROSBridge(Node):
    def __init__(self, request_port = 8888, kill_reply_port = 9999, subscriber_port = 5555, publisher_port = 7070, timeout = 15): # bridege timeout [sec]
        super().__init__("zmq_to_ros_bridge")

        self.timeout = timeout
        # ROS2 Publisher (Basilisk → ROS2)
        self.ROS2Publisher = self.create_publisher(PoseStamped, "basilisk_data", 10) # TODO - Custom message `FullState` combining PoseStamped & Twist!
        # self.ROS2Publisher = self.create_publisher(Point, "basilisk_data", 10)

        # ROS2 Subscriber (ROS2 → Basilisk)
        self.ROS2Subscription = self.create_subscription(
            Point,
            "ros_to_basilisk",
            self.ros_to_zmq,
            10
        )

        self.request_port = request_port
        self.kill_reply_port = kill_reply_port
        self.subscriber_port = subscriber_port
        self.publisher_port = publisher_port
        
        # Initialise as None before running `setup_zmq()`:
        self.req_socket = None
        self.kill_reply_socket = None
        self.sub_socket = None
        self.pub_socket = None
        # self.running = None
        self.thread = None
        
        # ZMQ Setup
        self.setup_zmq()
        self.get_logger().info(f"ZMQ To ROS2 Bridge started.")

    def setup_zmq(self):
        # ZMQ setup (can be called after initializing the ROS2 node)
        global context
        context = zmq.Context()
        
        # ZMQ Requester -> Request reply by sending "Start" to Basilisk side and expect returning "Ready" begin.
        self.req_socket = context.socket(zmq.REQ)
        self.req_socket.connect(f"tcp://localhost:{self.request_port}")
        self.req_socket.setsockopt(zmq.RCVTIMEO, self.timeout * 1000)  # 15-second timeout, convert sec to ms
        # Send request "Start"
        print("ZMQ Bridge: Sending 'Start' request to Basilisk side, waiting for 'Ready' reply...")
        start_message = "Start"
        expected_reply_message = "Ready"
        self.req_socket.send_string(start_message)
        
        # Requested and received `reply`:
        try:
            reply = self.req_socket.recv_string()
            if reply == expected_reply_message:
                print(f"ZMQ Bridge: Received {reply}. Proceeding with execution...")
            else:
                print(f"ZMQ Bridge: Unexpected reply: {reply}. Stopping.")
                exit(0)
        except zmq.error.Again:
            print(f"ZMQ Bridge: No response received within {self.timeout} seconds. Exiting.")
            exit(0)

        # Reconfigure REP socket for NO timeout:
        # self.req_socket.setsockopt(zmq.RCVTIMEO, -1) # set REP socket to infinite wait time
        
        # Kill REP message lister port thread
        self.kill_reply_socket = context.socket(zmq.REP)
        self.kill_reply_socket.bind(f"tcp://*:{self.kill_reply_port}")
        self.kill_reply_socket.setsockopt(zmq.RCVTIMEO, -1) # Set 15-second timeout for receiving.
        
        self.kill_thread = threading.Thread(target=self.recv_kill_message, daemon=True)
        self.kill_thread.start()
        
        # ZMQ Subscriber (Receiving from Basilisk)
        self.sub_socket = context.socket(zmq.SUB)
        self.sub_socket.connect(f"tcp://localhost:{self.subscriber_port}")  # Adjust as needed
        self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.sub_socket.setsockopt(zmq.CONFLATE, 1)  # Always latest data
        # self.sub_socket.setsockopt(zmq.RCVTIMEO, 5000) # Set 5-second timeout for receiving.
        # self.sub_socket.setsockopt(zmq.RCVTIMEO, 100)  # Timeout to prevent blocking forever
        
        # ZMQ Publisher (Sending back to Basilisk)
        self.pub_socket = context.socket(zmq.PUB)
        self.pub_socket.bind(f"tcp://*:{self.publisher_port}")  # Adjust as needed
        
        # Start ZMQ listener thread
        self.stop_thread_event = threading.Event()
        self.thread = threading.Thread(target=self.zmq_listener, daemon=True)
        self.thread.start()
        # self.running = True
        
    def zmq_listener(self):
        """Receives ZMQ messages from `ROS2Handler()` BSK Module from Basilisk and publishes them as ROS2 messages."""
        while (not self.stop_thread_event.is_set()) and rclpy.ok():
            try:
                msg = self.sub_socket.recv_string(flags=zmq.NOBLOCK)
                msg_data = json.loads(msg)

                # Convert to ROS2 message
                ros_msg = self.__Convert_to_ROS2_msg(msg_data)
                
                # Publish ROS2 message:
                self.ROS2Publisher.publish(ros_msg)
                self.get_logger().info(f"Published to ROS2: {ros_msg}")
            except zmq.error.Again:
                # print(f"ZMQ Bridge: No response received from BSK within {self.timeout} seconds. Exiting.")
                # exit(0)
                pass
            # except zmq.Again:
                # time.sleep(0.1)  # Avoid busy-waiting
                # pass # No new messages, continue loop
                

    def __Convert_to_ROS2_msg(self, json_msg_data):
        # ros_msg = Point(
        #             x=json_msg_data["attitude"][0], 
        #             y=json_msg_data["attitude"][1], 
        #             z=json_msg_data["attitude"][2]
        #         )
        # TODO - Define another ROS2 msg type to replace `Point()`!
        # TODO - Should be a combination of `PoseStamped()`, `Twist()`, with a timestamp.
        ros_msg = PoseStamped(
            header=Header(
                stamp=Time(sec=int(json_msg_data["time"]), nanosec=0),
                frame_id="map"
            ),
            pose=Pose( 
                position=Point(
                    x=json_msg_data["position"][0],
                    y=json_msg_data["position"][1],
                    z=json_msg_data["position"][2]
                ),
                orientation=Quaternion(
                    w=json_msg_data["attitude"][0],
                    x=json_msg_data["attitude"][1],
                    y=json_msg_data["attitude"][2],
                    z=json_msg_data["attitude"][3]
                )
        )
        )
        # TODO - publish to a different ROS2 topic:
        ros_msg_V = Twist(
            linear=Vector3(
            x=json_msg_data["velocity"][0],
            y=json_msg_data["velocity"][1],
            z=json_msg_data["velocity"][2]
            ),
            angular=Vector3(
            x=json_msg_data["omega"][0],
            y=json_msg_data["omega"][1],
            z=json_msg_data["omega"][2]
            )
        )
        
        return ros_msg

    def ros_to_zmq(self, msg):
        """Receives ROS2 messages and sends them back to Basilisk via ZMQ."""
        msg_data = {
            "FrCmd": [msg.x, msg.y, msg.z], # fake force using Point() ROS2 message
            "lrCmd": [msg.x, msg.y, msg.z] # fake force using Point() ROS2 message
        }
        # TODO - define/reuse another ROS2 message type for command force & torque!
        # msg_data = {
        #     "FrCmd": [msg.Fx, msg.Fy, msg.Fz],
        #     "lrCmd": [msg.Lx, msg.Ly, msg.Lz]
        # }
        json_msg = json.dumps(msg_data)
        
        self.pub_socket.send_string(json_msg)
        self.get_logger().info(f"Sent to Basilisk via ZMQ: {json_msg}")

    def clean_exit(self, signum=None, frame=None):
        self.get_logger().info("Shutting down bridge...")
        self.stop_thread_event.set()
        
        # Check which multi-threading thread it is currently in:
        current = threading.current_thread()
        if self.thread is not current:
            self.thread.join()
        if self.kill_thread is not current:
            self.kill_thread.join()
        
        self.req_socket.close()
        self.kill_reply_socket.close()
        self.sub_socket.close()
        self.pub_socket.close()
        
        global context
        if context is not None:
            context.term()
    
        self.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()  # Properly shut down ROS2
        
        sys.exit()
    
    def recv_kill_message(self):
        try:
            kill_msg = self.kill_reply_socket.recv_string()
            if kill_msg:
                if kill_msg == "Kill":
                    # self.stop_thread_event.set()
                    self.get_logger().info("Kill Bridge Message Received...")
                    self.kill_reply_socket.send_string("Killed")
                    
                    self.clean_exit()
                    sys.exit()
        except zmq.error.Again:
            pass
    
def main(default_req_port = 8888, default_kill_rep_port = 9999, default_sub_port = 5555, default_pub_port = 7070):
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("-r", "--req", type=int, 
                        help=f"[int] Localhost port number, default = {default_req_port}")
    parser.add_argument("-k", "--kill", type=int, 
                        help=f"[int] Localhost port number, default = {default_kill_rep_port}")
    parser.add_argument("-s", "--sub", type=int, 
                        help=f"[int] Localhost port number, default = {default_sub_port}")
    
    parser.add_argument("-p", "--pub", type=int, 
                        help=f"[int] Localhost port number, default = {default_pub_port}")
    
    args = parser.parse_args()
    req_port = args.req if args.req is not None else default_req_port
    kill_rep_port = args.kill if args.kill is not None else default_kill_rep_port
    sub_port = args.sub if args.sub is not None else default_sub_port
    pub_port = args.pub if args.pub is not None else default_pub_port

    rclpy.init()
    node = ZMQToROSBridge(req_port, kill_rep_port, sub_port, pub_port)
    
    # Attach custom handler for Ctrl+Z
    def handle_tstp(signum, frame):
        print("SIGTSTP (Ctrl+Z) received. Cleaning up...")
        node.clean_exit()

    signal.signal(signal.SIGTSTP, handle_tstp)
    
    try:
        while rclpy.ok():
            # rclpy.spin_once(node, timeout_sec=0.1)
            rclpy.spin(node)
    except KeyboardInterrupt:
        # sys.exit() 
        print("KeyboardInterrupt received. Cleaning up...")
    finally:
        node.clean_exit()
        sys.exit()

if __name__ == "__main__":
    main(default_req_port = 8888,
         default_kill_rep_port = 9999,
         default_sub_port = 5555, 
         default_pub_port = 7070)