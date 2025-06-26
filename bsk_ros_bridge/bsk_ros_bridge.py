import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, WrenchStamped, TwistStamped, Vector3Stamped, InertiaStamped
import zmq, yaml,json, threading, signal, sys, os, time, importlib
from ament_index_python.packages import get_package_share_directory

class BskRosBridge(Node):
    def __init__(self, config_path=None, sub_port=5555, pub_port=7070, heartbeat_port=9997, timeout=15):
        super().__init__('bsk_ros_bridge')

        self.stop_event = threading.Event()

        if config_path is None:
            config_path = self.declare_parameter(
                'config_path',
                os.path.join(
                    get_package_share_directory('bsk_ros_bridge'), 'config', 'topics.yaml'
                )
            ).get_parameter_value().string_value

        self.config = self.load_config(config_path)
        
        self.sub_port = sub_port
        self.pub_port = pub_port
        self.heartbeat_port = heartbeat_port
        self.timeout = timeout

        # Store active namespaces and their topics
        self.active_namespaces = {}
        self.bridge_publishers = {}
        self.bridge_subscribers = {}

        self.zmq_context = zmq.Context()

        # Setup ZMQ
        self.setup_zmq()

        # Start ZMQ Listener
        self.listener_thread = threading.Thread(target=self.zmq_listener, daemon=True)
        self.listener_thread.start()

        # Signal handling for clean shutdown
        signal.signal(signal.SIGINT, self.clean_exit)
        signal.signal(signal.SIGTERM, self.clean_exit)

        self.get_logger().info("Bridge initialised successfully.")

    def load_config(self, yaml_path):
        with open(yaml_path, 'r') as f:
            return yaml.safe_load(f)
        
    def setup_zmq(self):
        # ZMQ Subscriber (Basilisk → ROS2)
        self.sub_socket = self.zmq_context.socket(zmq.SUB)
        self.sub_socket.connect(f"tcp://localhost:{self.sub_port}")
        self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.sub_socket.setsockopt(zmq.CONFLATE, 1)  # Only keep latest message
        self.sub_socket.setsockopt(zmq.RCVTIMEO, 1000)  # 1 second timeout

        # ZMQ Publisher (ROS2 → Basilisk)
        self.pub_socket = self.zmq_context.socket(zmq.PUB)
        self.pub_socket.bind(f"tcp://*:{self.pub_port}")
        self.pub_socket.setsockopt(zmq.LINGER, 0) 
        self.pub_socket.setsockopt(zmq.SNDHWM, 100)

        # Bridge heartbeat
        self.heartbeat_pub = self.zmq_context.socket(zmq.PUB)
        self.heartbeat_pub.bind(f"tcp://*:{self.heartbeat_port}")
        self.heartbeat_pub.setsockopt(zmq.LINGER, 0)

        def heartbeat_thread():
            while not self.stop_event.is_set():
                try:
                    msg = json.dumps({"msg": "ping", "time": time.time()})
                    self.heartbeat_pub.send_string(msg, flags=zmq.NOBLOCK)
                except zmq.error.Again:
                    pass  # Skip if send would block
                time.sleep(0.1)  # 10 Hz

        threading.Thread(target=heartbeat_thread, daemon=True).start()

        self.get_logger().info("Bridge ZMQ sockets ready.")

    def setup_ros_topics(self, namespace=None):
        """Setup ROS topics for a specific namespace or initialize empty if no namespace provided"""
        if namespace is None:
            # Initialize empty - topics will be created dynamically
            self.get_logger().info("Bridge ready to handle dynamic namespaces")
            return
            
        if namespace in self.active_namespaces:
            self.get_logger().warn(f"Namespace {namespace} already active")
            return
            
        self.active_namespaces[namespace] = True
        self.bridge_publishers[namespace] = {}
        self.bridge_subscribers[namespace] = {}
        
        # Create publishers for this namespace (Basilisk -> ROS2)
        for topic_config in self.config.get('publications', []):
            topic_name = f"/{namespace}{topic_config['topic']}"
            msg_type = self.get_msg_class(topic_config['type'])
            
            publisher = self.create_publisher(msg_type, topic_name, 10)
            self.bridge_publishers[namespace][topic_config['topic']] = {
                'publisher': publisher,
                'type': msg_type,
                'config': topic_config
            }
            
        # Create subscribers for this namespace (ROS2 -> Basilisk)
        for topic_config in self.config.get('subscriptions', []):
            topic_name = f"/{namespace}{topic_config['topic']}"
            msg_type = self.get_msg_class(topic_config['type'])
            
            subscription = self.create_subscription(
                msg_type, topic_name,
                lambda msg, ns=namespace, topic=topic_config['topic']: self.ros_to_basilisk_callback(msg, ns, topic),
                10
            )
            self.bridge_subscribers[namespace][topic_config['topic']] = {
                'subscription': subscription,
                'type': msg_type,
                'config': topic_config
            }
            
        self.get_logger().info(f"Setup topics for namespace: {namespace}")
    
    def get_msg_class(self, type_str):
        """Import and return ROS message class from type string like 'geometry_msgs::msg::PoseStamped'"""
        # Handle both :: and / separators
        if '::' in type_str:
            parts = type_str.split('::')
            pkg_name = parts[0]
            msg_name = parts[2]  # Skip 'msg'
        else:
            parts = type_str.split('/')
            pkg_name = parts[0]
            msg_name = parts[1]
            
        try:
            module = importlib.import_module(f"{pkg_name}.msg")
            return getattr(module, msg_name)
        except (ImportError, AttributeError) as e:
            self.get_logger().error(f"Failed to import message type {type_str}: {e}")
            raise

    def zmq_listener(self):
        while not self.stop_event.is_set() and rclpy.ok():
            try:
                msg_str = self.sub_socket.recv_string(flags=zmq.NOBLOCK)
                data = json.loads(msg_str)
                
                # Extract namespace and topic from the message
                namespace = data.get('namespace', 'default')
                topic = data.get('topic', '/state')
                
                # Ensure namespace is active
                if namespace not in self.active_namespaces:
                    self.setup_ros_topics(namespace)
                
                # Convert and publish the message
                self.publish_to_ros(data, namespace, topic)
                
            except zmq.error.Again:
                # No message available, brief sleep to prevent CPU spinning
                time.sleep(0.001)  # 1ms sleep
            except json.JSONDecodeError as e:
                self.get_logger().error(f"JSON decode error: {e}")
            except Exception as e:
                self.get_logger().error(f"Unexpected error in ZMQ listener: {e}")

    def publish_to_ros(self, data, namespace, topic):
        """Publish BSK data to appropriate ROS topic based on namespace and topic"""
        try:
            if namespace not in self.bridge_publishers or topic not in self.bridge_publishers[namespace]:
                self.get_logger().warn(f"No publisher for {namespace}{topic}")
                return
                
            pub_info = self.bridge_publishers[namespace][topic]
            msg_type = pub_info['type']
            
            # Convert data to ROS message based on type
            if msg_type == PoseStamped:
                ros_msg = self.convert_to_pose_stamped(data)
            elif msg_type == TwistStamped:
                ros_msg = self.convert_to_twist_stamped(data)
            elif msg_type == InertiaStamped:
                ros_msg = self.convert_to_inertia_stamped(data)
            else:
                self.get_logger().warn(f"Unsupported message type: {msg_type}")
                return
                
            pub_info['publisher'].publish(ros_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing to ROS: {e}")

    def convert_to_pose_stamped(self, data):
        """Convert BSK data to PoseStamped message"""
        try:
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = data.get('frame_id', 'map')
            
            # Position from BSK position data
            position = data.get("position", [0.0, 0.0, 0.0])
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = position
            
            # Orientation from BSK attitude data (assuming quaternion)
            attitude = data.get("attitude", [1.0, 0.0, 0.0, 0.0])  # w, x, y, z
            msg.pose.orientation.w, msg.pose.orientation.x, \
            msg.pose.orientation.y, msg.pose.orientation.z = attitude
            
            return msg
        except Exception as e:
            self.get_logger().error(f"Error converting to PoseStamped: {e}")
            raise

    def convert_to_twist_stamped(self, data):
        """Convert BSK data to TwistStamped message"""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = data.get('frame_id', 'map')
        
        velocity = data.get("velocity", [0.0, 0.0, 0.0])
        angular_velocity = data.get("angular_velocity", [0.0, 0.0, 0.0])
        
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = velocity
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = angular_velocity
        
        return msg

    def convert_to_inertia_stamped(self, data):
        """Convert BSK data to InertiaStamped message"""
        msg = InertiaStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = data.get('frame_id', 'body')
        
        msg.inertia.m = data.get("mass", 1.0)
        
        # Inertia matrix (assuming BSK provides Ixx, Iyy, Izz, Ixy, Ixz, Iyz)
        inertia = data.get("inertia", [1.0, 1.0, 1.0, 0.0, 0.0, 0.0])
        msg.inertia.ixx, msg.inertia.iyy, msg.inertia.izz = inertia[:3]
        msg.inertia.ixy, msg.inertia.ixz, msg.inertia.iyz = inertia[3:6]
        
        return msg

    def ros_to_basilisk_callback(self, msg, namespace, topic):
        """Handle ROS messages and send them to Basilisk via ZMQ"""
        try:
            data = {
                'namespace': namespace,
                'topic': topic,
                'timestamp': self.get_clock().now().nanoseconds
            }
            
            # Convert different message types to JSON
            if isinstance(msg, WrenchStamped):
                data['force'] = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z]
                data['torque'] = [msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]
            elif isinstance(msg, PoseStamped):
                data['position'] = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
                data['attitude'] = [msg.pose.orientation.w, msg.pose.orientation.x, 
                                  msg.pose.orientation.y, msg.pose.orientation.z]
            elif isinstance(msg, Vector3Stamped):
                data['vector'] = [msg.vector.x, msg.vector.y, msg.vector.z]
            else:
                self.get_logger().warn(f"Unhandled ROS message type: {type(msg)}")
                return

            json_msg = json.dumps(data)
            self.pub_socket.send_string(json_msg)
            
            # Only log at debug level to reduce overhead
            self.get_logger().debug(f"Sent to Basilisk: {json_msg}")
            
        except Exception as e:
            self.get_logger().error(f"Error in ROS to Basilisk callback: {e}")

    def clean_exit(self, *args):
        self.get_logger().info("Shutting down bridge...")
        self.stop_event.set()

        # Wait for listener thread to finish
        if hasattr(self, 'listener_thread') and self.listener_thread.is_alive():
            self.listener_thread.join(timeout=2)

        # Close ZMQ sockets safely
        try:
            if hasattr(self, 'sub_socket'):
                self.sub_socket.close()
            if hasattr(self, 'pub_socket'):
                self.pub_socket.close()
            if hasattr(self, 'heartbeat_pub'):
                self.heartbeat_pub.close()
            if hasattr(self, 'zmq_context'):
                self.zmq_context.term()
        except Exception as e:
            self.get_logger().error(f"Error closing ZMQ sockets: {e}")

        # Clean up ROS2 node
        try:
            self.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            self.get_logger().error(f"Error during ROS2 cleanup: {e}")

        sys.exit()

def main(args=None):
    rclpy.init(args=args)
    bridge = BskRosBridge(
        sub_port=5555,
        pub_port=7070
    )
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        bridge.get_logger().info("Keyboard interrupt received, shutting down.")
    finally:
        bridge.clean_exit()

if __name__ == '__main__':
    main()