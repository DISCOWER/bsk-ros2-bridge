import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, WrenchStamped, TwistStamped, Vector3Stamped, InertiaStamped
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import zmq, yaml,json, threading, os, time, importlib
from ament_index_python.packages import get_package_share_directory

# Constants
DEFAULT_SUB_PORT = 5550              # ZMQ port for receiving data from Basilisk
DEFAULT_PUB_PORT = 5551              # ZMQ port for sending data to Basilisk
DEFAULT_HEARTBEAT_PORT = 5552        # ZMQ port for heartbeat/keep-alive messages
DEFAULT_TIMEOUT = 15                 # General timeout in seconds (unused currently)
HEARTBEAT_INTERVAL_SEC = 0.1         # Heartbeat ping interval (100ms)

class BskRosBridge(Node):
    """
    ROS2 node bridging Basilisk (via ZMQ) and ROS2 topics.
    Handles dynamic namespace/topic setup, ZMQ communication, and robust shutdown.
    """
    def __init__(self, config_path=None, sub_port=DEFAULT_SUB_PORT, pub_port=DEFAULT_PUB_PORT, 
                 heartbeat_port=DEFAULT_HEARTBEAT_PORT, timeout=DEFAULT_TIMEOUT):
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
        self.active_namespaces = {}
        self.bridge_publishers = {}
        self.bridge_subscribers = {}
        self.zmq_context = zmq.Context()
        self._msg_type_cache = {}  # Cache for message types
        
        # Message conversion registry for efficiency
        self._converter_registry = {
            PoseStamped: self.convert_to_pose_stamped,
            TwistStamped: self.convert_to_twist_stamped,
            InertiaStamped: self.convert_to_inertia_stamped
        }

        # Set up ZMQ
        self.setup_zmq()
        self.listener_thread = threading.Thread(target=self.zmq_listener, daemon=True)
        self.listener_thread.start()
        self.get_logger().info("Bridge initialised successfully.")
        self.sim_time_pub = self.create_publisher(Float64, '/bsk_sim_time', 10)

    def load_config(self, yaml_path):
        """Load YAML config for topic mapping."""
        try:
            with open(yaml_path, 'r') as f:
                config = yaml.safe_load(f)
            
            # Validate configuration
            self._validate_config(config)
            return config
        except FileNotFoundError:
            self.get_logger().error(f"Configuration file not found: {yaml_path}")
            raise
        except yaml.YAMLError as e:
            self.get_logger().error(f"Error parsing YAML config: {e}")
            raise
    
    def _validate_config(self, config):
        """Validate the loaded configuration."""
        required_sections = ['publications', 'subscriptions']
        for section in required_sections:
            if section not in config:
                self.get_logger().warn(f"Missing '{section}' section in config")
                config[section] = []
                
        # Validate each topic configuration
        for section in required_sections:
            for i, topic_config in enumerate(config[section]):
                if 'topic' not in topic_config:
                    raise ValueError(f"Missing 'topic' in {section}[{i}]")
                if 'type' not in topic_config:
                    raise ValueError(f"Missing 'type' in {section}[{i}]")

    def setup_zmq(self):
        """Initialize ZMQ sockets for Basilisk <-> ROS2 communication and heartbeat."""
        self.sub_socket = self.zmq_context.socket(zmq.SUB)
        self.sub_socket.connect(f"tcp://localhost:{self.sub_port}")
        self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.sub_socket.setsockopt(zmq.CONFLATE, 1)
        self.sub_socket.setsockopt(zmq.RCVTIMEO, 1000)

        self.pub_socket = self.zmq_context.socket(zmq.PUB)
        self.pub_socket.bind(f"tcp://*:{self.pub_port}")
        self.pub_socket.setsockopt(zmq.LINGER, 0)
        self.pub_socket.setsockopt(zmq.SNDHWM, 100)

        self.heartbeat_pub = self.zmq_context.socket(zmq.PUB)
        self.heartbeat_pub.bind(f"tcp://*:{self.heartbeat_port}")
        self.heartbeat_pub.setsockopt(zmq.LINGER, 0)

        def heartbeat_thread():
            while not self.stop_event.is_set():
                try:
                    msg = json.dumps({"msg": "ping", "time": time.time()})
                    self.heartbeat_pub.send_string(msg, flags=zmq.NOBLOCK)
                except zmq.error.Again:
                    pass
                except Exception:
                    break
                if self.stop_event.wait(HEARTBEAT_INTERVAL_SEC):
                    break
        self.heartbeat_thread = threading.Thread(target=heartbeat_thread, daemon=True)
        self.heartbeat_thread.start()
        self.get_logger().info("Bridge ZMQ sockets ready.")

    def zmq_listener(self):
        """Thread: Listen for ZMQ messages and publish to ROS topics."""
        self.get_logger().debug("ZMQ listener thread started")
        while not self.stop_event.is_set() and rclpy.ok():
            try:
                msg_str = self.sub_socket.recv_string(flags=zmq.NOBLOCK)
                data = json.loads(msg_str)
                
                # Validate message structure
                if not isinstance(data, dict):
                    self.get_logger().warn("Received non-dict message, skipping")
                    continue

                topic = data.get('topic', '/state')
                # Special handling for /bsk_sim_time (no namespace)
                if topic == '/bsk_sim_time':
                    sim_time = data.get('sim_time', None)
                    if sim_time is not None:
                        msg = Float64()
                        msg.data = float(sim_time)
                        self.sim_time_pub.publish(msg)
                    continue

                namespace = data.get('namespace', 'default')
                # Lazy topic creation: create publisher if not already present
                if namespace not in self.bridge_publishers:
                    self.bridge_publishers[namespace] = {}
                if topic not in self.bridge_publishers[namespace]:
                    self._create_publisher_for_topic(namespace, topic)
                self.publish_to_ros(data, namespace, topic)
            except zmq.error.Again:
                if self.stop_event.wait(0.001):
                    break
            except zmq.error.ZMQError as e:
                self.get_logger().error(f"ZMQError in listener: {e}")
                break
            except json.JSONDecodeError as e:
                self.get_logger().error(f"JSON decode error: {e}")
            except Exception as e:
                self.get_logger().error(f"Unexpected error in ZMQ listener: {e}")
                break
        self.get_logger().debug("ZMQ listener thread exiting")

    def _create_publisher_for_topic(self, namespace, topic):
        """Create a ROS2 publisher for the given topic if defined in config."""
        # Find topic config in publications
        topic_config = None
        for pub in self.config.get('publications', []):
            if pub['topic'] == topic:
                topic_config = pub
                break
        if topic_config is None:
            self.get_logger().warn(f"Topic {topic} not defined in publications config, skipping publisher creation.")
            return
        try:
            msg_type = self.get_msg_class(topic_config['type'])
            topic_name = self._format_topic_name(namespace, topic)
            publisher = self.create_publisher(msg_type, topic_name, 10)
            self.bridge_publishers[namespace][topic] = {
                'publisher': publisher,
                'type': msg_type,
                'config': topic_config
            }
            self.get_logger().info(f"Created publisher for {topic_name} ({msg_type})")
        except Exception as e:
            self.get_logger().error(f"Failed to create publisher for {topic}: {e}")

    def publish_to_ros(self, data, namespace, topic):
        """Publish BSK data to the appropriate ROS topic."""
        try:
            if namespace not in self.bridge_publishers or topic not in self.bridge_publishers[namespace]:
                self.get_logger().warn(f"No publisher for {namespace}{topic}")
                return
                
            pub_info = self.bridge_publishers[namespace][topic]
            msg_type = pub_info['type']
            
            # Use converter registry for efficiency
            converter = self._converter_registry.get(msg_type)
            if converter:
                ros_msg = converter(data)
                pub_info['publisher'].publish(ros_msg)
            else:
                self.get_logger().warn(f"Unsupported message type: {msg_type}")
                
        except Exception as e:
            self.get_logger().error(f"Error publishing to ROS: {e}")

    def _get_ros_time_from_bsk_sim_time(self, data):
        """Helper: Convert bsk_sim_time (float, seconds) to ROS2 builtin_interfaces/Time."""
        from builtin_interfaces.msg import Time as RosTime
        bsk_sim_time = data.get("time", None)
        if bsk_sim_time is not None:
            secs = int(bsk_sim_time)
            nsecs = int((bsk_sim_time - secs) * 1e9)
            t = RosTime()
            t.sec = secs
            t.nanosec = nsecs
            return t
        else:
            # fallback to current clock
            return self.get_clock().now().to_msg()

    def convert_to_pose_stamped(self, data):
        """Convert BSK data dict to PoseStamped message."""
        try:
            msg = PoseStamped()
            msg.header.stamp = self._get_ros_time_from_bsk_sim_time(data)
            msg.header.frame_id = data.get('frame_id', 'map')
            position = data.get("position", [0.0, 0.0, 0.0])
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = position
            attitude = data.get("attitude", [1.0, 0.0, 0.0, 0.0])
            msg.pose.orientation.w, msg.pose.orientation.x, \
            msg.pose.orientation.y, msg.pose.orientation.z = attitude
            return msg
        except Exception as e:
            self.get_logger().error(f"Error converting to PoseStamped: {e}")
            raise

    def convert_to_twist_stamped(self, data):
        """Convert BSK data to TwistStamped message"""
        msg = TwistStamped()
        msg.header.stamp = self._get_ros_time_from_bsk_sim_time(data)
        msg.header.frame_id = data.get('frame_id', 'map')
        velocity = data.get("velocity", [0.0, 0.0, 0.0])
        angular_velocity = data.get("angular_velocity", [0.0, 0.0, 0.0])
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = velocity
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = angular_velocity
        return msg

    def convert_to_inertia_stamped(self, data):
        """Convert BSK data to InertiaStamped message"""
        msg = InertiaStamped()
        msg.header.stamp = self._get_ros_time_from_bsk_sim_time(data)
        msg.header.frame_id = data.get('frame_id', 'body')
        msg.inertia.m = data.get("mass", 1.0)
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
                data['attitude'] = [msg.pose.orientation.w, msg.pose.orientation.x, \
                                  msg.pose.orientation.y, msg.pose.orientation.z]
            elif isinstance(msg, Vector3Stamped):
                data['vector'] = [msg.vector.x, msg.vector.y, msg.vector.z]
            else:
                self.get_logger().warn(f"Unhandled ROS message type: {type(msg)}")
                return
            json_msg = json.dumps(data)
            self.pub_socket.send_string(json_msg)
            self.get_logger().debug(f"Sent to Basilisk: {json_msg}")
        except Exception as e:
            self.get_logger().error(f"Error in ROS to Basilisk callback: {e}")

    def shutdown(self):
        """Cleanly shutdown bridge, sockets, and node."""
        self.get_logger().info("Shutdown requested. Setting stop_event.")
        self.stop_event.set()
        
        # Join threads with timeout for robust shutdown
        try:
            self.get_logger().info("Waiting for threads to complete...")
            if hasattr(self, 'listener_thread') and self.listener_thread.is_alive():
                self.listener_thread.join(timeout=2.0)
                if self.listener_thread.is_alive():
                    self.get_logger().warn("Listener thread did not finish in time")
                    
            if hasattr(self, 'heartbeat_thread') and self.heartbeat_thread.is_alive():
                self.heartbeat_thread.join(timeout=1.0)
                if self.heartbeat_thread.is_alive():
                    self.get_logger().warn("Heartbeat thread did not finish in time")
        except Exception as e:
            self.get_logger().error(f"Error joining threads: {e}")
            
        # Close ZMQ sockets with proper error handling
        try:
            self.get_logger().info("Closing ZMQ sockets...")
            sockets_to_close = []
            if hasattr(self, 'sub_socket'):
                sockets_to_close.append(('sub_socket', self.sub_socket))
            if hasattr(self, 'pub_socket'):
                sockets_to_close.append(('pub_socket', self.pub_socket))
            if hasattr(self, 'heartbeat_pub'):
                sockets_to_close.append(('heartbeat_pub', self.heartbeat_pub))
                
            for name, socket in sockets_to_close:
                try:
                    socket.setsockopt(zmq.LINGER, 0)
                    socket.close()
                except Exception as e:
                    self.get_logger().error(f"Error closing {name}: {e}")
                    
            if hasattr(self, 'zmq_context'):
                self.zmq_context.term()
        except Exception as e:
            self.get_logger().error(f"Error closing ZMQ sockets: {e}")
            
        # Destroy ROS2 node
        try:
            self.get_logger().info("Destroying ROS2 node...")
            super().destroy_node()
        except Exception as e:
            self.get_logger().error(f"Error destroying node: {e}")
            
        self.get_logger().info("Bridge shutdown complete")

    def __enter__(self):
        """Context manager entry."""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit with cleanup."""
        self.shutdown()
        return False  # Don't suppress exceptions

    def get_msg_class(self, type_str):
        """
        Import and return ROS message class from type string like 'geometry_msgs::msg::PoseStamped'
        """
        if type_str in self._msg_type_cache:
            return self._msg_type_cache[type_str]
        parts = type_str.split('::')
        if len(parts) != 3:
            raise ValueError(f"Invalid type string format: {type_str}")
        pkg_name = parts[0]
        msg_name = parts[2]
        try:
            module = importlib.import_module(f"{pkg_name}.msg")
            msg_class = getattr(module, msg_name)
            self._msg_type_cache[type_str] = msg_class
            return msg_class
        except (ImportError, AttributeError) as e:
            self.get_logger().error(f"Failed to import message type {type_str}: {e}")
            raise

    def _format_topic_name(self, namespace, topic):
        """
        Format topic name with namespace, ensuring proper leading slash.
        E.g. namespace='test_sat1', topic='/pose_inertial' -> '/test_sat1/pose_inertial'
        """
        if not topic.startswith('/'):
            topic = '/' + topic
        return f"/{namespace}{topic}"

def main(args=None):
    """Main entry point with improved error handling."""
    rclpy.init(args=args)
    bridge = None
    
    try:
        bridge = BskRosBridge()
        
        # Use context manager for cleaner resource management
        with bridge:
            rclpy.spin(bridge)
            
    except KeyboardInterrupt:
        if bridge:
            bridge.get_logger().info("Keyboard interrupt received")
    except Exception as e:
        error_msg = f"Unexpected error in main: {e}"
        if bridge:
            bridge.get_logger().error(error_msg)
        else:
            print(error_msg)
    finally:
        # Clean shutdown
        if bridge:
            try:
                bridge.shutdown()
            except Exception as e:
                print(f"Error during bridge cleanup: {e}")
        
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            print(f"Error during ROS2 shutdown: {e}")

if __name__ == '__main__':
    main()