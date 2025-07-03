import rclpy
from rclpy.node import Node
from bsk_msgs.msg import *
from std_msgs.msg import Float64
import zmq, json, threading, os, time, importlib, inspect
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
    Automatically handles BSK message types and creates topics dynamically.
    """
    _field_mapping_cache = {}  # Cache for lowercase field name mapping per message type

    def __init__(self, sub_port=DEFAULT_SUB_PORT, pub_port=DEFAULT_PUB_PORT, 
                 heartbeat_port=DEFAULT_HEARTBEAT_PORT, timeout=DEFAULT_TIMEOUT):
        super().__init__('bsk_ros_bridge')
        self.stop_event = threading.Event()

        self.sub_port = sub_port
        self.pub_port = pub_port
        self.heartbeat_port = heartbeat_port
        self.timeout = timeout
        self.bridge_publishers = {}
        self.bridge_subscribers = {}
        self.zmq_context = zmq.Context()
        self._msg_type_cache = {}  # Cache for message types
        self._bsk_msg_types = {}   # Cache for BSK message types by name
        
        # Discover all BSK message types
        self._discover_bsk_message_types()

        # Set up ZMQ
        self.setup_zmq()
        self.listener_thread = threading.Thread(target=self.zmq_listener, daemon=True)
        self.listener_thread.start()
        self.get_logger().info("Bridge initialised successfully.")
        self.sim_time_pub = self.create_publisher(Float64, '/bsk_sim_time', 10)

    def _discover_bsk_message_types(self):
        """Discover all BSK message types from bsk_msgs.msg module."""
        try:
            import bsk_msgs.msg as bsk_msgs
            for name in dir(bsk_msgs):
                # Skip built-in attributes
                if name.startswith('_'):
                    continue
                    
                obj = getattr(bsk_msgs, name)
                # Look for classes ending with 'MsgPayload'
                if inspect.isclass(obj) and name.endswith('MsgPayload'):
                    # Store both the class and its type name
                    self._bsk_msg_types[name] = obj
                    self.get_logger().debug(f"Discovered BSK message type: {name}")
            self.get_logger().info(f"Discovered {len(self._bsk_msg_types)} BSK message types")
        except ImportError as e:
            self.get_logger().error(f"Failed to import bsk_msgs: {e}")
            raise

    def _bsk_type_to_topic_name(self, bsk_type_name):
        """
        Convert BSK message type to snake_case topic name.
        E.g. 'SCStatesMsgPayload' -> 'sc_states'
        E.g. 'THRArrayCmdForceMsgPayload' -> 'thr_array_cmd_force'
        """
        # Remove 'MsgPayload' suffix if present
        if bsk_type_name.endswith('MsgPayload'):
            name = bsk_type_name[:-10]  # Remove 'MsgPayload'
        else:
            name = bsk_type_name
        
        # Convert CamelCase to snake_case
        result = []
        prev_char = ''
        
        for i, char in enumerate(name):
            if char.isupper():
                # Add underscore before uppercase if:
                # 1. Not the first character AND
                # 2. Previous character was lowercase OR next character is lowercase
                if (i > 0 and 
                    (prev_char.islower() or 
                     (i + 1 < len(name) and name[i + 1].islower()))):
                    result.append('_')
                result.append(char.lower())
            else:
                result.append(char.lower())
            prev_char = char
        
        return ''.join(result)

    def _json_to_bsk_msg(self, json_data, msg_type):
        """Convert JSON data to BSK message instance using reflection."""
        try:
            msg = msg_type()
            msg_type_name = msg_type.__name__

            # Use cached mapping if available
            if msg_type_name not in self._field_mapping_cache:
                bsk_field_mapping = {}
                for field_name in msg.get_fields_and_field_types():
                    bsk_field_mapping[field_name.lower()] = field_name
                self._field_mapping_cache[msg_type_name] = bsk_field_mapping
            else:
                bsk_field_mapping = self._field_mapping_cache[msg_type_name]

            # Set fields from JSON data
            for json_field_name, field_value in json_data.items():
                # Try to find the ROS2 field name (either direct match or via lowercase mapping)
                ros_field_name = None
                if hasattr(msg, json_field_name):
                    ros_field_name = json_field_name
                elif json_field_name.lower() in bsk_field_mapping:
                    ros_field_name = bsk_field_mapping[json_field_name.lower()]
                if ros_field_name and hasattr(msg, ros_field_name):
                    try:
                        setattr(msg, ros_field_name, field_value)
                    except Exception:
                        continue
            return msg
        except Exception as e:
            self.get_logger().error(f"Error converting JSON to BSK message: {e}")
            raise

    def _bsk_msg_to_json(self, msg):
        """Convert BSK message to JSON dict using reflection."""
        try:
            json_data = {}
            
            # Get all fields from the message
            for field_name in msg.get_fields_and_field_types():
                if hasattr(msg, field_name):
                    field_value = getattr(msg, field_name)
                    
                    # Convert numpy arrays to lists if necessary
                    if hasattr(field_value, 'tolist'):
                        json_data[field_name] = field_value.tolist()
                    else:
                        json_data[field_name] = field_value
            
            return json_data
        except Exception as e:
            self.get_logger().error(f"Error converting BSK message to JSON: {e}")
            raise

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

                # Special handling for /bsk_sim_time (no namespace)
                if 'sim_time' in data:
                    sim_time = data.get('sim_time', None)
                    if sim_time is not None:
                        msg = Float64()
                        msg.data = float(sim_time)
                        self.sim_time_pub.publish(msg)
                    continue

                # Handle BSK message types
                msg_type_name = data.get('msg_type', None)
                namespace = data.get('namespace', 'default')
                
                if msg_type_name and msg_type_name in self._bsk_msg_types:
                    self._handle_bsk_message(data, namespace, msg_type_name)
                else:
                    self.get_logger().warn(f"Unknown message type: {msg_type_name}")
                    
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

    def _handle_bsk_message(self, data, namespace, msg_type_name):
        """Handle incoming BSK message and publish to appropriate ROS topic."""
        try:
            # Use topic name from message if provided, otherwise convert from message type
            topic_name = data.get('topic_name')
            if topic_name is None:
                topic_name = self._bsk_type_to_topic_name(msg_type_name)
            
            full_topic_name = f"/{namespace}/bsk/out/{topic_name}"
            
            # Create publisher if not exists
            if namespace not in self.bridge_publishers:
                self.bridge_publishers[namespace] = {}
                # Initialize subscribers dict for this namespace
                if namespace not in self.bridge_subscribers:
                    self.bridge_subscribers[namespace] = {}
            
            if topic_name not in self.bridge_publishers[namespace]:
                msg_type = self._bsk_msg_types[msg_type_name]
                publisher = self.create_publisher(msg_type, full_topic_name, 10)
                self.bridge_publishers[namespace][topic_name] = {
                    'publisher': publisher,
                    'type': msg_type
                }
                self.get_logger().info(f"Created publisher for {full_topic_name} ({msg_type_name})")
            
            # Convert JSON to BSK message and publish
            pub_info = self.bridge_publishers[namespace][topic_name]
            bsk_msg = self._json_to_bsk_msg(data, pub_info['type'])
            pub_info['publisher'].publish(bsk_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error handling BSK message {msg_type_name}: {e}")

    def create_bsk_subscribers(self, namespace, msg_type_name):
        """Create a single BSK subscriber if it doesn't exist yet."""
        if namespace not in self.bridge_subscribers:
            self.bridge_subscribers[namespace] = {}
            
        topic_name = self._bsk_type_to_topic_name(msg_type_name)
        
        # Only create if it doesn't exist
        if topic_name not in self.bridge_subscribers[namespace]:
            if msg_type_name in self._bsk_msg_types:
                msg_type = self._bsk_msg_types[msg_type_name]
                full_topic_name = f"/{namespace}/bsk/in/{topic_name}"
                
                # Create callback that captures the message type name
                def create_callback(msg_type_name, namespace):
                    def callback(msg):
                        self.ros_to_basilisk_callback(msg, namespace, msg_type_name)
                    return callback
                
                try:
                    subscriber = self.create_subscription(
                        msg_type,
                        full_topic_name,
                        create_callback(msg_type_name, namespace),
                        10
                    )
                    
                    self.bridge_subscribers[namespace][topic_name] = subscriber
                    self.get_logger().info(f"Created subscriber for {full_topic_name} ({msg_type_name})")
                    return subscriber
                except Exception as e:
                    self.get_logger().debug(f"Could not create subscriber for {full_topic_name}: {e}")
                    return None
        
        return self.bridge_subscribers[namespace].get(topic_name)

    def create_bsk_subscribers(self, namespace):
        """Legacy method - now just initializes the namespace dict."""
        if namespace not in self.bridge_subscribers:
            self.bridge_subscribers[namespace] = {}
            self.get_logger().debug(f"Initialized subscriber dict for namespace: {namespace}")

    def get_or_create_subscriber(self, namespace, msg_type_name):
        """Get existing subscriber or create it if needed."""
        topic_name = self._bsk_type_to_topic_name(msg_type_name)
        
        # Check if subscriber already exists
        if (namespace in self.bridge_subscribers and 
            topic_name in self.bridge_subscribers[namespace]):
            return self.bridge_subscribers[namespace][topic_name]
        
        # Create it if it doesn't exist
        return self.create_bsk_subscribers(namespace, msg_type_name)

    def ros_to_basilisk_callback(self, msg, namespace, msg_type_name):
        """Handle ROS messages and send them to Basilisk via ZMQ"""
        try:
            # Convert BSK message to JSON
            json_data = self._bsk_msg_to_json(msg)
            
            # Add metadata
            data = {
                'namespace': namespace,
                'msg_type': msg_type_name,
                'timestamp': self.get_clock().now().nanoseconds,
                **json_data
            }
            
            json_msg = json.dumps(data)
            self.pub_socket.send_string(json_msg)
            self.get_logger().debug(f"Sent to Basilisk: {msg_type_name} for {namespace}")
            
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