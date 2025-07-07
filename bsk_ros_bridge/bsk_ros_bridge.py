import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import zmq
import threading
import orjson
import inspect
import re

# Constants
DEFAULT_SUB_PORT = 5550              # ZMQ port for receiving data from Basilisk
DEFAULT_PUB_PORT = 5551              # ZMQ port for sending data to Basilisk
DEFAULT_HEARTBEAT_PORT = 5552        # ZMQ port for heartbeat/keep-alive messages
HEARTBEAT_INTERVAL_SEC = 0.1         # Heartbeat ping interval (100ms)

# Pre-compiled regex for camelCase conversion
CAMEL_TO_SNAKE_RE = re.compile(r'(?<!^)(?=[A-Z])')

# Using orjson for fast JSON processing
def json_loads(data): 
    return orjson.loads(data)

def json_dumps(data): 
    return orjson.dumps(data)  # Return bytes directly

class BskRosBridge(Node):
    """
    ROS2 node bridging Basilisk (via ZMQ) and ROS2 topics.
    Automatically handles BSK message types and creates topics dynamically.
    """
    _field_mapping_cache = {}  # Cache for lowercase field name mapping per message type
    _topic_name_cache = {}     # Cache for topic name conversions

    def __init__(self, sub_port=DEFAULT_SUB_PORT, pub_port=DEFAULT_PUB_PORT, heartbeat_port=DEFAULT_HEARTBEAT_PORT):
        super().__init__('bsk_ros_bridge')
        self.stop_event = threading.Event()

        # Declare parameters for port configuration
        self.declare_parameter('sub_port', sub_port)
        self.declare_parameter('pub_port', pub_port)
        self.declare_parameter('heartbeat_port', heartbeat_port)

        # Get port values from parameters (allows launch-time configuration)
        self.sub_port = self.get_parameter('sub_port').get_parameter_value().integer_value
        self.pub_port = self.get_parameter('pub_port').get_parameter_value().integer_value
        self.heartbeat_port = self.get_parameter('heartbeat_port').get_parameter_value().integer_value

        self.bridge_publishers = {}
        self.bridge_subscribers = {}
        self.zmq_context = zmq.Context()
        self._bsk_msg_types = {}   # Cache for BSK message types by name
        self._sim_time_msg = Float64()
        
        # Discover all BSK message types
        self._discover_bsk_message_types()

        # Set up ZMQ
        self.setup_zmq()
        self.listener_thread = threading.Thread(target=self.zmq_listener, daemon=True)
        self.listener_thread.start()
        self.get_logger().info(f"Bridge initialised successfully on ports {self.sub_port}/{self.pub_port}/{self.heartbeat_port}.")
        self.sim_time_pub = self.create_publisher(Float64, '/bsk_sim_time', 10)

    def _discover_bsk_message_types(self):
        """Discover all BSK message types from bsk_msgs.msg module."""
        try:
            import bsk_msgs.msg as bsk_msgs
            # Use list comprehension for better performance
            msg_classes = [
                (name, getattr(bsk_msgs, name)) 
                for name in dir(bsk_msgs) 
                if not name.startswith('_') and 
                   inspect.isclass(getattr(bsk_msgs, name)) and 
                   name.endswith('MsgPayload')
            ]
            
            self._bsk_msg_types = dict(msg_classes)
            self.get_logger().info(f"Discovered {len(self._bsk_msg_types)} BSK message types")
        except ImportError as e:
            self.get_logger().error(f"Failed to import bsk_msgs: {e}")
            raise

    def _bsk_type_to_topic_name(self, bsk_type_name):
        """Convert BSK message type to snake_case topic name with caching."""
        if bsk_type_name in self._topic_name_cache:
            return self._topic_name_cache[bsk_type_name]
            
        # Remove 'MsgPayload' suffix if present
        name = bsk_type_name[:-10] if bsk_type_name.endswith('MsgPayload') else bsk_type_name
        
        # Use regex for more efficient camelCase to snake_case conversion
        topic_name = CAMEL_TO_SNAKE_RE.sub('_', name).lower()
        
        self._topic_name_cache[bsk_type_name] = topic_name
        return topic_name

    def _json_to_bsk_msg(self, json_data, msg_type):
        """Convert JSON data to BSK message instance using cached reflection."""
        msg = msg_type()
        msg_type_name = msg_type.__name__

        # Use cached field mapping
        if msg_type_name not in self._field_mapping_cache:
            # Pre-compute field mapping once per message type
            field_types = msg.get_fields_and_field_types()
            bsk_field_mapping = {name.lower(): name for name in field_types}
            self._field_mapping_cache[msg_type_name] = (bsk_field_mapping, field_types)
        
        bsk_field_mapping, field_types = self._field_mapping_cache[msg_type_name]

        # Set fields from JSON data
        for json_field_name, field_value in json_data.items():
            # Special handling for timestamp
            if json_field_name == "stamp" and hasattr(msg, "stamp") and isinstance(field_value, dict):
                msg.stamp.sec = int(field_value.get("sec", 0))
                msg.stamp.nanosec = int(field_value.get("nanosec", 0))
                continue
                
            # Find ROS2 field name
            if json_field_name in field_types:
                ros_field_name = json_field_name
            else:
                ros_field_name = bsk_field_mapping.get(json_field_name.lower())
            
            if ros_field_name:
                attr_spec = field_types[ros_field_name]
                if attr_spec.startswith('float64[') and isinstance(field_value, list):
                    setattr(msg, ros_field_name, [float(x) for x in field_value])
                else:
                    setattr(msg, ros_field_name, field_value)
        return msg

    def _bsk_msg_to_json(self, msg):
        """Convert BSK message to JSON dict using cached field access."""
        json_data = {}
        
        # Get field types once and iterate
        for field_name in msg.get_fields_and_field_types():
            field_value = getattr(msg, field_name)
            
            # Handle special types efficiently
            if hasattr(field_value, 'sec') and hasattr(field_value, 'nanosec'):
                json_data[field_name] = {'sec': int(field_value.sec), 'nanosec': int(field_value.nanosec)}
            elif hasattr(field_value, 'tolist'):
                json_data[field_name] = field_value.tolist()
            else:
                json_data[field_name] = field_value
        
        return json_data

    def setup_zmq(self):
        """Initialize ZMQ sockets for Basilisk <-> ROS2 communication and heartbeat."""
        self.sub_socket = self.zmq_context.socket(zmq.SUB)
        self.sub_socket.connect(f"tcp://localhost:{self.sub_port}")
        self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.sub_socket.setsockopt(zmq.CONFLATE, 1)
        self.sub_socket.setsockopt(zmq.RCVTIMEO, 100)
        self.sub_socket.setsockopt(zmq.RCVHWM, 10)

        self.pub_socket = self.zmq_context.socket(zmq.PUB)
        self.pub_socket.bind(f"tcp://*:{self.pub_port}")
        self.pub_socket.setsockopt(zmq.LINGER, 0)
        self.pub_socket.setsockopt(zmq.SNDHWM, 50)
        self.pub_socket.setsockopt(zmq.IMMEDIATE, 1)

        self.heartbeat_pub = self.zmq_context.socket(zmq.PUB)
        self.heartbeat_pub.bind(f"tcp://*:{self.heartbeat_port}")
        self.heartbeat_pub.setsockopt(zmq.LINGER, 0)
        self.heartbeat_pub.setsockopt(zmq.SNDHWM, 1)

        # Pre-serialize heartbeat message as bytes
        self.heartbeat_msg = json_dumps({"msg": "ping"})

        def heartbeat_thread():
            while not self.stop_event.wait(HEARTBEAT_INTERVAL_SEC):
                try:
                    self.heartbeat_pub.send(self.heartbeat_msg, flags=zmq.NOBLOCK)
                except zmq.error.Again:
                    pass
                except Exception:
                    break
                    
        self.heartbeat_thread = threading.Thread(target=heartbeat_thread, daemon=True)
        self.heartbeat_thread.start()
        self.get_logger().info("Bridge ZMQ sockets ready.")

    def zmq_listener(self):
        """Thread: Listen for ZMQ messages and publish to ROS topics."""
        self.get_logger().debug("ZMQ listener thread started")
        while not self.stop_event.is_set() and rclpy.ok():
            try:
                msg_bytes = self.sub_socket.recv(flags=zmq.NOBLOCK)  # Receive as bytes
                data = json_loads(msg_bytes)
                
                # Validate message structure
                if not isinstance(data, dict):
                    continue

                # Handle subscription requests from BSK
                if data.get('subscription_request', False):
                    self._handle_subscription_request(data)
                elif 'sim_time' in data:
                    sim_time = data['sim_time']
                    if sim_time is not None:
                        # Reuse pre-allocated message
                        self._sim_time_msg.data = float(sim_time)
                        self.sim_time_pub.publish(self._sim_time_msg)
                else:
                    # Handle BSK messages
                    msg_type_name = data.get('msg_type')
                    if msg_type_name and msg_type_name in self._bsk_msg_types:
                        namespace = data.get('namespace', 'default')
                        self._handle_bsk_message(data, namespace, msg_type_name)
                    
            except zmq.error.Again:
                continue  # No message available, continue loop
            except orjson.JSONDecodeError:
                self.get_logger().debug("Invalid JSON received")
            except Exception as e:
                self.get_logger().error(f"Error in ZMQ listener: {e}")
                break
        self.get_logger().debug("ZMQ listener thread exiting")

    def _handle_subscription_request(self, data):
        """Handle subscription request from BSK and create ROS2 subscriber."""
        try:
            namespace = data.get('namespace', 'default')
            msg_type_name = data.get('msg_type')
            topic_name = data.get('topic_name')
            request_id = data.get('request_id')
            
            if not all([namespace, msg_type_name, topic_name]):
                self.get_logger().warn("Incomplete subscription request data")
                return
            
            # Construct full topic name for subscription (BSK input topics)
            full_topic_name = f"/{namespace}/bsk/in/{topic_name}"
            
            # Initialize namespace dict if needed
            if namespace not in self.bridge_subscribers:
                self.bridge_subscribers[namespace] = {}
            
            success = False
            
            # Only create subscriber if it doesn't exist
            if topic_name not in self.bridge_subscribers[namespace]:
                if msg_type_name in self._bsk_msg_types:
                    msg_type = self._bsk_msg_types[msg_type_name]
                    
                    # Create callback that captures the message type name and namespace
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
                        self.get_logger().info(f"Created subscriber for {full_topic_name} ({msg_type_name}) on request from BSK")
                        success = True
                        
                    except Exception as e:
                        self.get_logger().error(f"Could not create subscriber for {full_topic_name}: {e}")
                else:
                    self.get_logger().warn(f"Unknown BSK message type in subscription request: {msg_type_name}")
            else:
                self.get_logger().debug(f"Subscriber for {full_topic_name} already exists")
                success = True  # Already exists, so consider it successful
            
            # Send confirmation back to BSK if request_id provided
            if request_id and success:
                confirmation = {
                    "subscription_confirmation": True,
                    "namespace": namespace,
                    "msg_type": msg_type_name,
                    "topic_name": topic_name,
                    "request_id": request_id,
                    "success": True
                }
                
                try:
                    self.pub_socket.send(json_dumps(confirmation), flags=zmq.NOBLOCK)
                    self.get_logger().debug(f"Sent subscription confirmation for {request_id}")
                except Exception as e:
                    self.get_logger().error(f"Failed to send subscription confirmation: {e}")
                
        except Exception as e:
            self.get_logger().error(f"Error handling subscription request: {e}")

    def _handle_bsk_message(self, data, namespace, msg_type_name):
        """Handle incoming BSK message and publish to appropriate ROS topic."""
        try:
            # Validate input data first
            if not isinstance(data, dict):
                self.get_logger().warn(f"Invalid data type for BSK message: {type(data)}")
                return
                
            if not msg_type_name:
                self.get_logger().warn("Missing msg_type_name in BSK message")
                return
                
            if msg_type_name not in self._bsk_msg_types:
                self.get_logger().warn(f"Unknown BSK message type: {msg_type_name}")
                return

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
            
        except Exception:
            pass

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
            
            json_bytes = json_dumps(data)  # Returns bytes directly
            self.pub_socket.send(json_bytes, flags=zmq.NOBLOCK)
            self.get_logger().debug(f"Sent to Basilisk: {msg_type_name} for {namespace}")
            
        except zmq.error.Again:
            self.get_logger().debug("ZMQ send would block, message dropped")
        except Exception as e:
            self.get_logger().error(f"Error in ROS to Basilisk callback: {e}")

    def shutdown(self):
        """Cleanly shutdown bridge, sockets, and node."""
        self.get_logger().info("Shutting down bridge...")
        self.stop_event.set()
        
        try:
            for socket in [self.sub_socket, self.pub_socket, self.heartbeat_pub]:
                socket.setsockopt(zmq.LINGER, 0)
                socket.close()
            self.zmq_context.term()
        except Exception as e:
            self.get_logger().error(f"Error closing ZMQ: {e}")
        
        # Wait for threads with shorter timeout
        for thread in [self.listener_thread, self.heartbeat_thread]:
            if thread.is_alive():
                thread.join(timeout=1.0)
        
        super().destroy_node()
        self.get_logger().info("Bridge shutdown complete")

    def __enter__(self):
        """Context manager entry."""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit with cleanup."""
        self.shutdown()
        return False

def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        with BskRosBridge() as bridge:
            rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass  # Normal shutdown
    except Exception as e:
        print(f"Bridge error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()