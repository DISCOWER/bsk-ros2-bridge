"""
BSK-ROS2 Bridge: Bidirectional communication bridge between Basilisk and ROS2.

This bridge enables real-time data exchange between the Basilisk astrodynamics 
simulator and ROS2 ecosystem via ZeroMQ messaging with JSON serialization.
"""
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
import bsk_msgs.msg as bsk_msgs
import zmq
import threading
import orjson  # Faster JSON processing than standard json
import inspect
import re
import time

# =============================================================================
# COMMUNICATION CONFIGURATION
# =============================================================================
DEFAULT_SUB_PORT = 5550              # Receive telemetry from Basilisk
DEFAULT_PUB_PORT = 5551              # Send commands to Basilisk  
DEFAULT_HEARTBEAT_PORT = 5552        # Connection health monitoring
HEARTBEAT_INTERVAL_SEC = 0.1         # Keep-alive frequency
DEFAULT_ROS_CLOCK_TIMESTEP = 0.01    # Default ROS clock update interval

# =============================================================================
# PERFORMANCE OPTIMIZATIONS
# =============================================================================
# Pre-compiled regex for efficient camelCase to snake_case conversion
CAMEL_TO_SNAKE_RE = re.compile(r'(?<!^)(?=[A-Z])')

def json_loads(data): 
    """Fast JSON deserialization using orjson."""
    return orjson.loads(data)

def json_dumps(data): 
    """Fast JSON serialization using orjson - returns bytes directly."""
    return orjson.dumps(data)


class BskRosBridge(Node):
    """
    Bidirectional bridge between Basilisk simulator and ROS2.
    
    Architecture:
    - ZMQ SUB socket: Receives telemetry data from BSK
    - ZMQ PUB socket: Sends commands/data to BSK  
    - Dynamic topic creation: Auto-discovers BSK message types
    - Namespace support: Multiple spacecraft/simulation instances
    """
    
    # Class-level caches for performance optimization
    _field_mapping_cache = {}  # Message field name mappings (lowercase lookup)
    _topic_name_cache = {}     # Topic name conversions (camelCase -> snake_case)

    def __init__(self):
        super().__init__('bsk_ros2_bridge')
        
        # Threading control
        self.stop_event = threading.Event()
        
        # Port configuration - allows runtime override via ROS parameters
        self._setup_parameters()
        
        # Communication infrastructure
        self.bridge_publishers = {}   # namespace -> {topic_name -> {publisher, type}}
        self.bridge_subscribers = {}  # namespace -> {topic_name -> subscriber}
        self.zmq_context = zmq.Context()
        
        # Clock synchronization
        self._current_sim_time = 0.0
        self._last_published_time = 0.0  # Track last published simulation time
        self._accelFactor = 1.0
        self._last_clock_update = self.get_clock().now()
        
        # Message type discovery and caching
        self._bsk_msg_types = {}      # BSK message type registry
        self._clock_msg = Clock()  # Pre-allocated for performance
        
        # Initialize bridge components
        self._discover_bsk_message_types()
        self._setup_zmq_communication()
        self._start_background_threads()
        
        # Special topic for simulation time synchronization
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        
        # Start clock update timer
        self._start_clock_timer()
        
        self.get_logger().info(
            f"BSK-ROS2 Bridge ready on ports {self.sub_port}/{self.pub_port}/{self.heartbeat_port}"
        )

    # =========================================================================
    # INITIALIZATION METHODS
    # =========================================================================
    
    def _setup_parameters(self):
        """Configure ROS parameters for port settings."""
        self.declare_parameter('sub_port', DEFAULT_SUB_PORT)
        self.declare_parameter('pub_port', DEFAULT_PUB_PORT)
        self.declare_parameter('heartbeat_port', DEFAULT_HEARTBEAT_PORT)
        self.declare_parameter('ros_clock_timestep', DEFAULT_ROS_CLOCK_TIMESTEP)

        self.sub_port = self.get_parameter('sub_port').get_parameter_value().integer_value
        self.pub_port = self.get_parameter('pub_port').get_parameter_value().integer_value
        self.heartbeat_port = self.get_parameter('heartbeat_port').get_parameter_value().integer_value
        self.ros_clock_timestep = self.get_parameter('ros_clock_timestep').get_parameter_value().double_value

    def _discover_bsk_message_types(self):
        """
        Auto-discover all BSK message types for dynamic topic creation.
        This enables the bridge to handle any BSK message without manual registration.
        """
        try:            
            # Find all classes ending with 'MsgPayload' (BSK convention)
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

    def _setup_zmq_communication(self):
        """
        Initialize ZMQ sockets with performance optimizations.
        
        Socket configuration rationale:
        - CONFLATE: Only keep latest message (real-time data)
        - Low HWM: Prevent memory buildup under high load
        - NOBLOCK: Non-blocking operations for responsive bridge
        """
        # Subscriber: Receive telemetry from BSK
        self.sub_socket = self.zmq_context.socket(zmq.SUB)
        self.sub_socket.connect(f"tcp://localhost:{self.sub_port}")
        self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all messages
        self.sub_socket.setsockopt(zmq.CONFLATE, 1)           # Keep only latest message
        self.sub_socket.setsockopt(zmq.RCVTIMEO, 100)         # 100ms receive timeout
        self.sub_socket.setsockopt(zmq.RCVHWM, 10)            # Low receive buffer

        # Publisher: Send commands to BSK
        self.pub_socket = self.zmq_context.socket(zmq.PUB)
        self.pub_socket.bind(f"tcp://*:{self.pub_port}")
        self.pub_socket.setsockopt(zmq.LINGER, 0)             # No blocking on close
        self.pub_socket.setsockopt(zmq.SNDHWM, 50)            # Send buffer limit
        self.pub_socket.setsockopt(zmq.IMMEDIATE, 1)          # Immediate delivery

        # Heartbeat: Connection monitoring
        self.heartbeat_pub = self.zmq_context.socket(zmq.PUB)
        self.heartbeat_pub.bind(f"tcp://*:{self.heartbeat_port}")
        self.heartbeat_pub.setsockopt(zmq.LINGER, 0)
        self.heartbeat_pub.setsockopt(zmq.SNDHWM, 1)          # Only latest heartbeat

        # Pre-serialize heartbeat message for efficiency
        self.heartbeat_msg = json_dumps({"msg": "ping"})
        
        self.get_logger().info("ZMQ communication sockets initialized")

    def _start_background_threads(self):
        """Start daemon threads for ZMQ message handling and heartbeat."""
        # Message listener thread
        self.listener_thread = threading.Thread(target=self._zmq_listener, daemon=True)
        self.listener_thread.start()
        
        # Heartbeat thread for connection monitoring
        self.heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self.heartbeat_thread.start()

    # =========================================================================
    # MESSAGE CONVERSION UTILITIES
    # =========================================================================
    
    def _bsk_type_to_topic_name(self, bsk_type_name):
        """
        Convert BSK message type to ROS topic name with caching.
        """
        if bsk_type_name in self._topic_name_cache:
            return self._topic_name_cache[bsk_type_name]
            
        # Remove BSK suffix and convert to snake_case
        name = bsk_type_name[:-10] if bsk_type_name.endswith('MsgPayload') else bsk_type_name
        topic_name = CAMEL_TO_SNAKE_RE.sub('_', name).lower()
        
        self._topic_name_cache[bsk_type_name] = topic_name
        return topic_name

    def _json_to_bsk_msg(self, json_data, msg_type):
        """
        Convert JSON data to BSK message using cached field mappings.
        Handles case-insensitive field matching and special timestamp conversion.
        """
        msg = msg_type()
        msg_type_name = msg_type.__name__

        # Use cached field mapping for performance
        if msg_type_name not in self._field_mapping_cache:
            field_types = msg.get_fields_and_field_types()
            bsk_field_mapping = {name.lower(): name for name in field_types}
            self._field_mapping_cache[msg_type_name] = (bsk_field_mapping, field_types)
        
        bsk_field_mapping, field_types = self._field_mapping_cache[msg_type_name]

        # Map JSON fields to ROS message fields
        for json_field_name, field_value in json_data.items():
            # Handle ROS timestamp format conversion
            if json_field_name == "stamp" and hasattr(msg, "stamp") and isinstance(field_value, dict):
                msg.stamp.sec = int(field_value.get("sec", 0))
                msg.stamp.nanosec = int(field_value.get("nanosec", 0))
                continue
                
            # Find corresponding ROS field (case-insensitive)
            ros_field_name = (json_field_name if json_field_name in field_types 
                            else bsk_field_mapping.get(json_field_name.lower()))
            
            if ros_field_name:
                attr_spec = field_types[ros_field_name]
                # Handle array fields
                if attr_spec.startswith('float64[') and isinstance(field_value, list):
                    setattr(msg, ros_field_name, [float(x) for x in field_value])
                else:
                    setattr(msg, ros_field_name, field_value)
        return msg

    def _bsk_msg_to_json(self, msg):
        """Convert BSK message to JSON dict with efficient field iteration."""
        json_data = {}
        
        for field_name in msg.get_fields_and_field_types():
            field_value = getattr(msg, field_name)
            
            # Handle special ROS types
            if hasattr(field_value, 'sec') and hasattr(field_value, 'nanosec'):
                json_data[field_name] = {'sec': int(field_value.sec), 'nanosec': int(field_value.nanosec)}
            elif hasattr(field_value, 'tolist'):
                json_data[field_name] = field_value.tolist()
            else:
                json_data[field_name] = field_value
        
        return json_data

    # =========================================================================
    # BACKGROUND THREAD METHODS
    # =========================================================================
    
    def _heartbeat_loop(self):
        """Background thread: Send periodic heartbeat to monitor connection health."""
        while not self.stop_event.wait(HEARTBEAT_INTERVAL_SEC):
            try:
                self.heartbeat_pub.send(self.heartbeat_msg, flags=zmq.NOBLOCK)
            except zmq.error.Again:
                pass  # Socket block, continue
            except Exception:
                break  # Socket error, exit thread

    def _zmq_listener(self):
        """
        Background thread: Process incoming ZMQ messages from BSK.
        Handles subscription requests, telemetry data, and simulation time updates.
        """
        self.get_logger().debug("ZMQ listener thread started")
        
        while not self.stop_event.is_set() and rclpy.ok():
            try:
                msg_bytes = self.sub_socket.recv(flags=zmq.NOBLOCK)  # Receive as bytes
                data = json_loads(msg_bytes)
                
                # Validate message structure
                if not isinstance(data, dict):
                    continue

                # Handle unified topic requests from BSK
                if data.get('topic_request', False):
                    self._handle_topic_request(data)
                elif data.get('clock_reset', False):
                    # Reset clock when a new handler initializes
                    self._current_sim_time = float(data.get('sim_time', 0.0))
                    self._last_published_time = self._current_sim_time
                    self._accelFactor = float(data.get('accelFactor', 1.0))
                    self.get_logger().info("Clock reset by new RosBridgeHandler")
                    # Send acknowledgement
                    ack = {
                        "clock_reset_ack": True,
                        "sim_time": self._current_sim_time,
                        "accelFactor": self._accelFactor
                    }
                    self.pub_socket.send(json_dumps(ack), flags=zmq.NOBLOCK)
                elif 'sim_time' in data:
                    self._handle_sim_time_update(data)
                else:
                    self._handle_telemetry_message(data)
                    
            except zmq.error.Again:
                time.sleep(0.01)
                continue  # No message available, continue loop
            except orjson.JSONDecodeError:
                self.get_logger().debug("Invalid JSON received from BSK")
            except Exception as e:
                self.get_logger().error(f"Error in ZMQ listener: {e}")
                break
                
        self.get_logger().debug("ZMQ listener thread exiting")

    # =========================================================================
    # MESSAGE HANDLING METHODS
    # =========================================================================
    
    def _handle_sim_time_update(self, data):
        """Handle simulation time synchronization from BSK."""
        sim_time = data['sim_time']
        if sim_time is not None:
            self._current_sim_time = float(sim_time)
            self._accelFactor = float(data.get('accelFactor', 1.0))
            # Update clock timer rate if it exists
            if hasattr(self, 'clock_timer'):
                timer_period = self.ros_clock_timestep / max(self._accelFactor, 1.0)
                self.clock_timer.timer_period_ns = int(timer_period * 1e9)
            
    def _update_ros_clock(self, sim_time):
        """Update ROS clock with the given simulation time."""
        self._clock_msg.clock.sec = int(sim_time)
        self._clock_msg.clock.nanosec = int((sim_time - int(sim_time)) * 1e9)
        self.clock_pub.publish(self._clock_msg)

    def _handle_telemetry_message(self, data):
        """
        Handle incoming telemetry data from BSK.
        Creates publishers dynamically and forwards data to ROS topics.
        """
        try:
            msg_type_name = data.get('msg_type')
            if not msg_type_name or msg_type_name not in self._bsk_msg_types:
                return
                
            namespace = data.get('namespace', 'default')
            
            # Use explicit topic name or derive from message type
            topic_name = data.get('topic_name', self._bsk_type_to_topic_name(msg_type_name))
            full_topic_name = f"/{namespace}/bsk/out/{topic_name}"
            
            # Initialize namespace tracking
            if namespace not in self.bridge_publishers:
                self.bridge_publishers[namespace] = {}
                if namespace not in self.bridge_subscribers:
                    self.bridge_subscribers[namespace] = {}
            
            # Create publisher on first message
            if topic_name not in self.bridge_publishers[namespace]:
                msg_type = self._bsk_msg_types[msg_type_name]
                publisher = self.create_publisher(msg_type, full_topic_name, 10)
                self.bridge_publishers[namespace][topic_name] = {
                    'publisher': publisher,
                    'type': msg_type
                }
                # self.get_logger().info(f"Created publisher for {full_topic_name} ({msg_type_name})")
            
            # Convert and publish message
            pub_info = self.bridge_publishers[namespace][topic_name]
            bsk_msg = self._json_to_bsk_msg(data, pub_info['type'])
            pub_info['publisher'].publish(bsk_msg)
            
        except Exception:
            pass  # Silent failure to avoid log spam on bad messages

    def _handle_topic_request(self, data):
        """
        Handle dynamic topic requests from BSK.
        Creates ROS subscribers for "in" direction or confirms publishers for "out" direction.
        """
        try:
            namespace = data.get('namespace', 'default')
            msg_type_name = data.get('msg_type')
            topic_name = data.get('topic_name')
            direction = data.get('direction', 'in')  # Default to "in" for backward compatibility
            request_id = data.get('request_id')
            
            if not all([namespace, msg_type_name, topic_name, direction]):
                self.get_logger().warn("Incomplete topic request from BSK")
                return
            
            success = False
            
            if direction == "in":
                # BSK wants to receive data from ROS2 - create subscriber
                success = self._create_bsk_input_subscriber(namespace, msg_type_name, topic_name)
            elif direction == "out":
                # BSK wants to send data to ROS2 - confirm publisher will be created on demand
                success = self._confirm_bsk_output_publisher(namespace, msg_type_name, topic_name)
            else:
                self.get_logger().warn(f"Unknown direction '{direction}' in topic request")
                return

            # Send confirmation back to BSK
            self._send_topic_confirmation(request_id, namespace, msg_type_name, topic_name, direction, success)
                
        except Exception as e:
            self.get_logger().error(f"Error handling topic request: {e}")

    def _create_bsk_input_subscriber(self, namespace, msg_type_name, topic_name):
        """Create ROS subscriber for BSK input (ROS2 -> BSK)."""
        full_topic_name = f"/{namespace}/bsk/in/{topic_name}"
        
        # Initialize namespace tracking
        if namespace not in self.bridge_subscribers:
            self.bridge_subscribers[namespace] = {}
        
        # Create subscriber if it doesn't exist
        if topic_name not in self.bridge_subscribers[namespace]:
            if msg_type_name in self._bsk_msg_types:
                msg_type = self._bsk_msg_types[msg_type_name]
                
                # Create callback with captured context
                def create_callback(msg_type_name, namespace):
                    def callback(msg):
                        self._ros_to_basilisk_callback(msg, namespace, msg_type_name)
                    return callback
                
                try:
                    subscriber = self.create_subscription(
                        msg_type,
                        full_topic_name,
                        create_callback(msg_type_name, namespace),
                        10
                    )
                    
                    self.bridge_subscribers[namespace][topic_name] = subscriber
                    # self.get_logger().info(f"Confirmed BSK subscriber: {full_topic_name} ({msg_type_name})")
                    return True
                    
                except Exception as e:
                    self.get_logger().error(f"Could not create subscriber for {full_topic_name}: {e}")
                    return False
            else:
                self.get_logger().warn(f"Unknown BSK message type: {msg_type_name}")
                return False
        else:
            return True  # Already exists

    def _confirm_bsk_output_publisher(self, namespace, msg_type_name, topic_name):
        """Confirm that BSK output publisher will be created on demand."""
        full_topic_name = f"/{namespace}/bsk/out/{topic_name}"
        
        # Initialize namespace tracking
        if namespace not in self.bridge_publishers:
            self.bridge_publishers[namespace] = {}
        
        # Check if message type is valid
        if msg_type_name not in self._bsk_msg_types:
            self.get_logger().warn(f"Unknown BSK message type: {msg_type_name}")
            return False
        
        # Publisher will be created on demand when first message arrives
        # self.get_logger().info(f"Confirmed BSK publisher: {full_topic_name} ({msg_type_name}) - will create publisher on demand")
        return True

    def _send_topic_confirmation(self, request_id, namespace, msg_type_name, topic_name, direction, success):
        """Send topic confirmation back to BSK."""
        if not request_id or not success:
            return
            
        confirmation = {
            "topic_confirmation": True,
            "namespace": namespace,
            "msg_type": msg_type_name,
            "topic_name": topic_name,
            "direction": direction,
            "request_id": request_id,
            "success": True
        }
        
        try:
            self.pub_socket.send(json_dumps(confirmation), flags=zmq.NOBLOCK)
            action = "subscription" if direction == "in" else "publisher"
            self.get_logger().debug(f"Sent {action} confirmation for {request_id}")
        except Exception as e:
            self.get_logger().error(f"Failed to send topic confirmation: {e}")

    def _start_clock_timer(self):
        """Initialize and start the clock update timer."""
        # Calculate timer period in wall clock time to achieve desired sim time updates
        # Example: For 0.01s sim time steps at 100x speed:
        # - We want updates every 0.01 sim seconds
        # - Wall clock period = sim_timestep / accelFactor = 0.01/100 = 0.0001s
        timer_period = self.ros_clock_timestep / max(self._accelFactor, 1e-6)  # Avoid division by zero or slow-motion
        self.clock_timer = self.create_timer(timer_period, self._clock_timer_callback)
        
    def _clock_timer_callback(self):
        """Update the ROS clock based on simulation time and speed."""
        if not self.stop_event.is_set():
            current_time = self.get_clock().now()
            dt = (current_time - self._last_clock_update).nanoseconds * 1e-9
            
            # Update simulation time based on accelFactor
            self._current_sim_time += dt * self._accelFactor
            
            # Only publish if time has advanced beyond last published time
            if self._current_sim_time >= self._last_published_time:
                self._update_ros_clock(self._current_sim_time)
                self._last_published_time = self._current_sim_time
            
            self._last_clock_update = current_time

    def _ros_to_basilisk_callback(self, msg, namespace, msg_type_name):
        """
        Forward ROS messages to BSK via ZMQ.
        Called when ROS publishes to topics that BSK is subscribed to.
        """
        try:
            # Convert to JSON with metadata
            json_data = self._bsk_msg_to_json(msg)
            data = {
                'namespace': namespace,
                'msg_type': msg_type_name,
                'timestamp': self.get_clock().now().nanoseconds,
                **json_data
            }
            
            # Send to BSK
            json_bytes = json_dumps(data)
            self.pub_socket.send(json_bytes, flags=zmq.NOBLOCK)
            self.get_logger().debug(f"Sent to Basilisk: {msg_type_name} for {namespace}")
            
        except zmq.error.Again:
            self.get_logger().debug("ZMQ send buffer full, message dropped")
        except Exception as e:
            self.get_logger().error(f"Error in ROS->BSK callback: {e}")

    # =========================================================================
    # LIFECYCLE MANAGEMENT
    # =========================================================================
    
    def shutdown(self):
        """Clean shutdown of bridge components."""
        self.get_logger().info("Shutting down BSK-ROS2 Bridge...")
        self.stop_event.set()
        
        # Close ZMQ sockets
        try:
            for socket in [self.sub_socket, self.pub_socket, self.heartbeat_pub]:
                socket.setsockopt(zmq.LINGER, 0)
                socket.close()
            self.zmq_context.term()
        except Exception:
            pass
        
        # Wait for background threads
        for thread in [self.listener_thread, self.heartbeat_thread]:
            if thread.is_alive():
                thread.join(timeout=1.0)
        
        super().destroy_node()

    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.shutdown()
        return False


def main(args=None):
    """Main entry point for BSK-ROS2 Bridge."""
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