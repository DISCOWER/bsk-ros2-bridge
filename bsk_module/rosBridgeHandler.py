# =============================================================================
# BASILISK-ROS2 BRIDGE HANDLER
# =============================================================================
# Core Basilisk imports
from Basilisk.architecture import sysModel, messaging
import Basilisk.architecture.messaging as messaging_pkg

# Communication and data processing
import zmq
import threading
import time
import orjson  # Faster JSON processing than standard json
import numpy as np
import re

# Module discovery
import inspect
import importlib
import pkgutil

# =============================================================================
# GLOBAL STATE AND OPTIMIZATIONS
# =============================================================================
# Shared ZMQ resources for multi-spacecraft scenarios
_shared_context = None
_shared_sockets = {}

# Pre-compiled regex for better performance
NORMALIZE_FIELD_RE = re.compile(r'_')

# =============================================================================
# JSON UTILITIES - Using orjson for performance
# =============================================================================
def json_loads(data):
    """Fast JSON deserialization using orjson."""
    return orjson.loads(data)

def json_dumps(data):
    """Fast JSON serialization using orjson - returns bytes directly."""
    return orjson.dumps(data)


class RosBridgeHandler(sysModel.SysModel):
    """
    Basilisk-ROS2 Bridge Handler - Enables bidirectional communication between
    Basilisk spacecraft simulation and ROS2 ecosystem via ZMQ.
    
    Key Features:
    - Automatic BSK message type discovery and conversion
    - Dynamic topic creation based on message types
    - Multi-spacecraft namespace support
    - Robust connection monitoring with heartbeat
    - Performance optimized with caching and pre-compilation
    
    Architecture:
    - Inherits from SysModel (Basilisk module pattern)
    - Uses ZMQ for low-latency communication with bridge
    - JSON serialization for cross-language compatibility
    - Background threads for heartbeat monitoring
    """
    
    # Performance caches - shared across instances
    _field_mapping_cache = {}
    _bsk_attrs_cache = {}

    def __init__(self, namespace="spacecraft1", send_port=5550, receive_port=5551, 
                 heartbeat_port=5552):
        """
        Initialize ROS Bridge Handler.

        Args:
            namespace (str): Spacecraft namespace for topic routing
            send_port (int): ZMQ port for sending data to bridge (BSK -> ROS2)
            receive_port (int): ZMQ port for receiving data from bridge (ROS2 -> BSK)
            heartbeat_port (int): ZMQ port for heartbeat monitoring
        """
        super(RosBridgeHandler, self).__init__()

        # Configuration
        self.namespace = namespace
        self.send_port = send_port
        self.receive_port = receive_port
        self.heartbeat_port = heartbeat_port
        
        # Dynamic message management
        self.input_msg_readers = {}    # BSK -> ROS2 publishers
        self.output_msg_writers = {}   # ROS2 -> BSK subscribers
        
        # Topic handshaking - ensures ROS2 topics are ready
        self.pending_topics = {}
        self.confirmed_topics = set()
        
        # Pre-allocate commonly used objects to reduce allocations
        self._sim_time_msg_template = {"sim_time": 0.0}
        # Pre-serialize for better performance
        self._topic_request_template = {
            "topic_request": True,
            "namespace": self.namespace,
            "msg_type": "",
            "topic_name": "",
            "direction": "",  # "in" or "out"
            "request_id": ""
        }
        
        # State tracking
        self.last_heartbeat_time = None
        self._running = True
        self._heartbeat_was_lost = False
        self._initialization_complete = False
        
        # Initialize components
        self._discover_bsk_messages()
        self._setup_zmq_communication()
        self._wait_for_bridge_connection()
        
        self._initialization_complete = True

    # =========================================================================
    # INITIALIZATION AND DISCOVERY
    # =========================================================================
    
    def _discover_bsk_messages(self):
        """Automatically discover all BSK message types in Basilisk.architecture.messaging."""
        self.bsk_msg_types = {}

        try:
            for _, modname, _ in list(pkgutil.walk_packages(messaging_pkg.__path__, messaging_pkg.__name__ + ".")):
                try:
                    mod = importlib.import_module(modname)
                    for name, obj in inspect.getmembers(mod, inspect.isclass):
                        # Register all MsgPayload, Msg, and MsgReader classes in the messaging namespace
                        if (name.endswith('MsgPayload') or name.endswith('Msg') or name.endswith('MsgReader')) and hasattr(obj, '__init__'):
                            setattr(messaging, name, obj)
                        # Only MsgPayload classes are used for type discovery
                        if name.endswith('MsgPayload') and hasattr(obj, '__init__'):
                            self.bsk_msg_types[name] = obj
                except Exception:
                    continue
        except Exception as e:
            print(f"[{self.namespace}] ERROR: Could not import Basilisk.architecture.messaging: {e}")

        print(f"[{self.namespace}] Discovered {len(self.bsk_msg_types)} BSK message types")

    def _setup_zmq_communication(self):
        """Initialize ZMQ sockets with shared context for multi-spacecraft support."""
        global _shared_context, _shared_sockets
        
        if _shared_context is None:
            _shared_context = zmq.Context()
        self.context = _shared_context
        
        self._setup_heartbeat_monitoring()
        self._setup_data_publisher()
        self._setup_command_subscriber()
        
    def _setup_heartbeat_monitoring(self):
        """Setup background heartbeat monitoring for connection health."""
        self.heartbeat_sub = self.context.socket(zmq.SUB)
        self.heartbeat_sub.connect(f"tcp://localhost:{self.heartbeat_port}")
        self.heartbeat_sub.setsockopt_string(zmq.SUBSCRIBE, "")
        self.heartbeat_sub.setsockopt(zmq.RCVTIMEO, 10)

        def heartbeat_monitor():
            """Background thread - monitors bridge health via heartbeat."""
            while self._running:
                try:
                    self.heartbeat_sub.recv_string(flags=zmq.NOBLOCK)
                    self.last_heartbeat_time = time.time()
                except zmq.Again:
                    time.sleep(0.01)
                except (zmq.ZMQError, zmq.ContextTerminated) as e:
                    if self._running and self._initialization_complete:
                        print(f"[{self.namespace}] Heartbeat monitoring stopped: {e}")
                    break
                except Exception as e:
                    if self._initialization_complete and self._running:
                        print(f"[{self.namespace}] Heartbeat monitoring error: {e}")
                    time.sleep(0.1)
        
        self.heartbeat_thread = threading.Thread(target=heartbeat_monitor, daemon=True)
        self.heartbeat_thread.start()
        
    def _setup_data_publisher(self):
        """Setup shared ZMQ publisher for BSK -> ROS2 data flow."""
        global _shared_sockets
        
        socket_key = f"send_{self.send_port}"
        
        if socket_key not in _shared_sockets:
            self.send_socket = self.context.socket(zmq.PUB)
            self.send_socket.setsockopt(zmq.LINGER, 0)
            self.send_socket.setsockopt(zmq.SNDHWM, 100)  # Prevent memory buildup
            
            try:
                self.send_socket.bind(f"tcp://*:{self.send_port}")
                _shared_sockets[socket_key] = self.send_socket
                print(f"[{self.namespace}] Created and bound shared publisher on port {self.send_port}")
            except zmq.ZMQError as e:
                raise RuntimeError(f"Failed to bind to port {self.send_port}: {e}")
        else:
            self.send_socket = _shared_sockets[socket_key]
            print(f"[{self.namespace}] Using shared publisher on port {self.send_port}")

    def _setup_command_subscriber(self):
        """Setup ZMQ subscriber for ROS2 -> BSK commands."""
        self.receive_socket = self.context.socket(zmq.SUB)
        self.receive_socket.connect(f"tcp://localhost:{self.receive_port}")
        self.receive_socket.setsockopt_string(zmq.SUBSCRIBE, '')
        self.receive_socket.setsockopt(zmq.CONFLATE, 1)  # Only keep latest
        self.receive_socket.setsockopt(zmq.RCVTIMEO, 1)   # Non-blocking

    def _wait_for_bridge_connection(self):
        """Block until bridge heartbeat detected - ensures bridge is ready."""
        print(f"[{self.namespace}] Waiting for bridge heartbeat...")
        
        while self.last_heartbeat_time is None:
            time.sleep(0.01)
            
        print(f"[{self.namespace}] Bridge heartbeat detected. Module ready.")

    # =========================================================================
    # SET UP MESSAGE HANDLING
    # =========================================================================

    def add_bsk_msg_handler(self, msg_type_name, direction, handler_name, topic_name):
        """
        Add BSK message handler for automatic communication with ROS2.
        
        Args:
            msg_type_name (str): BSK message type name (e.g., 'SCStatesMsgPayload')
            direction (str): "out" for BSK -> ROS2 (readers), "in" for ROS2 -> BSK (writers)
            handler_name (str): Name for the handler (e.g., 'scStateInMsg')
            topic_name (str): Topic name (e.g., 'imu_1', 'imu_2')
        """
        if msg_type_name not in self.bsk_msg_types:
            self._log_error(f"Unknown BSK message type: {msg_type_name}")
            return
            
        if direction not in ["in", "out"]:
            self._log_error(f"Invalid direction '{direction}'. Must be 'in' or 'out'")
            return
        
        # Generate default names based on direction
        if direction == "out":
            handler_name = handler_name or f"{msg_type_name.lower()}_reader"
            class_suffix = "MsgReader"
            storage_dict = self.input_msg_readers
            arrow = "->"
        else:  # direction == "in"
            handler_name = handler_name or f"{msg_type_name.lower()}_writer"
            class_suffix = "Msg"
            storage_dict = self.output_msg_writers
            arrow = "<-"
        
        # Create handler class
        try:
            handler_class_name = msg_type_name.replace('MsgPayload', class_suffix)
            handler_class = getattr(messaging, handler_class_name)
            handler = handler_class()
        except AttributeError:
            self._log_error(f"No {class_suffix.lower()} found for message type: {msg_type_name} (tried {handler_class_name})")
            return
        
        # Store handler configuration
        handler_key = "reader" if direction == "out" else "writer"
        storage_dict[handler_name] = {
            handler_key: handler,
            'msg_type': msg_type_name,
            'topic_name': topic_name
        }
        
        # Make handler accessible as attribute
        setattr(self, handler_name, handler)
        
        # For "in" direction (ROS2 -> BSK), initialize with zero message to prevent read errors
        if direction == "in":
            zero_msg = self._create_zero_message(msg_type_name)
            if zero_msg is not None:
                try:
                    handler.write(zero_msg, 0, 0)  # Write at time 0 with moduleID 0
                    self._log_info(f"Initialized {handler_name} with zero message to prevent read errors")
                except Exception as e:
                    self._log_warning(f"Failed to initialize {handler_name} with zero message: {e}")
        
        # Send topic request to bridge
        self._send_topic_request(msg_type_name, topic_name, direction)
        
        self._log_info(f"Added BSK message {handler_key}: {handler_name} ({msg_type_name}) {arrow} /{self.namespace}/bsk/{direction}/{topic_name}")
        return

    # =========================================================================
    # TOPIC REQUEST MANAGEMENT
    # =========================================================================

    def _send_topic_request(self, msg_type_name, topic_name, direction):
        """Send topic request to bridge for automatic ROS2 topic creation."""
        request_id = f"{self.namespace}_{msg_type_name}_{topic_name}_{direction}"
        
        self._topic_request_template.update({
            "msg_type": msg_type_name,
            "topic_name": topic_name,
            "direction": direction,
            "request_id": request_id
        })
        
        self.pending_topics[request_id] = {
            "msg_type": msg_type_name,
            "topic_name": topic_name,
            "direction": direction,
            "attempts": 0,
            "last_attempt": time.time()
        }
        
        try:
            self.send_socket.send(json_dumps(self._topic_request_template), flags=zmq.NOBLOCK)
        except Exception as e:
            self._log_error(f"Error sending topic request: {e}")

    def _check_pending_topics(self):
        """Check for pending topic requests and retry if needed."""
        current_time = time.time()
        max_attempts = 10
        retry_interval = 2.0
        
        for request_id in list(self.pending_topics.keys()):
            if request_id in self.confirmed_topics:
                del self.pending_topics[request_id]
                continue
            
            request_info = self.pending_topics[request_id]
            
            if current_time - request_info["last_attempt"] > retry_interval:
                if request_info["attempts"] < max_attempts:
                    request_info["attempts"] += 1
                    request_info["last_attempt"] = current_time
                    
                    self._topic_request_template.update({
                        "msg_type": request_info["msg_type"],
                        "topic_name": request_info["topic_name"],
                        "direction": request_info["direction"],
                        "request_id": request_id
                    })
                    
                    try:
                        self.send_socket.send(json_dumps(self._topic_request_template), flags=zmq.NOBLOCK)
                        if request_info['attempts'] > 1:
                            action = "subscription" if request_info["direction"] == "in" else "publisher"
                            print(f"[{self.namespace}] Retrying {action} request {request_id} (attempt {request_info['attempts']}/{max_attempts})")
                    except Exception as e:
                        print(f"ERROR: Failed to retry topic request {request_id}: {e}")
                else:
                    # Max attempts reached
                    action = "subscription" if request_info["direction"] == "in" else "publisher"
                    print(f"[{self.namespace}] WARNING: {action.capitalize()} request {request_id} failed after {max_attempts} attempts")
                    del self.pending_topics[request_id]

    # =========================================================================
    # MESSAGE CONVERSION
    # =========================================================================

    def _create_zero_message(self, msg_type_name):
        """Create a BSK message with zero/default values to prevent read errors."""
        try:
            msg_payload = self.bsk_msg_types[msg_type_name]()
            
            # Initialize numeric arrays and scalars to zero
            for attr_name in dir(msg_payload):
                if (not attr_name.startswith('_') and attr_name not in {'this', 'thisown'} 
                    and not callable(getattr(msg_payload, attr_name, None))):
                    try:
                        attr_value = getattr(msg_payload, attr_name)
                        if isinstance(attr_value, np.ndarray):
                            # Zero out numpy arrays
                            setattr(msg_payload, attr_name, np.zeros_like(attr_value))
                        elif isinstance(attr_value, (int, float)):
                            # Zero out numeric scalars
                            setattr(msg_payload, attr_name, 0.0 if isinstance(attr_value, float) else 0)
                        elif isinstance(attr_value, (list, tuple)) and len(attr_value) > 0:
                            # Zero out numeric lists/tuples
                            if isinstance(attr_value[0], (int, float)):
                                zero_list = [0.0 if isinstance(x, float) else 0 for x in attr_value]
                                setattr(msg_payload, attr_name, zero_list)
                    except Exception:
                        continue
            
            return msg_payload
        except Exception as e:
            self._log_error(f"Failed to create zero message for {msg_type_name}: {e}")
            return None

    def _bsk_msg_to_json(self, msg_payload, msg_type_name, CurrentSimNanos=None):
        """Convert BSK message payload to JSON dict with caching for better performance."""
        if msg_type_name not in self._bsk_attrs_cache:
            valid_attrs = [attr for attr in dir(msg_payload) 
                          if not attr.startswith('_') and attr not in {'this', 'thisown'} 
                          and not callable(getattr(msg_payload, attr, None))]
            self._bsk_attrs_cache[msg_type_name] = valid_attrs
        
        json_data = {}
        for attr_name in self._bsk_attrs_cache[msg_type_name]:
            try:
                attr_value = getattr(msg_payload, attr_name)
                if isinstance(attr_value, (int, float, str, bool)) or attr_value is None:
                    json_data[attr_name.lower()] = attr_value
                elif hasattr(attr_value, 'tolist'):
                    json_data[attr_name.lower()] = attr_value.tolist()
                elif isinstance(attr_value, (list, tuple)):
                    json_data[attr_name.lower()] = list(attr_value)
            except Exception:
                continue

        # Add ROS2 timestamp if requested
        if CurrentSimNanos is not None:
            json_data['stamp'] = {
                'sec': int(CurrentSimNanos // 1_000_000_000),
                'nanosec': int(CurrentSimNanos % 1_000_000_000)
            }

        return json_data

    def _json_to_bsk_msg(self, json_data, msg_type_name):
        """Convert JSON to BSK message with robust field matching."""
        msg_payload = self.bsk_msg_types[msg_type_name]()

        if msg_type_name not in self._field_mapping_cache:
            bsk_field_mapping = {}
            for attr_name in dir(msg_payload):
                if (not attr_name.startswith('_') and attr_name not in {'this', 'thisown'} 
                    and not callable(getattr(msg_payload, attr_name, None))):
                    bsk_field_mapping[NORMALIZE_FIELD_RE.sub('', attr_name).lower()] = attr_name
            self._field_mapping_cache[msg_type_name] = bsk_field_mapping
        
        bsk_field_mapping = self._field_mapping_cache[msg_type_name]

        # Skip metadata fields
        metadata_fields = {'namespace', 'msg_type', 'topic_name', 'time', 'timestamp', 'stamp'}
        
        for json_field_name, attr_value in json_data.items():
            if json_field_name in metadata_fields:
                continue

            bsk_field_name = bsk_field_mapping.get(NORMALIZE_FIELD_RE.sub('', json_field_name).lower())
            if bsk_field_name:
                try:
                    current_value = getattr(msg_payload, bsk_field_name)
                    if isinstance(current_value, np.ndarray):
                        attr_array = np.array(attr_value, dtype=current_value.dtype)
                        attr_array.shape = current_value.shape
                        setattr(msg_payload, bsk_field_name, attr_array)
                    else:
                        setattr(msg_payload, bsk_field_name, attr_value)
                except Exception:
                    continue

        return msg_payload

    # =========================================================================
    # BASILISK MODULE INTERFACE
    # =========================================================================

    def Reset(self, CurrentSimNanos):
        """
        Reset method called to initialize persistent data to ready state.
        
        Args:
            CurrentSimNanos (int): Current simulation time in nanoseconds
        """
        # Validate message connections - this follows Basilisk best practices
        for reader_name, reader_info in self.input_msg_readers.items():
            if not reader_info['reader'].isLinked():
                self._log_warning(f"Reader {reader_name} is not connected.")
            
        self._log_info(f"Reset complete for RosBridgeHandler namespace: {self.namespace}")
        return

    def UpdateState(self, CurrentSimNanos):
        """
        Cyclical worker method called at the specified task rate.
        Handles:
        - Heartbeat monitoring
        - Publishing spacecraft state to ROS2
        - Publishing simulation time to ROS2
        - Receiving control commands from ROS2
        - Updating output messages for Basilisk
        
        Args:
            CurrentSimNanos (int): Current simulation time in nanoseconds
        """
        # Check bridge heartbeat health
        if not self._check_bridge_health():
            return

        # Check and retry pending topic requests
        self._check_pending_topics()

        # Publish sim time (special topic, always published)
        self._publish_sim_time(CurrentSimNanos)

        # Process all input message readers and publish to ROS2
        for reader_name, reader_info in self.input_msg_readers.items():
            if reader_info['reader'].isLinked() and reader_info['reader'].isWritten():
                msg_payload = reader_info['reader']()
                self._publish_bsk_message(CurrentSimNanos, msg_payload, reader_info['msg_type'])

        # Receive and process commands from ROS2
        self._receive_ros_message(CurrentSimNanos)

    # =========================================================================
    # RUNTIME COMMUNICATION METHODS
    # =========================================================================

    def _check_bridge_health(self):
        """Check if bridge heartbeat is healthy."""
        now = time.time()
        if self.last_heartbeat_time is None or (now - self.last_heartbeat_time > 1.0):
            if not self._heartbeat_was_lost:
                print(f"[{self.namespace}] No bridge heartbeat for >1s! Pausing UpdateState.")
                self._heartbeat_was_lost = True
            return False
        else:
            if self._heartbeat_was_lost:
                print(f"[{self.namespace}] Bridge heartbeat restored. Resuming UpdateState.")
                self._heartbeat_was_lost = False
            return True
            
    def _publish_sim_time(self, CurrentSimNanos):
        """Publish simulation time for ROS2 synchronization."""
        try:
            # Reuse pre-allocated template for better performance
            self._sim_time_msg_template["sim_time"] = CurrentSimNanos * 1.0E-9
            self.send_socket.send(json_dumps(self._sim_time_msg_template), flags=zmq.NOBLOCK)
        except Exception as e:
            self._log_error(f"Error publishing sim_time: {e}")

    def _publish_bsk_message(self, CurrentSimNanos, msg_payload, msg_type_name):
        """Publish BSK message data to ROS2 via bridge."""
        try:
            # Find topic name
            topic_name = next((info['topic_name'] for info in self.input_msg_readers.values() 
                              if info['msg_type'] == msg_type_name), None)
            
            if topic_name is None:
                self._log_error(f"No topic_name found for message type {msg_type_name}")
                return

            # Update timestamp if message supports it
            if hasattr(msg_payload, "stamp"):
                try:
                    secs, nsecs = divmod(CurrentSimNanos, 1_000_000_000)
                    if hasattr(msg_payload.stamp, "sec"):
                        msg_payload.stamp.sec = int(secs)
                        msg_payload.stamp.nanosec = int(nsecs)
                    elif isinstance(msg_payload.stamp, dict):
                        msg_payload.stamp.update({"sec": int(secs), "nanosec": int(nsecs)})
                    elif msg_payload.stamp is None:
                        msg_payload.stamp = {"sec": int(secs), "nanosec": int(nsecs)}
                except Exception:
                    pass

            json_data = self._bsk_msg_to_json(msg_payload, msg_type_name, CurrentSimNanos)
            msg = {
                "namespace": self.namespace,
                "msg_type": msg_type_name,
                "topic_name": topic_name,
                "time": CurrentSimNanos * 1e-9,
                **json_data
            }

            self.send_socket.send(json_dumps(msg), flags=zmq.NOBLOCK)
        except Exception as e:
            self._log_error(f"Error publishing {msg_type_name}: {e}")

    def _receive_ros_message(self, CurrentSimNanos):
        """Receive and process incoming message from ROS2."""
        try:
            while True:
                command_data = json_loads(self.receive_socket.recv(flags=zmq.NOBLOCK))
                
                # Check if this message is for our namespace
                if command_data.get('namespace') != self.namespace:
                    continue
                
                # Handle topic confirmations
                if command_data.get('topic_confirmation', False):
                    request_id = command_data.get('request_id')
                    if request_id:
                        self.confirmed_topics.add(request_id)
                        self._log_info(
                            f"Received topic confirmation for "
                            f"/{self.namespace}/bsk/{command_data.get('direction', '')}/{command_data.get('topic_name', '')}"
                        )
                    continue
                    
                msg_type_name = command_data.get('msg_type')
                if not msg_type_name:
                    continue
                
                # Find writer and process message
                writer_info = next((info for info in self.output_msg_writers.values() 
                                  if info['msg_type'] == msg_type_name), None)
                if writer_info:
                    # Convert JSON to BSK message and write
                    msg_payload = self._json_to_bsk_msg(command_data, msg_type_name)
                    writer_info['writer'].write(msg_payload, CurrentSimNanos, self.moduleID)
                    
        except zmq.Again:
            pass  # No more messages
        except Exception as e:
            self._log_error(f"Error processing commands: {e}")

    # =========================================================================
    # LOGGING UTILITIES
    # =========================================================================

    def _log_info(self, message):
        """Unified info logging."""
        if hasattr(self, 'bskLogger') and self.bskLogger:
            self.bskLogger.bskLog(sysModel.BSK_INFORMATION, message)

    def _log_warning(self, message):
        """Unified warning logging."""
        if hasattr(self, 'bskLogger') and self.bskLogger:
            self.bskLogger.bskLog(sysModel.BSK_WARNING, message)

    def _log_error(self, message):
        """Unified error logging."""
        if hasattr(self, 'bskLogger') and self.bskLogger:
            self.bskLogger.bskLog(sysModel.BSK_ERROR, message)

    def _log_debug(self, message):
        """Unified debug logging."""
        if hasattr(self, 'bskLogger') and self.bskLogger:
            self.bskLogger.bskLog(sysModel.BSK_DEBUG, message)