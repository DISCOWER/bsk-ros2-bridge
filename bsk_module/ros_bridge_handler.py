from Basilisk.architecture import sysModel, messaging
from Basilisk.utilities import RigidBodyKinematics as RBK
import zmq
import threading
import time
import json
import inspect
import importlib
import pkgutil
import numpy as np
import re

# Global variables should be avoided, but keeping for compatibility
global delay_count
delay_count = 0

# Shared ZMQ context and sockets for multi-spacecraft scenarios
_shared_context = None
_shared_sockets = {}

# See https://hanspeterschaub.info/basilisk/Learn/makingModules/pyModules.html for Python Module creation original example.
class RosBridgeHandler(sysModel.SysModel):
    """
    Basilisk-ROS2 Bridge Handler Module
    
    This class bridges Basilisk spacecraft simulation with ROS2 by handling:
    - ZMQ communication between Basilisk and ROS2 bridge
    - Automatic BSK message type discovery and conversion
    - Publishing any BSK message payload to ROS2 topics
    - Receiving any BSK message payload from ROS2 topics
    - Health monitoring via heartbeat mechanism
    - Graceful shutdown and resource cleanup
    
    Inherits from SysModel and implements the required methods:
    - Reset(): Initialize persistent data to ready state
    - UpdateState(): Cyclical worker method called at specified rate
    
    Usage:
        bridge = RosBridgeHandler(namespace="spacecraft1")
        bridge.scStateInMsg.subscribeTo(scStateMsg)
        # Add to simulation task...
    """
    _field_mapping_cache = {}  # Cache for lowercase field name mapping per message type

    def __init__(self, namespace="spacecraft1", send_port=5550, receive_port=5551, 
                 heartbeat_port=5552, timeout=15):
        """
        Initialize the ROS Bridge Handler.
        
        Args:
            namespace (str): Spacecraft namespace for topic routing
            send_port (int): ZMQ port for sending data to bridge (BSK -> ROS2)
            receive_port (int): ZMQ port for receiving data from bridge (ROS2 -> BSK)
            heartbeat_port (int): ZMQ port for heartbeat monitoring
            timeout (int): Timeout in seconds for various operations
        """
        # Call parent constructor first - this is critical for Basilisk modules
        super(RosBridgeHandler, self).__init__()

        # Configuration parameters
        self.namespace = namespace
        self.timeout = timeout
        self.send_port = send_port
        self.receive_port = receive_port
        self.heartbeat_port = heartbeat_port
        
        # Dynamic message handling
        self.input_msg_readers = {}    # BSK message readers for inputs
        self.output_msg_writers = {}   # BSK message writers for outputs
        self.last_msg_data = {}        # Cache for last received message data
        
        # Subscription handshaking
        self.pending_subscriptions = {}  # Track pending subscription requests
        self.confirmed_subscriptions = set()  # Track confirmed subscriptions
        
        # Discover available BSK message types
        self._discover_bsk_messages()
        
        # State tracking variables
        self.last_heartbeat_time = None
        self._running = True
        self._heartbeat_was_lost = False
        self._initialization_complete = False
        
        # Initialize ZMQ communication (done in __init__ as per Basilisk pattern)
        self._setup_zmq_communication()
        
        # Wait for bridge heartbeat before proceeding
        self._wait_for_bridge_connection()
        
        self._initialization_complete = True
        
    def _discover_bsk_messages(self):
        """Automatically discover all BSK message types in Basilisk.architecture.messaging."""
        self.bsk_msg_types = {}

        try:
            import Basilisk.architecture.messaging as messaging_pkg
            # Recursively walk all submodules in messaging
            for finder, modname, ispkg in list(pkgutil.walk_packages(messaging_pkg.__path__, messaging_pkg.__name__ + ".")):
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

    def add_bsk_msg_reader(self, msg_type_name, reader_name=None, topic_name=None):
        """
        Add a BSK message reader for automatic publishing to ROS2.
        
        Args:
            msg_type_name (str): BSK message type name (e.g., 'SCStatesMsgPayload')
            reader_name (str): Optional custom name for the reader
            topic_name (str): Optional custom topic name (e.g., 'imu_1', 'imu_2')
        """
        if msg_type_name not in self.bsk_msg_types:
            if hasattr(self, 'bskLogger') and self.bskLogger:
                self.bskLogger.bskLog(sysModel.BSK_ERROR, 
                                     f"Unknown BSK message type: {msg_type_name}")
            else:
                print(f"ERROR: Unknown BSK message type: {msg_type_name}")
            return None
            
        reader_name = reader_name or f"{msg_type_name.lower()}_reader"
        
        # If no custom topic name provided, use automatic conversion
        if topic_name is None:
            topic_name = self._msg_type_to_topic_name(msg_type_name)
        
        # Automatically generate reader class name: 'SCStatesMsgPayload' -> 'SCStatesMsgReader'
        try:
            reader_class_name = msg_type_name.replace('MsgPayload', 'MsgReader')
            reader_class = getattr(messaging, reader_class_name)
            reader = reader_class()
        except AttributeError:
            if hasattr(self, 'bskLogger') and self.bskLogger:
                self.bskLogger.bskLog(sysModel.BSK_ERROR, 
                                     f"No reader found for message type: {msg_type_name} (tried {reader_class_name})")
            else:
                print(f"ERROR: No reader found for message type: {msg_type_name} (tried {reader_class_name})")
            return None
        
        self.input_msg_readers[reader_name] = {
            'reader': reader,
            'msg_type': msg_type_name,
            'topic_name': topic_name
        }
        
        # Make reader accessible as module attribute
        setattr(self, reader_name, reader)
        
        if hasattr(self, 'bskLogger') and self.bskLogger:
            self.bskLogger.bskLog(sysModel.BSK_INFORMATION, 
                                 f"Added BSK message reader: {reader_name} ({msg_type_name}) -> /{self.namespace}/bsk/out/{topic_name}")
        else:
            print(f"[{self.namespace}] Added BSK message reader: {reader_name} ({msg_type_name}) -> /{self.namespace}/bsk/out/{topic_name}")
        return reader

    def add_bsk_msg_writer(self, msg_type_name, writer_name=None, topic_name=None):
        """
        Add a BSK message writer for automatic subscription from ROS2.
        
        Args:
            msg_type_name (str): BSK message type name (e.g., 'CmdForceBodyMsgPayload')
            writer_name (str): Optional custom name for the writer
            topic_name (str): Optional custom topic name (e.g., 'thruster_1', 'thruster_2')
        """
        if msg_type_name not in self.bsk_msg_types:
            if hasattr(self, 'bskLogger') and self.bskLogger:
                self.bskLogger.bskLog(sysModel.BSK_ERROR, 
                                     f"Unknown BSK message type: {msg_type_name}")
            else:
                print(f"ERROR: Unknown BSK message type: {msg_type_name}")
            return None
            
        writer_name = writer_name or f"{msg_type_name.lower()}_writer"
        
        # If no custom topic name provided, use automatic conversion
        if topic_name is None:
            topic_name = self._msg_type_to_topic_name(msg_type_name)
        
        # Automatically generate writer class name: 'CmdForceBodyMsgPayload' -> 'CmdForceBodyMsg'
        try:
            writer_class_name = msg_type_name.replace('MsgPayload', 'Msg')
            writer_class = getattr(messaging, writer_class_name)
            writer = writer_class()
        except AttributeError:
            if hasattr(self, 'bskLogger') and self.bskLogger:
                self.bskLogger.bskLog(sysModel.BSK_ERROR, 
                                     f"No writer found for message type: {msg_type_name} (tried {writer_class_name})")
            else:
                print(f"ERROR: No writer found for message type: {msg_type_name} (tried {writer_class_name})")
            return None
        
        # Store only the writer
        self.output_msg_writers[writer_name] = {
            'writer': writer,
            'msg_type': msg_type_name,
            'topic_name': topic_name
        }
        
        # Make writer accessible as module attribute  
        setattr(self, writer_name, writer)
        
        # Send subscription request to bridge
        self._send_subscription_request(msg_type_name, topic_name)
        
        if hasattr(self, 'bskLogger') and self.bskLogger:
            self.bskLogger.bskLog(sysModel.BSK_INFORMATION, 
                                 f"Added BSK message writer: {writer_name} ({msg_type_name}) <- /{self.namespace}/bsk/in/{topic_name}")
        else:
            print(f"[{self.namespace}] Added BSK message writer: {writer_name} ({msg_type_name}) <- /{self.namespace}/bsk/in/{topic_name}")
        return writer

    def _send_subscription_request(self, msg_type_name, topic_name):
        """Send subscription request to bridge for automatic subscriber creation."""
        try:
            request_id = f"{self.namespace}_{msg_type_name}_{topic_name}"
            
            subscription_request = {
                "subscription_request": True,
                "namespace": self.namespace,
                "msg_type": msg_type_name,
                "topic_name": topic_name,
                "request_id": request_id
            }
            
            # Track pending request
            self.pending_subscriptions[request_id] = {
                "msg_type": msg_type_name,
                "topic_name": topic_name,
                "attempts": 0,
                "last_attempt": time.time()
            }
            
            self.send_socket.send_string(json.dumps(subscription_request), flags=zmq.NOBLOCK)
            
            if hasattr(self, 'bskLogger') and self.bskLogger:
                self.bskLogger.bskLog(sysModel.BSK_INFORMATION, 
                                     f"Sent subscription request for {msg_type_name} -> /{self.namespace}/bsk/in/{topic_name} (ID: {request_id})")
            else:
                print(f"[{self.namespace}] Sent subscription request for {msg_type_name} -> /{self.namespace}/bsk/in/{topic_name} (ID: {request_id})")
                
        except Exception as e:
            if hasattr(self, 'bskLogger') and self.bskLogger:
                self.bskLogger.bskLog(sysModel.BSK_ERROR, 
                                     f"Error sending subscription request: {e}")
            else:
                print(f"ERROR: Error sending subscription request: {e}")

    def _check_pending_subscriptions(self):
        """Check for pending subscription requests and retry if needed."""
        current_time = time.time()
        max_attempts = 5
        retry_interval = 2.0  # seconds
        
        for request_id, request_info in list(self.pending_subscriptions.items()):
            if request_id in self.confirmed_subscriptions:
                # Remove confirmed subscriptions from pending
                del self.pending_subscriptions[request_id]
                continue
                
            time_since_last = current_time - request_info["last_attempt"]
            
            if time_since_last > retry_interval:
                if request_info["attempts"] < max_attempts:
                    # Retry the subscription request
                    request_info["attempts"] += 1
                    request_info["last_attempt"] = current_time
                    
                    subscription_request = {
                        "subscription_request": True,
                        "namespace": self.namespace,
                        "msg_type": request_info["msg_type"],
                        "topic_name": request_info["topic_name"],
                        "request_id": request_id
                    }
                    
                    try:
                        self.send_socket.send_string(json.dumps(subscription_request), flags=zmq.NOBLOCK)
                        print(f"[{self.namespace}] Retrying subscription request {request_id} (attempt {request_info['attempts']}/{max_attempts})")
                    except Exception as e:
                        print(f"ERROR: Failed to retry subscription request {request_id}: {e}")
                else:
                    # Max attempts reached
                    print(f"[{self.namespace}] WARNING: Subscription request {request_id} failed after {max_attempts} attempts")
                    del self.pending_subscriptions[request_id]

    def _msg_type_to_topic_name(self, msg_type_name):
        """
        Convert BSK message type name to snake_case topic name.
        E.g. 'SCStatesMsgPayload' -> 'sc_states'
        E.g. 'THRArrayCmdForceMsgPayload' -> 'thr_array_cmd_force'
        """
        # Remove 'MsgPayload' suffix if present
        if msg_type_name.endswith('MsgPayload'):
            name = msg_type_name[:-10]  # Remove 'MsgPayload'
        else:
            name = msg_type_name
        
        # Convert CamelCase to snake_case with proper handling of consecutive uppercase
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

    def _bsk_msg_to_json(self, msg_payload, msg_type_name, CurrentSimNanos=None):
        """Convert BSK message payload to JSON dict. Optionally add ROS2 timestamp."""
        json_data = {}
        
        # SWIG attributes to skip 
        swig_attrs_to_skip = {'this', 'thisown'}
        
        # Get all attributes from the message payload, excluding SWIG internals
        for attr_name in dir(msg_payload):
            if (not attr_name.startswith('_') and 
                attr_name not in swig_attrs_to_skip and
                not callable(getattr(msg_payload, attr_name, None))):
                try:
                    attr_value = getattr(msg_payload, attr_name)
                    ros2_field_name = attr_name.lower()
                    
                    # Convert different types to JSON-serializable format
                    if isinstance(attr_value, (list, tuple)):
                        json_data[ros2_field_name] = list(attr_value)
                    elif hasattr(attr_value, 'tolist'):
                        json_data[ros2_field_name] = attr_value.tolist()
                    elif isinstance(attr_value, (int, float, str, bool)):
                        json_data[ros2_field_name] = attr_value
                    elif attr_value is None:
                        json_data[ros2_field_name] = None
                    else:
                        continue
                except Exception:
                    continue

        # Add ROS2 timestamp if requested
        if CurrentSimNanos is not None:
            secs = int(CurrentSimNanos // 1_000_000_000)
            nsecs = int(CurrentSimNanos % 1_000_000_000)
            json_data['stamp'] = {'sec': secs, 'nanosec': nsecs}

        return json_data

    def normalize_field_name(self, field_name):
        """Normalize field names to lowercase, underscore-agnostic."""
        return re.sub(r'_', '', field_name).lower()

    def _json_to_bsk_msg(self, json_data, msg_type_name):
        """Robustly convert JSON dict to BSK message payload."""
        msg_payload_class = self.bsk_msg_types[msg_type_name]
        msg_payload = msg_payload_class()

        # Generate field mapping once per message type
        if msg_type_name not in self._field_mapping_cache:
            bsk_field_mapping = {}
            for attr_name in dir(msg_payload):
                if not attr_name.startswith('_') and not callable(getattr(msg_payload, attr_name, None)):
                    normalized_name = self.normalize_field_name(attr_name)
                    bsk_field_mapping[normalized_name] = attr_name
            self._field_mapping_cache[msg_type_name] = bsk_field_mapping
        else:
            bsk_field_mapping = self._field_mapping_cache[msg_type_name]

        for json_field_name, attr_value in json_data.items():
            if json_field_name in ['namespace', 'msg_type', 'topic_name', 'time', 'timestamp', 'stamp']:
                continue

            normalized_json_name = self.normalize_field_name(json_field_name)
            bsk_field_name = bsk_field_mapping.get(normalized_json_name)

            if bsk_field_name and hasattr(msg_payload, bsk_field_name):
                try:
                    current_value = getattr(msg_payload, bsk_field_name)

                    if isinstance(current_value, np.ndarray):
                        attr_array = np.array(attr_value, dtype=current_value.dtype)
                        attr_array.shape = current_value.shape
                        setattr(msg_payload, bsk_field_name, attr_array)
                    else:
                        setattr(msg_payload, bsk_field_name, attr_value)

                except Exception as e:
                    print(f"WARNING: Failed to set {bsk_field_name}: {e}")

        return msg_payload


    def _setup_zmq_communication(self):
        """Setup all ZMQ sockets and communication channels."""
        global _shared_context, _shared_sockets
        
        # Use shared context for multi-spacecraft scenarios
        if _shared_context is None:
            _shared_context = zmq.Context()
        self.context = _shared_context
        
        # Setup heartbeat monitoring
        self._setup_heartbeat_monitoring()
        
        # Setup data publisher (BSK -> ROS2) - use shared socket
        self._setup_data_publisher()
        
        # Setup command subscriber (ROS2 -> BSK)
        self._setup_command_subscriber()
        
    def _setup_heartbeat_monitoring(self):
        """Setup heartbeat monitoring from bridge."""
        self.heartbeat_sub = self.context.socket(zmq.SUB)
        self.heartbeat_sub.connect(f"tcp://localhost:{self.heartbeat_port}")
        self.heartbeat_sub.setsockopt_string(zmq.SUBSCRIBE, "")
        self.heartbeat_sub.setsockopt(zmq.RCVTIMEO, 10)  # 10ms timeout

        def heartbeat_monitor():
            """Background thread to monitor bridge heartbeat."""
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
        """Setup ZMQ publisher for sending data to bridge."""
        global _shared_sockets
        
        # Use shared socket for all spacecraft to avoid port conflicts
        socket_key = f"send_{self.send_port}"
        
        if socket_key not in _shared_sockets:
            # First handler creates and binds the socket
            self.send_socket = self.context.socket(zmq.PUB)
            self.send_socket.setsockopt(zmq.LINGER, 0)
            self.send_socket.setsockopt(zmq.SNDHWM, 100)  # High water mark
            
            try:
                self.send_socket.bind(f"tcp://*:{self.send_port}")
                _shared_sockets[socket_key] = self.send_socket
                print(f"[{self.namespace}] Created and bound shared publisher on port {self.send_port}")
            except zmq.ZMQError as e:
                error_msg = f"Failed to bind to port {self.send_port}: {e}"
                print(f"ERROR: {error_msg}")
                self._init_error = error_msg
                raise RuntimeError(error_msg)
        else:
            # Subsequent handlers use the existing socket
            self.send_socket = _shared_sockets[socket_key]
            print(f"[{self.namespace}] Using shared publisher on port {self.send_port}")
            
    def _setup_command_subscriber(self):
        """Setup ZMQ subscriber for receiving commands from bridge."""
        self.receive_socket = self.context.socket(zmq.SUB)
        self.receive_socket.connect(f"tcp://localhost:{self.receive_port}")
        self.receive_socket.setsockopt_string(zmq.SUBSCRIBE, '')
        self.receive_socket.setsockopt(zmq.CONFLATE, 1)  # Keep only latest message
        self.receive_socket.setsockopt(zmq.RCVTIMEO, 1)   # 1ms timeout
        
    def _wait_for_bridge_connection(self):
        """Wait for the first heartbeat before starting operations."""
        print(f"[{self.namespace}] Waiting for bridge heartbeat...")
        start_time = time.time()
        
        while self.last_heartbeat_time is None:
            if time.time() - start_time > self.timeout:
                error_msg = f"Timeout waiting for bridge heartbeat after {self.timeout}s"
                print(f"ERROR: {error_msg}")
                raise TimeoutError(error_msg)
            time.sleep(0.01)
            
        print(f"[{self.namespace}] Bridge heartbeat detected. Module ready.")
        
    def Reset(self, CurrentSimNanos):
        """
        Reset method called to initialize persistent data to ready state.
        
        This method is called once after selfInit/crossInit, but should be
        written to allow multiple calls if necessary.
        
        Args:
            CurrentSimNanos (int): Current simulation time in nanoseconds
        """
        # Validate message connections - this follows Basilisk best practices
        for reader_name, reader_info in self.input_msg_readers.items():
            if not reader_info['reader'].isLinked():
                if hasattr(self, 'bskLogger') and self.bskLogger:
                    self.bskLogger.bskLog(sysModel.BSK_WARNING, 
                                         f"Reader {reader_name} is not connected.")
                else:
                    print(f"WARNING: Reader {reader_name} is not connected.")
        
        # Don't initialize output messages to zero - they should receive data from ROS2
        # Only validate that they exist
        for writer_name, writer_info in self.output_msg_writers.items():
            if hasattr(self, 'bskLogger') and self.bskLogger:
                self.bskLogger.bskLog(sysModel.BSK_DEBUG, 
                                     f"Output writer {writer_name} ready to receive commands from ROS2")
            
        if hasattr(self, 'bskLogger') and self.bskLogger:
            self.bskLogger.bskLog(sysModel.BSK_INFORMATION, 
                                 f"Reset complete for RosBridgeHandler namespace: {self.namespace}")
        else:
            print(f"[{self.namespace}] Reset complete for RosBridgeHandler")
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

        # Check and retry pending subscriptions
        self._check_pending_subscriptions()

        # Publish sim time (special topic, always published)
        self._publish_sim_time(CurrentSimNanos)

        # Process all input message readers and publish to ROS2
        for reader_name, reader_info in self.input_msg_readers.items():
            if reader_info['reader'].isLinked() and reader_info['reader'].isWritten():
                msg_payload = reader_info['reader']()
                self._publish_bsk_message(CurrentSimNanos, msg_payload, reader_info['msg_type'])
                if reader_info['msg_type'] == 'CmdForceBodyMsgPayload':
                    print(f"[{self.namespace}] <<< Echo reader sees: {msg_payload.forceRequestBody}")

        # Receive and process commands from ROS2
        self._receive_and_process_commands(CurrentSimNanos)

    def _check_bridge_health(self):
        """Check if bridge heartbeat is healthy."""
        # Use simulation time for logging, but keep wall time for heartbeat timeout
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
        """Publish simulation time to ROS2."""
        try:
            sim_time_msg = {
                "sim_time": CurrentSimNanos * 1.0E-9  # seconds
            }
            self.send_socket.send_string(json.dumps(sim_time_msg), flags=zmq.NOBLOCK)
        except Exception as e:
            if hasattr(self, 'bskLogger') and self.bskLogger:
                self.bskLogger.bskLog(sysModel.BSK_ERROR, f"Error publishing sim_time: {e}")
            else:
                print(f"ERROR: Error publishing sim_time: {e}")

    def _publish_bsk_message(self, CurrentSimNanos, msg_payload, msg_type_name):
        """Publish any BSK message to ROS2."""
        try:
            # Find the topic name for this message type
            topic_name = None
            for reader_name, reader_info in self.input_msg_readers.items():
                if reader_info['msg_type'] == msg_type_name:
                    topic_name = reader_info['topic_name']
                    break

            if topic_name is None:
                topic_name = self._msg_type_to_topic_name(msg_type_name)

            # Convert BSK message to JSON
            secs = int(CurrentSimNanos // 1_000_000_000)
            nsecs = int(CurrentSimNanos % 1_000_000_000)
            if hasattr(msg_payload, "stamp"):
                try:
                    # If stamp is a message object with sec/nanosec fields
                    if hasattr(msg_payload.stamp, "sec") and hasattr(msg_payload.stamp, "nanosec"):
                        msg_payload.stamp.sec = secs
                        msg_payload.stamp.nanosec = nsecs
                    # If stamp is a dict or similar
                    elif isinstance(msg_payload.stamp, dict):
                        msg_payload.stamp["sec"] = secs
                        msg_payload.stamp["nanosec"] = nsecs
                    # If stamp is None or uninitialized, try to set as dict
                    elif msg_payload.stamp is None:
                        msg_payload.stamp = {"sec": secs, "nanosec": nsecs}
                except Exception:
                    pass

            # Convert BSK message to JSON (will include the updated stamp)
            json_data = self._bsk_msg_to_json(msg_payload, msg_type_name, CurrentSimNanos)

            # Add metadata for the bridge
            msg = {
                "namespace": self.namespace,
                "msg_type": msg_type_name,
                "topic_name": topic_name,
                "time": float(CurrentSimNanos) * 1e-9,
                **json_data
            }

            self.send_socket.send_string(json.dumps(msg), flags=zmq.NOBLOCK)

        except Exception as e:
            if hasattr(self, 'bskLogger') and self.bskLogger:
                self.bskLogger.bskLog(sysModel.BSK_ERROR,
                                     f"Error publishing {msg_type_name}: {e}")
            else:
                print(f"ERROR: Error publishing {msg_type_name}: {e}")

    def _receive_and_process_commands(self, CurrentSimNanos):
        """Receive and process any BSK message commands from ROS2."""
        try:
            while True:  # Process all available messages
                ros_msg = self.receive_socket.recv_string(flags=zmq.NOBLOCK)
                command_data = json.loads(ros_msg)
                
                # Check if this message is for our namespace
                if command_data.get('namespace') != self.namespace:
                    continue
                
                # Handle subscription confirmations
                if command_data.get('subscription_confirmation', False):
                    request_id = command_data.get('request_id')
                    if request_id:
                        self.confirmed_subscriptions.add(request_id)
                        if hasattr(self, 'bskLogger') and self.bskLogger:
                            self.bskLogger.bskLog(sysModel.BSK_INFORMATION, 
                                                 f"Received subscription confirmation for {request_id}")
                        else:
                            print(f"[{self.namespace}] Received subscription confirmation for {request_id}")
                    continue
                    
                msg_type_name = command_data.get('msg_type')
                if not msg_type_name:
                    continue
                
                # Find appropriate writer for this message type
                writer_info = None
                for writer_name, info in self.output_msg_writers.items():
                    if info['msg_type'] == msg_type_name:
                        writer_info = info
                        break
                
                if writer_info:
                    # if msg_type_name == 'CmdForceBodyMsgPayload':
                    #     command_data['forceRequestBody'] = command_data.get('force_request_body', [0, 0, 0])
                    # Convert JSON to BSK message and write
                    msg_payload = self._json_to_bsk_msg(command_data, msg_type_name)
                    writer_info['writer'].write(msg_payload, CurrentSimNanos, self.moduleID)
                    
                    # Cache the data
                    self.last_msg_data[msg_type_name] = command_data
                    
                    # Debug logging for force and torque commands
                    if msg_type_name == 'CmdForceBodyMsgPayload':
                        force_data = command_data.get('force_request_body', [0, 0, 0])
                        print(f"[{self.namespace}] Received force command: [{force_data[0]:.3f}, {force_data[1]:.3f}, {force_data[2]:.3f}] N")
                    elif msg_type_name == 'CmdTorqueBodyMsgPayload':
                        torque_data = command_data.get('torque_request_body', [0, 0, 0])
                        print(f"[{self.namespace}] Received torque command: [{torque_data[0]:.3f}, {torque_data[1]:.3f}, {torque_data[2]:.3f}] Nm")
                    
        except zmq.Again:
            # No more messages available
            pass
        except Exception as e:
            if hasattr(self, 'bskLogger') and self.bskLogger:
                self.bskLogger.bskLog(sysModel.BSK_ERROR, 
                                     f"Error processing commands: {e}")
            else:
                print(f"ERROR: Error processing commands: {e}")

    def shutdown(self):
        """
        Graceful shutdown of the bridge handler.
        
        Stops all background threads, closes ZMQ sockets, and cleans up resources.
        """
        if not hasattr(self, '_running') or not self._running:
            return  # Already shut down
            
        self._running = False
        
        # Wait for heartbeat thread to finish
        if hasattr(self, 'heartbeat_thread') and self.heartbeat_thread.is_alive():
            self.heartbeat_thread.join(timeout=1.0)
        
        # Close all sockets
        try:
            if hasattr(self, 'receive_socket'):
                self.receive_socket.close()
            if hasattr(self, 'heartbeat_sub'):
                self.heartbeat_sub.close()
        except Exception as e:
            if hasattr(self, 'bskLogger') and self.bskLogger:
                self.bskLogger.bskLog(sysModel.BSK_WARNING, f"Error during socket cleanup: {e}")
            else:
                print(f"Warning: Error during socket cleanup: {e}")
        
        if hasattr(self, 'bskLogger') and self.bskLogger:
            self.bskLogger.bskLog(sysModel.BSK_INFORMATION, 
                                 f"RosBridgeHandler [{self.namespace}] shutdown complete")
        else:
            print(f"[{self.namespace}] RosBridgeHandler shutdown complete")

    def send_kill_signal(self):
        """
        Shutdown the bridge handler.
        
        Returns:
            bool: True indicating successful shutdown
        """
        global delay_count
        print(f'[{self.namespace}] Delayed count for actuation messages: {delay_count}')
        
        if hasattr(self, 'bskLogger') and self.bskLogger:
            self.bskLogger.bskLog(sysModel.BSK_INFORMATION, "Shutdown initiated")
        else:
            print(f"[{self.namespace}] Shutdown initiated")
        
        self.shutdown()
        return True
