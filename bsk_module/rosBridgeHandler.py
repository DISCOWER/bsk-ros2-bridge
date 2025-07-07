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
# Shared ZMQ resources for multi-spacecraft scenarios - avoids port conflicts
_shared_context = None
_shared_sockets = {}

# Pre-compiled regex for better performance
CAMEL_TO_SNAKE_RE = re.compile(r'(?<!^)(?=[A-Z])')
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
    _topic_name_cache = {}
    _bsk_attrs_cache = {}

    def __init__(self, namespace="spacecraft1", send_port=5550, receive_port=5551, 
                 heartbeat_port=5552, timeout=15):
        """
        Initialize ROS Bridge Handler.

        Args:
            namespace (str): Spacecraft namespace for topic routing
            send_port (int): ZMQ port for sending data to bridge (BSK -> ROS2)
            receive_port (int): ZMQ port for receiving data from bridge (ROS2 -> BSK)
            heartbeat_port (int): ZMQ port for heartbeat monitoring
            timeout (int): Timeout in seconds for various operations
        """
        super(RosBridgeHandler, self).__init__()

        # Configuration
        self.namespace = namespace
        self.timeout = timeout
        self.send_port = send_port
        self.receive_port = receive_port
        self.heartbeat_port = heartbeat_port
        
        # Dynamic message management
        self.input_msg_readers = {}    # BSK -> ROS2 publishers
        self.output_msg_writers = {}   # ROS2 -> BSK subscribers
        self.last_msg_data = {}        # Cache for debugging/monitoring
        
        # Subscription handshaking - ensures ROS2 subscribers are ready
        self.pending_subscriptions = {}
        self.confirmed_subscriptions = set()
        
        # Pre-allocate commonly used objects to reduce allocations
        self._sim_time_msg_template = {"sim_time": 0.0}
        # Pre-serialize for better performance
        self._subscription_request_template = {
            "subscription_request": True,
            "namespace": self.namespace,
            "msg_type": "",
            "topic_name": "",
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

    def _setup_zmq_communication(self):
        """
        Initialize ZMQ sockets with shared context for multi-spacecraft support.
        """
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
        """
        Setup shared ZMQ publisher for BSK -> ROS2 data flow.
        """
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
                error_msg = f"Failed to bind to port {self.send_port}: {e}"
                print(f"ERROR: {error_msg}")
                self._init_error = error_msg
                raise RuntimeError(error_msg)
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
        start_time = time.time()
        
        while self.last_heartbeat_time is None:
            if time.time() - start_time > self.timeout:
                error_msg = f"Timeout waiting for bridge heartbeat after {self.timeout}s"
                print(f"ERROR: {error_msg}")
                raise TimeoutError(error_msg)
            time.sleep(0.01)
            
        print(f"[{self.namespace}] Bridge heartbeat detected. Module ready.")

    # =========================================================================
    # DYNAMIC MESSAGE HANDLING
    # =========================================================================

    def add_bsk_msg_reader(self, msg_type_name, reader_name=None, topic_name=None):
        """
        Add BSK message reader for automatic publishing to ROS2.
        
        Args:
            msg_type_name (str): BSK message type name (e.g., 'SCStatesMsgPayload')
            reader_name (str): Optional custom name for the reader
            topic_name (str): Optional custom topic name (e.g., 'imu_1', 'imu_2')
        """
        if msg_type_name not in self.bsk_msg_types:
            self._log_error(f"Unknown BSK message type: {msg_type_name}")
            return None
            
        reader_name = reader_name or f"{msg_type_name.lower()}_reader"
        topic_name = topic_name or self._msg_type_to_topic_name(msg_type_name)
        
        try:
            reader_class_name = msg_type_name.replace('MsgPayload', 'MsgReader')
            reader_class = getattr(messaging, reader_class_name)
            reader = reader_class()
        except AttributeError:
            self._log_error(f"No reader found for message type: {msg_type_name} (tried {reader_class_name})")
            return None
        
        self.input_msg_readers[reader_name] = {
            'reader': reader,
            'msg_type': msg_type_name,
            'topic_name': topic_name
        }
        
        setattr(self, reader_name, reader)
        
        self._log_info(f"Added BSK message reader: {reader_name} ({msg_type_name}) -> /{self.namespace}/bsk/out/{topic_name}")
        return reader

    def add_bsk_msg_writer(self, msg_type_name, writer_name=None, topic_name=None):
        """
        Add BSK message writer for automatic subscription from ROS2.
        
        Args:
            msg_type_name (str): BSK message type name (e.g., 'CmdForceBodyMsgPayload')
            writer_name (str): Optional custom name for the writer
            topic_name (str): Optional custom topic name (e.g., 'thruster_1', 'thruster_2')
        """
        if msg_type_name not in self.bsk_msg_types:
            self._log_error(f"Unknown BSK message type: {msg_type_name}")
            return None
            
        writer_name = writer_name or f"{msg_type_name.lower()}_writer"
        topic_name = topic_name or self._msg_type_to_topic_name(msg_type_name)
        
        try:
            writer_class_name = msg_type_name.replace('MsgPayload', 'Msg')
            writer_class = getattr(messaging, writer_class_name)
            writer = writer_class()
        except AttributeError:
            self._log_error(f"No writer found for message type: {msg_type_name} (tried {writer_class_name})")
            return None
        
        self.output_msg_writers[writer_name] = {
            'writer': writer,
            'msg_type': msg_type_name,
            'topic_name': topic_name
        }
        
        setattr(self, writer_name, writer)
        self._send_subscription_request(msg_type_name, topic_name)
        
        self._log_info(f"Added BSK message writer: {writer_name} ({msg_type_name}) <- /{self.namespace}/bsk/in/{topic_name}")
        return writer

    # =========================================================================
    # SUBSCRIPTION MANAGEMENT
    # =========================================================================

    def _send_subscription_request(self, msg_type_name, topic_name):
        """
        Send subscription request to bridge for automatic ROS2 subscriber creation.
        """
        try:
            request_id = f"{self.namespace}_{msg_type_name}_{topic_name}"
            
            self._subscription_request_template.update({
                "msg_type": msg_type_name,
                "topic_name": topic_name,
                "request_id": request_id
            })
            
            current_time = time.time()
            self.pending_subscriptions[request_id] = {
                "msg_type": msg_type_name,
                "topic_name": topic_name,
                "attempts": 0,
                "last_attempt": current_time
            }
            
            self.send_socket.send(json_dumps(self._subscription_request_template), flags=zmq.NOBLOCK)
            
            self._log_info(f"Sent subscription request for {msg_type_name} -> /{self.namespace}/bsk/in/{topic_name} (ID: {request_id})")
                
        except Exception as e:
            self._log_error(f"Error sending subscription request: {e}")

    def _check_pending_subscriptions(self):
        """Check for pending subscription requests and retry if needed."""
        current_time = time.time()
        max_attempts = 10
        retry_interval = 2.0
        
        for request_id in list(self.pending_subscriptions.keys()):
            if request_id in self.confirmed_subscriptions:
                # Remove confirmed subscriptions from pending
                del self.pending_subscriptions[request_id]
                continue
            
            request_info = self.pending_subscriptions[request_id]
            time_since_last = current_time - request_info["last_attempt"]
            
            if time_since_last > retry_interval:
                if request_info["attempts"] < max_attempts:
                    # Retry the subscription request
                    request_info["attempts"] += 1
                    request_info["last_attempt"] = current_time
                    
                    self._subscription_request_template.update({
                        "msg_type": request_info["msg_type"],
                        "topic_name": request_info["topic_name"],
                        "request_id": request_id
                    })
                    
                    try:
                        self.send_socket.send(json_dumps(self._subscription_request_template), flags=zmq.NOBLOCK)
                        if request_info['attempts'] > 1:
                            # Log retries only after the first attempt
                            print(f"[{self.namespace}] Retrying subscription request {request_id} (attempt {request_info['attempts']}/{max_attempts})")
                    except Exception as e:
                        print(f"ERROR: Failed to retry subscription request {request_id}: {e}")
                else:
                    # Max attempts reached
                    print(f"[{self.namespace}] WARNING: Subscription request {request_id} failed after {max_attempts} attempts")
                    del self.pending_subscriptions[request_id]

    # =========================================================================
    # MESSAGE CONVERSION - Optimized with caching
    # =========================================================================

    def _msg_type_to_topic_name(self, msg_type_name):
        """
        Convert BSK message type name to snake_case topic name.
        E.g. 'SCStatesMsgPayload' -> 'sc_states'
        E.g. 'THRArrayCmdForceMsgPayload' -> 'thr_array_cmd_force'
        """
        if msg_type_name in self._topic_name_cache:
            return self._topic_name_cache[msg_type_name]
            
        # Remove 'MsgPayload' suffix if present
        name = msg_type_name[:-10] if msg_type_name.endswith('MsgPayload') else msg_type_name
        topic_name = CAMEL_TO_SNAKE_RE.sub('_', name).lower()
        
        self._topic_name_cache[msg_type_name] = topic_name
        return topic_name

    def _bsk_msg_to_json(self, msg_payload, msg_type_name, CurrentSimNanos=None):
        """Convert BSK message payload to JSON dict with caching for better performance."""
        # Use cached attributes for this message type
        if msg_type_name not in self._bsk_attrs_cache:
            # Pre-compute valid attributes once per message type
            swig_attrs_to_skip = {'this', 'thisown'}
            valid_attrs = []
            
            for attr_name in dir(msg_payload):
                if (not attr_name.startswith('_') and 
                    attr_name not in swig_attrs_to_skip and
                    not callable(getattr(msg_payload, attr_name, None))):
                    valid_attrs.append(attr_name)
            
            self._bsk_attrs_cache[msg_type_name] = valid_attrs
        
        valid_attrs = self._bsk_attrs_cache[msg_type_name]
        json_data = {}
        
        # Process attributes efficiently
        for attr_name in valid_attrs:
            try:
                attr_value = getattr(msg_payload, attr_name)
                ros2_field_name = attr_name.lower()
                
                if isinstance(attr_value, (int, float, str, bool)):
                    json_data[ros2_field_name] = attr_value
                elif attr_value is None:
                    json_data[ros2_field_name] = None
                elif hasattr(attr_value, 'tolist'):
                    json_data[ros2_field_name] = attr_value.tolist()
                elif isinstance(attr_value, (list, tuple)):
                    json_data[ros2_field_name] = list(attr_value)
            except Exception:
                continue

        # Add ROS2 timestamp if requested
        if CurrentSimNanos is not None:
            secs = int(CurrentSimNanos // 1_000_000_000)
            nsecs = int(CurrentSimNanos % 1_000_000_000)
            json_data['stamp'] = {'sec': secs, 'nanosec': nsecs}

        return json_data

    def normalize_field_name(self, field_name):
        """Normalize field names for case-insensitive matching."""
        return NORMALIZE_FIELD_RE.sub('', field_name).lower()

    def _json_to_bsk_msg(self, json_data, msg_type_name):
        """
        Convert JSON to BSK message with robust field matching.
        """
        msg_payload_class = self.bsk_msg_types[msg_type_name]
        msg_payload = msg_payload_class()

        if msg_type_name not in self._field_mapping_cache:
            bsk_field_mapping = {}
            swig_attrs_to_skip = {'this', 'thisown'}
            
            for attr_name in dir(msg_payload):
                if (not attr_name.startswith('_') and 
                    attr_name not in swig_attrs_to_skip and
                    not callable(getattr(msg_payload, attr_name, None))):
                    normalized_name = self.normalize_field_name(attr_name)
                    bsk_field_mapping[normalized_name] = attr_name
            
            self._field_mapping_cache[msg_type_name] = bsk_field_mapping
        
        bsk_field_mapping = self._field_mapping_cache[msg_type_name]

        # Skip metadata fields efficiently
        metadata_fields = {'namespace', 'msg_type', 'topic_name', 'time', 'timestamp', 'stamp'}
        
        for json_field_name, attr_value in json_data.items():
            if json_field_name in metadata_fields:
                continue

            normalized_json_name = self.normalize_field_name(json_field_name)
            bsk_field_name = bsk_field_mapping.get(normalized_json_name)

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
    # BASILISK MODULE INTERFACE - Required by SysModel
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
        
        for writer_name, writer_info in self.output_msg_writers.items():
            self._log_debug(f"Output writer {writer_name} ready to receive commands from ROS2")
            
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

        # Check and retry pending subscriptions
        self._check_pending_subscriptions()

        # Publish sim time (special topic, always published)
        self._publish_sim_time(CurrentSimNanos)

        # Process all input message readers and publish to ROS2
        for reader_name, reader_info in self.input_msg_readers.items():
            if reader_info['reader'].isLinked() and reader_info['reader'].isWritten():
                msg_payload = reader_info['reader']()
                self._publish_bsk_message(CurrentSimNanos, msg_payload, reader_info['msg_type'])

        # Receive and process commands from ROS2
        self._receive_and_process_commands(CurrentSimNanos)

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
            # Find the topic name for this message type
            topic_name = None
            for reader_info in self.input_msg_readers.values():
                if reader_info['msg_type'] == msg_type_name:
                    topic_name = reader_info['topic_name']
                    break

            if topic_name is None:
                topic_name = self._msg_type_to_topic_name(msg_type_name)

            # Update timestamp if message supports it
            if hasattr(msg_payload, "stamp"):
                secs = int(CurrentSimNanos // 1_000_000_000)
                nsecs = int(CurrentSimNanos % 1_000_000_000)
                try:
                    if hasattr(msg_payload.stamp, "sec") and hasattr(msg_payload.stamp, "nanosec"):
                        msg_payload.stamp.sec = secs
                        msg_payload.stamp.nanosec = nsecs
                    elif isinstance(msg_payload.stamp, dict):
                        msg_payload.stamp["sec"] = secs
                        msg_payload.stamp["nanosec"] = nsecs
                    elif msg_payload.stamp is None:
                        msg_payload.stamp = {"sec": secs, "nanosec": nsecs}
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

    def _receive_and_process_commands(self, CurrentSimNanos):
        """Process incoming commands from ROS2."""
        try:
            while True:
                ros_msg_bytes = self.receive_socket.recv(flags=zmq.NOBLOCK)
                command_data = json_loads(ros_msg_bytes)
                
                # Check if this message is for our namespace
                if command_data.get('namespace') != self.namespace:
                    continue
                
                # Handle subscription confirmations
                if command_data.get('subscription_confirmation', False):
                    request_id = command_data.get('request_id')
                    if request_id:
                        self.confirmed_subscriptions.add(request_id)
                        self._log_info(f"Received subscription confirmation for {request_id}")
                    continue
                    
                msg_type_name = command_data.get('msg_type')
                if not msg_type_name:
                    continue
                
                # Find appropriate writer for this message type
                writer_info = None
                for info in self.output_msg_writers.values():
                    if info['msg_type'] == msg_type_name:
                        writer_info = info
                        break
                
                if writer_info:
                    # Convert JSON to BSK message and write
                    msg_payload = self._json_to_bsk_msg(command_data, msg_type_name)
                    writer_info['writer'].write(msg_payload, CurrentSimNanos, self.moduleID)
                    self.last_msg_data[msg_type_name] = command_data
                    
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
        else:
            print(f"[{self.namespace}] {message}")

    def _log_warning(self, message):
        """Unified warning logging."""
        if hasattr(self, 'bskLogger') and self.bskLogger:
            self.bskLogger.bskLog(sysModel.BSK_WARNING, message)
        else:
            print(f"[{self.namespace}] WARNING: {message}")

    def _log_error(self, message):
        """Unified error logging."""
        if hasattr(self, 'bskLogger') and self.bskLogger:
            self.bskLogger.bskLog(sysModel.BSK_ERROR, message)
        else:
            print(f"[{self.namespace}] ERROR: {message}")

    def _log_debug(self, message):
        """Unified debug logging."""
        if hasattr(self, 'bskLogger') and self.bskLogger:
            self.bskLogger.bskLog(sysModel.BSK_DEBUG, message)
        else:
            print(f"[{self.namespace}] DEBUG: {message}")