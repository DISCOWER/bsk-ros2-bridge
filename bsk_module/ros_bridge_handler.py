from Basilisk.architecture import sysModel, messaging
from Basilisk.utilities import orbitalMotion, unitTestSupport as UAT, RigidBodyKinematics as RBK
import zmq
import json
import time
import threading
import numpy as np

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
    - Publishing spacecraft state data (position, velocity, attitude, mass properties)
    - Receiving control commands (forces and torques) from ROS2
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
        
        # Declare input messages (following Basilisk pattern)
        self.scStateInMsg = messaging.SCStatesMsgReader()
        # TODO: Add vehicle config support
        # self.vehConfigInMsg = messaging.VehicleConfigMsgReader()
        
        # Declare output messages (following Basilisk pattern)
        self.cmdForceOutMsg = messaging.CmdForceBodyMsg()
        self.cmdTorqueOutMsg = messaging.CmdTorqueBodyMsg()
        
        # State tracking variables
        self.last_force_cmd = [0.0, 0.0, 0.0]
        self.last_torque_cmd = [0.0, 0.0, 0.0]
        self.last_heartbeat_time = None
        self._running = True
        self._heartbeat_was_lost = False
        self._initialization_complete = False
        
        # Initialize ZMQ communication (done in __init__ as per Basilisk pattern)
        self._setup_zmq_communication()
        
        # Wait for bridge heartbeat before proceeding
        self._wait_for_bridge_connection()
        
        self._initialization_complete = True
        
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
        if not self.scStateInMsg.isLinked():
            self.bskLogger.bskLog(sysModel.BSK_ERROR, 
                                 "RosBridgeHandler.scStateInMsg is not connected.")
            
        # TODO: Add vehicle config validation when implemented
        # if not self.vehConfigInMsg.isLinked():
        #     self.bskLogger.bskLog(sysModel.BSK_ERROR, "RosBridgeHandler.vehConfigInMsg is not connected.")
        
        # Reset state variables
        self.last_force_cmd = [0.0, 0.0, 0.0]
        self.last_torque_cmd = [0.0, 0.0, 0.0]
        
        # Initialize output messages to zero state (Basilisk best practice)
        forceOutMsgBuffer = messaging.CmdForceBodyMsgPayload()
        forceOutMsgBuffer.forceRequestBody = [0.0, 0.0, 0.0]
        self.cmdForceOutMsg.write(forceOutMsgBuffer, CurrentSimNanos, self.moduleID)
        
        torqueOutMsgBuffer = messaging.CmdTorqueBodyMsgPayload()
        torqueOutMsgBuffer.torqueRequestBody = [0.0, 0.0, 0.0]
        self.cmdTorqueOutMsg.write(torqueOutMsgBuffer, CurrentSimNanos, self.moduleID)
        
        self.bskLogger.bskLog(sysModel.BSK_INFORMATION, 
                             f"Reset complete for RosBridgeHandler namespace: {self.namespace}")
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

        # Read input messages
        scStateInMsgBuffer = self.scStateInMsg()

        # Publish simulation time to a unique ROS topic
        self._publish_sim_time(CurrentSimNanos)

        # Publish spacecraft state to ROS2 (every UpdateState call for maximum frequency)
        self._publish_spacecraft_state(CurrentSimNanos, scStateInMsgBuffer)
        self._publish_spacecraft_velocity(CurrentSimNanos, scStateInMsgBuffer)
        # TODO: Implement mass properties publishing
        # self._publish_mass_properties(CurrentSimNanos, vehConfigMsgBuffer)
            
        # Receive and process control commands from ROS2
        force_cmd, torque_cmd = self._receive_control_commands()
        
        # Update output messages if commands changed
        self._update_force_output(force_cmd, CurrentSimNanos)
        self._update_torque_output(torque_cmd, CurrentSimNanos)
        
        # Periodic logging (once per second to avoid performance impact)
        # Following Basilisk pattern for module logging
        if CurrentSimNanos % int(1e9) == 0:
            self.bskLogger.bskLog(sysModel.BSK_INFORMATION, 
                                 f"RosBridgeHandler [{self.namespace}] Update at {CurrentSimNanos * 1.0E-9:.3f}s")
            self.bskLogger.bskLog(sysModel.BSK_DEBUG, 
                                 f"Force Command: {force_cmd}, Torque Command: {torque_cmd}")
            
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
        """Publish simulation time to a unique ROS topic for synchronization."""
        try:
            sim_time_msg = {
                "topic": "/bsk_sim_time",
                "sim_time": CurrentSimNanos * 1.0E-9  # seconds
            }
            self.send_socket.send_string(json.dumps(sim_time_msg), flags=zmq.NOBLOCK)
        except Exception as e:
            self.bskLogger.bskLog(sysModel.BSK_ERROR, f"Error publishing sim_time: {e}")

    def _publish_spacecraft_state(self, CurrentSimNanos, scStateInMsgBuffer):
        """Publish spacecraft position and attitude state."""
        try:
            # Convert MRP to quaternion (w, x, y, z format)
            quaternion = RBK.MRP2EP(scStateInMsgBuffer.sigma_BN)

            state_msg = {
                "namespace": self.namespace,
                "topic": "/state",
                "time": CurrentSimNanos * 1.0E-9,  # simulation time in seconds
                "position": scStateInMsgBuffer.r_BN_N.tolist() if hasattr(scStateInMsgBuffer.r_BN_N, 'tolist') else list(scStateInMsgBuffer.r_BN_N),
                "orientation": quaternion.tolist() if hasattr(quaternion, 'tolist') else list(quaternion),
                "frame_id": "map"
            }

            self.send_socket.send_string(json.dumps(state_msg), flags=zmq.NOBLOCK)

        except Exception as e:
            self.bskLogger.bskLog(sysModel.BSK_ERROR, f"Error publishing state: {e}")

    def _publish_spacecraft_velocity(self, CurrentSimNanos, scStateInMsgBuffer):
        """Publish spacecraft linear and angular velocity."""
        try:
            velocity_msg = {
                "namespace": self.namespace,
                "topic": "/velocity",
                "time": CurrentSimNanos * 1.0E-9,  # simulation time in seconds
                "linear": scStateInMsgBuffer.v_BN_N.tolist() if hasattr(scStateInMsgBuffer.v_BN_N, 'tolist') else list(scStateInMsgBuffer.v_BN_N),
                "angular": scStateInMsgBuffer.omega_BN_B.tolist() if hasattr(scStateInMsgBuffer.omega_BN_B, 'tolist') else list(scStateInMsgBuffer.omega_BN_B),
                "frame_id": f"{self.namespace.strip('/')}_body"
            }

            self.send_socket.send_string(json.dumps(velocity_msg), flags=zmq.NOBLOCK)

        except Exception as e:
            self.bskLogger.bskLog(sysModel.BSK_ERROR, f"Error publishing velocity: {e}")

    def _receive_control_commands(self):
        """Receive and parse control commands from ROS2."""
        try:
            ros_msg = self.receive_socket.recv_string(flags=zmq.NOBLOCK)
            command_data = json.loads(ros_msg)
            
            # Extract force and torque commands
            force_cmd = command_data.get("force", [0.0, 0.0, 0.0])
            torque_cmd = command_data.get("torque", [0.0, 0.0, 0.0])
            
            # Validate command format
            if not (isinstance(force_cmd, list) and len(force_cmd) == 3):
                raise ValueError(f"Invalid force command format: {force_cmd}")
            if not (isinstance(torque_cmd, list) and len(torque_cmd) == 3):
                raise ValueError(f"Invalid torque command format: {torque_cmd}")
                
            # Reset delay count on successful receive
            global delay_count
            delay_count = 0
            
            return force_cmd, torque_cmd
            
        except zmq.Again:
            # No new message available, return last known commands
            return self.last_force_cmd, self.last_torque_cmd
        except (json.JSONDecodeError, KeyError, ValueError) as e:
            self.bskLogger.bskLog(sysModel.BSK_WARNING, f"Error parsing control command: {e}")
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
        except Exception as e:
            self.bskLogger.bskLog(sysModel.BSK_ERROR, f"Unexpected error receiving commands: {e}")
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
            
    def _update_force_output(self, force_cmd, CurrentSimNanos):
        """Update force output message if command changed."""
        if force_cmd != self.last_force_cmd:
            forceOutMsgBuffer = messaging.CmdForceBodyMsgPayload()
            forceOutMsgBuffer.forceRequestBody = force_cmd
            self.cmdForceOutMsg.write(forceOutMsgBuffer, CurrentSimNanos, self.moduleID)
            self.last_force_cmd = force_cmd.copy() if isinstance(force_cmd, list) else force_cmd
            
    def _update_torque_output(self, torque_cmd, CurrentSimNanos):
        """Update torque output message if command changed."""
        if torque_cmd != self.last_torque_cmd:
            torqueOutMsgBuffer = messaging.CmdTorqueBodyMsgPayload()
            torqueOutMsgBuffer.torqueRequestBody = torque_cmd
            self.cmdTorqueOutMsg.write(torqueOutMsgBuffer, CurrentSimNanos, self.moduleID)
            self.last_torque_cmd = torque_cmd.copy() if isinstance(torque_cmd, list) else torque_cmd
            
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
            if hasattr(self, 'bskLogger'):
                self.bskLogger.bskLog(sysModel.BSK_WARNING, f"Error during socket cleanup: {e}")
            else:
                print(f"Warning: Error during socket cleanup: {e}")
        
        if hasattr(self, 'bskLogger'):
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
        
        if hasattr(self, 'bskLogger'):
            self.bskLogger.bskLog(sysModel.BSK_INFORMATION, "Shutdown initiated")
        else:
            print(f"[{self.namespace}] Shutdown initiated")
        
        self.shutdown()
        return True
        
    # Legacy method names for backward compatibility
    def Port_Clean_Exit(self):
        """Legacy method - use shutdown() instead."""
        print("Warning: Port_Clean_Exit() is deprecated, use shutdown() instead")
        self.shutdown()
        
    def Kill_Bridge_Send(self):
        """Legacy method - use send_kill_signal() instead."""
        print("Warning: Kill_Bridge_Send() is deprecated, use send_kill_signal() instead")
        return self.send_kill_signal()
