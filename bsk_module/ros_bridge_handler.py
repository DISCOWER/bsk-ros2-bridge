from Basilisk.architecture import sysModel, messaging
from Basilisk.utilities import orbitalMotion, unitTestSupport as UAT, RigidBodyKinematics as RBK # Import for Hill-frame conversion & control if necessary

import zmq, json, time
import threading
global delay_count
delay_count = 0

# See https://hanspeterschaub.info/basilisk/Learn/makingModules/pyModules.html for Python Module creation original example.
class RosBridgeHandler(sysModel.SysModel):
    """
    This class inherits from the `SysModel` available in the ``Basilisk.architecture.sysModel`` module.
    The `SysModel` is the parent class which your Python BSK modules must inherit.
    The class uses the following
    virtual functions:

    #. ``Reset``: The method that will initialize any persistent data in your model to a common
       "ready to run" state (e.g. filter states, integral control sums, etc).
    #. ``UpdateState``: The method that will be called at the rate specified
       in the PythonTask that was created in the input file.

    Additionally, your class should ensure that in the ``__init__`` method, your call the super
    ``__init__`` method for the class so that the base class' constructor also gets called:

    .. code-block:: python

        super(RosBridgeHandler, self).__init__()

    You class must implement the above four functions. Beyond these four functions you class
    can complete any other computations you need (``Numpy``, ``matplotlib``, vision processing
    AI, whatever).
    """
    def __init__(self, namespace="spacecraft1", kill_request_port = 9999, send_port = 5555, receive_port = 7070, heartbeat_port=9997, timeout = 15):
        super(RosBridgeHandler, self).__init__()

        # Spacecraft namespace for topic routing
        self.namespace = namespace

        # Initialise TCP port for ZMQ socket binding:
        self.timeout = timeout
        self.kill_request_port = kill_request_port
        self.send_port = send_port
        self.receive_port = receive_port
        self.heartbeat_port = heartbeat_port
        
        # INPUT MSG:
        self.scStateInMsg = messaging.SCStatesMsgReader() # For all S/C states.
        # self.vehConfigInMsg = messaging.VehicleConfigMsgReader() # For mass, Isc, CoM & ADCS states.        
        
        # OUTPUT MSG --> Should be designed to subscribe published ROS2 topic containing Cmd Force & Torque messages created by ROS2-side controllers.
        self.cmdForceOutMsg = messaging.CmdForceBodyMsg() # 
        self.cmdTorqueOutMsg = messaging.CmdTorqueBodyMsg()
        
        # Initialize instance variables for efficiency
        self.last_publish_time = 0
        self.publish_interval = 0.01  # Publish at 100 Hz max
        self.last_force_cmd = [0, 0, 0]
        self.last_torque_cmd = [0, 0, 0]
        
        # Initialize ZMQ Context
        self.context = zmq.Context()

        # --- Heartbeat monitoring from bridge ---
        self.heartbeat_sub = self.context.socket(zmq.SUB)
        self.heartbeat_sub.connect(f"tcp://localhost:{self.heartbeat_port}")
        self.heartbeat_sub.setsockopt_string(zmq.SUBSCRIBE, "")

        self.last_heartbeat_time = None
        self._running = True
        self._heartbeat_was_lost = False

        def heartbeat_monitor():
            while self._running:
                try:
                    msg = self.heartbeat_sub.recv_string(flags=zmq.NOBLOCK)
                    self.last_heartbeat_time = time.time()
                except zmq.Again:
                    time.sleep(0.01)
        threading.Thread(target=heartbeat_monitor, daemon=True).start()

        # --- ZMQ Publisher (BSK → ROS2) ---
        self.send_socket = self.context.socket(zmq.PUB)
        self.send_socket.setsockopt(zmq.LINGER, 0)  # Ensures the socket closes immediately
        try:
            self.send_socket.bind(f"tcp://*:{self.send_port}") # Combine port string
        except zmq.ZMQError as e:
            self.bskLogger.bskLog(sysModel.BSK_INFORMATION, f"Port {self.send_port} is already in use: {e}, check for available Localhost Ports and reset. \n\r Exiting Python Simulation...")
            exit(0)

        # --- ZMQ Subscriber (ROS2 → BSK) ---
        self.receive_socket = self.context.socket(zmq.SUB)
        self.receive_socket.connect(f"tcp://localhost:{self.receive_port}")  # ROS2 will publish here
        self.receive_socket.setsockopt_string(zmq.SUBSCRIBE, '')
        self.receive_socket.setsockopt(zmq.CONFLATE, 1)  # Only keep the latest message, TODO check if this will be problematic!

        # --- Kill request port ---
        self.kill_req_socket = self.context.socket(zmq.REQ)
        self.kill_req_socket.connect(f"tcp://localhost:{self.kill_request_port}")
        self.kill_req_socket.setsockopt(zmq.RCVTIMEO, -1)  # -1 means wait forever

        # --- Wait for the first heartbeat before starting! ---
        print("Waiting for bridge heartbeat...")
        while self.last_heartbeat_time is None:
            time.sleep(0.01)
        print("Bridge heartbeat detected. Starting module.")
        
    def Reset(self, CurrentSimNanos):
         # 1) Message subscription check -> throw BSK log error if not linked;
        if not self.scStateInMsg.isLinked():
            self.bskLogger.bskLog(sysModel.BSK_ERROR, f"Error: RosBridgeHandler.scStateInMsg wasn't connected.")
        # if not self.vehConfigInMsg.isLinked():
        #     self.bskLogger.bskLog(sysModel.BSK_ERROR, f"Error: RosBridgeHandler.vehConfigInMsg wasn't connected.")
        """
        The Reset method is used to clear out any persistent variables that need to get changed
        when a task is restarted.  This method is typically only called once after selfInit/crossInit,
        but it should be written to allow the user to call it multiple times if necessary.
        :param CurrentSimNanos: current simulation time in nano-seconds
        :return: none
        """
        return

    def UpdateState(self, CurrentSimNanos):
        """
        The updateState method is the cyclical worker method for a given Basilisk class.  It
        will get called periodically at the rate specified in the task that the model is
        attached to.  It persists and anything can be done inside of it.  If you have realtime
        requirements though, be careful about how much processing you put into a Python UpdateState
        method.  You could easily detonate your sim's ability to run in realtime.

        :param CurrentSimNanos: current simulation time in nano-seconds
        :return: none
        """
        now = time.time()
        if self.last_heartbeat_time is None or (now - self.last_heartbeat_time > 1.0):
            if not self._heartbeat_was_lost:
                print("No bridge heartbeat for >1s! Pausing UpdateState.")
                self._heartbeat_was_lost = True
            return
        else:
            if self._heartbeat_was_lost:
                print("Bridge heartbeat restored. Resuming UpdateState.")
                self._heartbeat_was_lost = False

        # read input message
        scStateInMsgBuffer = self.scStateInMsg()
        # vehConfigMsgBuffer = self.vehConfigInMsg()
        
        # create output message buffer
        forceOutMsgBuffer = messaging.CmdForceBodyMsgPayload()
        torqueOutMsgBuffer = messaging.CmdTorqueBodyMsgPayload()
        
        ### BSK -> ROS2 MSG:
        self.__ZMQ_SCState_Publisher(CurrentSimNanos, scStateInMsgBuffer)
        
        #### ROS2 -> BSK MSG:
        FrCmd, lrCmd = self.__ZMQ_Force_Torque_Listener()
        
        # Only update if commands changed
        if FrCmd != self.last_force_cmd:
            forceOutMsgBuffer.forceRequestBody = FrCmd
            self.cmdForceOutMsg.write(forceOutMsgBuffer, CurrentSimNanos, self.moduleID)
            self.last_force_cmd = FrCmd.copy() if isinstance(FrCmd, list) else FrCmd
        
        if lrCmd != self.last_torque_cmd:
            torqueOutMsgBuffer.torqueRequestBody = lrCmd
            self.cmdTorqueOutMsg.write(torqueOutMsgBuffer, CurrentSimNanos, self.moduleID)
            self.last_torque_cmd = lrCmd.copy() if isinstance(lrCmd, list) else lrCmd

        # All Python SysModels have self.bskLogger available
        # The logger level flags (i.e. BSK_INFORMATION) may be
        # accessed from sysModel
        # Only log occasionally to avoid performance impact
        if CurrentSimNanos % int(1e9) == 0:  # Log once per second
            self.bskLogger.bskLog(sysModel.BSK_DEBUG, f"------ RosBridgeHandler Module ------")
            self.bskLogger.bskLog(sysModel.BSK_DEBUG, f"Time: {CurrentSimNanos * 1.0E-9} s")
            self.bskLogger.bskLog(sysModel.BSK_DEBUG, f"Written Msg - ForceRequestBody: {FrCmd}")
            self.bskLogger.bskLog(sysModel.BSK_DEBUG, f"Written Msg - TorqueRequestBody: {lrCmd}")
            
        return
    
    def __ZMQ_SCState_Publisher(self, CurrentSimNanos, scStateInMsgBuffer):
        # scStateInMsgBuffer.sigma_BN
        BSKMsg = {
                "namespace": self.namespace,
                "topic": "/state",
                "time": CurrentSimNanos * 1.0E-9,
                "position": scStateInMsgBuffer.r_BN_N, # To check if we need `tolist()`!
                "velocity": scStateInMsgBuffer.v_BN_N,
                "attitude": RBK.MRP2EP(scStateInMsgBuffer.sigma_BN).tolist(), # convert MRP to Quaternions.
                "angular_velocity": scStateInMsgBuffer.omega_BN_B,
                "frame_id": "map"
        }
        try:
            self.send_socket.send_string(json.dumps(BSKMsg))
            self.bskLogger.bskLog(sysModel.BSK_DEBUG, f"Published: {BSKMsg}")

        except Exception as e:
            self.bskLogger.bskLog(sysModel.BSK_ERROR, f"BSK-To-ROS2 Publishing Error: {e}")
        
        return
    
    def __ZMQ_Force_Torque_Listener(self):
        """ Listens for incoming ZMQ messages from ROS2 and processes them. """
        try:
            ROS_zmq_msg = self.receive_socket.recv_string(flags=zmq.NOBLOCK)
            ROS2_raw_data_json = json.loads(ROS_zmq_msg)
            self.bskLogger.bskLog(sysModel.BSK_DEBUG, f"Received command from ROS2: {ROS2_raw_data_json}")
            
            # Unpack ROS2 json - now using combined wrench format:
            FrCmd = ROS2_raw_data_json["force"]
            lrCmd = ROS2_raw_data_json["torque"]
            
            # Reset delay count on successful receive
            global delay_count
            if delay_count > 0:
                delay_count = 0
                
            return FrCmd, lrCmd
            
        except zmq.Again:
            # No message available, return last known commands
            return self.last_force_cmd, self.last_torque_cmd
        except (json.JSONDecodeError, KeyError) as e:
            self.bskLogger.bskLog(sysModel.BSK_ERROR, f"Error parsing ROS2 message: {e}")
            return [0, 0, 0], [0, 0, 0]
        except Exception as e:
            self.bskLogger.bskLog(sysModel.BSK_ERROR, f"Unexpected error in ZMQ listener: {e}")
            return [0, 0, 0], [0, 0, 0]
    
    # Graceful exit function
    def Port_Clean_Exit(self):
        self._running = False
        self.send_socket.close()
        self.receive_socket.close()
        self.kill_req_socket.close()
        self.context.term()
        self.bskLogger.bskLog(sysModel.BSK_INFORMATION, f"ZMQ socket closed.")

    def Kill_Bridge_Send(self):
        global delay_count
        print(f'Delayed count for actuation messages: {delay_count}')
        isKilled = False
        while not isKilled:
            try:
                self.kill_req_socket.send_string("Kill")
                self.bskLogger.bskLog(sysModel.BSK_INFORMATION, "Bridge Kill Signal sent to Kill-REP port.")
                # try:
                reply = self.kill_req_socket.recv_string()
                if reply == "Killed":
                    isKilled = True
                    self.bskLogger.bskLog(sysModel.BSK_INFORMATION, "Bridge Killed. Exiting...")
                    break
                else:
                    self.bskLogger.bskLog(sysModel.BSK_ERROR, f"Bridge Kill-REP port replied error: {reply}")
                # except zmq.error.Again:
                #     pass

            except zmq.error.ZMQError:
                self.bskLogger.bskLog(sysModel.BSK_INFORMATION, "Operation cannot be accomplished in current state, trying again")
                pass
        self.Port_Clean_Exit()
