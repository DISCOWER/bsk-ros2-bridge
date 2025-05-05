from Basilisk.architecture import sysModel, messaging
from Basilisk.utilities import orbitalMotion, unitTestSupport as UAT, RigidBodyKinematics as RBK # Import for Hill-frame conversion & control if necessary

import zmq, json, time, numpy as np, signal
import threading
global delay_count
delay_count = 0

# See https://hanspeterschaub.info/basilisk/Learn/makingModules/pyModules.html for Python Module creation original example.
class ROS2Handler(sysModel.SysModel):
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

        super(ROS2Handler, self).__init__()

    You class must implement the above four functions. Beyond these four functions you class
    can complete any other computations you need (``Numpy``, ``matplotlib``, vision processing
    AI, whatever).
    """
    def __init__(self, reply_port = 8888, kill_request_port = 9999, send_port = 5555, receive_port = 7070, timeout = 15):
        super(ROS2Handler, self).__init__()

        # Initialise TCP port for ZMQ socket binding:
        self.timeout = timeout
        self.reply_port = reply_port
        self.kill_request_port = kill_request_port
        self.send_port = send_port
        self.receive_port = receive_port
        
        # INPUT MSG:
        self.scStateInMsg = messaging.SCStatesMsgReader() # For all S/C states.
        # self.vehConfigInMsg = messaging.VehicleConfigMsgReader() # For mass, Isc, CoM & ADCS states.        
        
        # OUTPUT MSG --> Should be designed to subscribe published ROS2 topic containing Cmd Force & Torque messages created by ROS2-side controllers.
        self.cmdForceOutMsg = messaging.CmdForceBodyMsg() # 
        self.cmdTorqueOutMsg = messaging.CmdTorqueBodyMsg()
        
        # Initialize ZMQ Context
        self.context = zmq.Context()
        
        # ZMQ REP: Send Reply to ZMQ Bridge REQ port when ready
        self.reply_socket = self.context.socket(zmq.REP)
        self.reply_socket.bind(f"tcp://*:{self.reply_port}")
        self.reply_socket.setsockopt(zmq.RCVTIMEO, self.timeout * 1000) # Set 15-second timeout for receiving.
        
        # TODO 20250401 - Maybe add while loop for ?
        print("Waiting for ZMQ Bridge Request...")
        
        try:
            request = self.reply_socket.recv_string()
        except zmq.Again:
            print("Error - ZMQ Bridge not Ready, exiting...")
            self.reply_socket.close()
            exit(0)
            
        # Received `request` and check for handshaking:
        if request == "Start":
            self.reply_socket.send_string("Ready")
            print("ROS2Handler: Sent 'Ready' response to ZMQ Bridge.")
        else: 
            self.reply_socket.send_string("Error")
            self.reply_socket.close()
            exit(0)
        
        # Reconfigure REP socket for NO timeout:
        # self.reply_socket.setsockopt(zmq.RCVTIMEO, -1) # set REP socket to infinite wait time
        self.reply_socket.close()
        
        # ZMQ Publisher (BSK → ROS2)
        self.send_socket = self.context.socket(zmq.PUB)
        self.send_socket.setsockopt(zmq.LINGER, 0)  # Ensures the socket closes immediately
        try:
            self.send_socket.bind(f"tcp://*:{self.send_port}") # Combine port string
        except zmq.ZMQError as e:
            self.bskLogger.bskLog(sysModel.BSK_INFORMATION, f"Port {self.send_port} is already in use: {e}, check for available Localhost Ports and reset. \n\r Exiting Python Simulation...")
            exit(0)
        
        # ZMQ Subscriber (ROS2 → BSK)
        self.receive_socket = self.context.socket(zmq.SUB)
        self.receive_socket.connect(f"tcp://localhost:{self.receive_port}")  # ROS2 will publish here
        self.receive_socket.setsockopt_string(zmq.SUBSCRIBE, '')
        self.receive_socket.setsockopt(zmq.CONFLATE, 1)  # Only keep the latest message, TODO check if this will be problematic!

        # Register signal handlers for Ctrl+C (SIGINT) and Ctrl+Z (SIGTSTP)
        signal.signal(signal.SIGINT, lambda s, f: self.Port_Clean_Exit(s, f))   # Handle Ctrl+C, bind with `lambda` function to skip `self` instance.
        signal.signal(signal.SIGTSTP, lambda s, f: self.Port_Clean_Exit(s, f))  # Handle Ctrl+Z, bind with `lambda` function to skip `self` instance.
        
        # Kill request port:
        self.kill_req_socket = self.context.socket(zmq.REQ)
        self.kill_req_socket.connect(f"tcp://localhost:{self.kill_request_port}")
        self.kill_req_socket.setsockopt(zmq.RCVTIMEO, -1)  # 15-second timeout, convert sec to ms
        
        # Wait for ? seconds so that ZMQ Bridge is ready:
        sleep_time = 5
        print(f"Wait for {sleep_time} seconds to sync with ZMQ Bridge...")
        time.sleep(5)
        
        # Start ZMQ Listener Thread for TCP subscription:
        self.running = True
        self.receive_thread = threading.Thread(target=self.__ZMQ_Force_Torque_Listener, daemon=True)
        self.receive_thread.start()
        
    def Reset(self, CurrentSimNanos):
         # 1) Message subscription check -> throw BSK log error if not linked;
        if not self.scStateInMsg.isLinked():
            self.bskLogger.bskLog(sysModel.BSK_ERROR, f"Error: ROS2Handler.scStateInMsg wasn't connected.")
        # if not self.vehConfigInMsg.isLinked():
        #     self.bskLogger.bskLog(sysModel.BSK_ERROR, f"Error: ROS2Handler.vehConfigInMsg wasn't connected.")
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
        
        forceOutMsgBuffer.forceRequestBody = FrCmd # MIND if a NEGATIVE sign is needed when using the module!!!
        self.cmdForceOutMsg.write(forceOutMsgBuffer, CurrentSimNanos, self.moduleID)
        
        torqueOutMsgBuffer.torqueRequestBody = lrCmd # MIND if a NEGATIVE sign is needed when using the module!!!
        self.cmdTorqueOutMsg.write(torqueOutMsgBuffer, CurrentSimNanos, self.moduleID)

        # All Python SysModels have self.bskLogger available
        # The logger level flags (i.e. BSK_INFORMATION) may be
        # accessed from sysModel
        if True:
            self.bskLogger.bskLog(sysModel.BSK_INFORMATION, f"------ ROS2Handler Module ------")
            """Sample Python module method"""
            self.bskLogger.bskLog(sysModel.BSK_INFORMATION, f"Time: {CurrentSimNanos * 1.0E-9} s")
            
            # TODO
            # self.bskLogger.bskLog(sysModel.BSK_DEBUG, f"BSK-to-ROS2 converted message: {self.cmdForceOutMsg.read().forceRequestBody}")
            
            self.bskLogger.bskLog(sysModel.BSK_INFORMATION, f"Written Msg - ForceRequestBody: {self.cmdForceOutMsg.read().forceRequestBody}")
            self.bskLogger.bskLog(sysModel.BSK_INFORMATION, f"Written Msg - TorqueRequestBody: {self.cmdTorqueOutMsg.read().torqueRequestBody}")
            
        return
    
    def __ZMQ_SCState_Publisher(self, CurrentSimNanos, scStateInMsgBuffer):
        # scStateInMsgBuffer.sigma_BN
        BSKMsg = {
                "time": CurrentSimNanos * 1.0E-9,
                "position": scStateInMsgBuffer.r_BN_N, # To check if we need `tolist()`!
                "velocity": scStateInMsgBuffer.v_BN_N,
                "attitude": RBK.MRP2EP(scStateInMsgBuffer.sigma_BN).tolist(), # convert MRP to Quaternions.
                "omega": scStateInMsgBuffer.omega_BN_B,
        }
        try:
            self.send_socket.send_string(json.dumps(BSKMsg))
            self.bskLogger.bskLog(sysModel.BSK_INFORMATION, f"Published: {BSKMsg}")

        except Exception as e:
            self.bskLogger.bskLog(sysModel.BSK_ERROR, f"BSK-To-ROS2 Publishing Error: {e}")
        
        return
    
    def __ZMQ_Force_Torque_Listener(self):
        """ Listens for incoming ZMQ messages from ROS2 and processes them. """
        # while self.running:
        ROS_zmq_msg = None
        # while ROS_zmq_msg is None:
        try:
            # ROS_zmq_msg = self.receive_socket.recv_string(flags=0)
            ROS_zmq_msg = self.receive_socket.recv_string(flags=zmq.NOBLOCK)
        except zmq.Again:
            pass  # No message available, continue loop
        
        if ROS_zmq_msg:
        
            ROS2_raw_data_json = json.loads(ROS_zmq_msg)
            self.bskLogger.bskLog(sysModel.BSK_DEBUG, f"Received command from ROS2: {ROS2_raw_data_json}")
            
            # Unpack ROS2 json (TODO - Support dynamic JSON input of the key pair to be read from TCP!):
            FrCmd = ROS2_raw_data_json["ros_to_bsk_F"]
            lrCmd = ROS2_raw_data_json["ros_to_bsk_L"] 
        else: 
            FrCmd = [0,0,0] # Temporary
            lrCmd = [0,0,0] # Temporary
            print(f"ROS2 Controller side is not ready... sending empty force/torque commands for now...")
            global delay_count
            delay_count += 1
            
        return FrCmd, lrCmd
    
    # Graceful exit function
    def Port_Clean_Exit(self, signum=None, frame=None):
        self.running = False
        self.receive_thread.join()
        
        # self.reply_socket.close() # Maybe can close this port right after getting reply?
        self.send_socket.close() # Close the socket properly
        self.receive_socket.close() # Close the socket properly
        self.kill_req_socket.close()
        
        self.context.term() # Terminate ZMQ context
     
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
