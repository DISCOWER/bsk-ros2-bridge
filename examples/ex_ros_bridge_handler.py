import pytest
import inspect, os, sys
import argparse

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
# Add the parent directory to the path to access bsk_module
sys.path.append(os.path.join(path, '..'))

from Basilisk.architecture import bskLogging
from Basilisk.utilities import SimulationBaseClass, macros # general support file with common unit test functions
from Basilisk.simulation import simSynch, spacecraft, extForceTorque
from bsk_module.ros_bridge_handler import RosBridgeHandler
from Basilisk.architecture import messaging
import numpy as np
from Basilisk.architecture import sysModel

class CmdEchoBuffer(sysModel.SysModel):
    def __init__(self):
        super(CmdEchoBuffer, self).__init__()
        self.cmdForceBodyInMsg = messaging.CmdForceBodyMsgReader()
        self.cmdForceBodyOutMsg = messaging.CmdForceBodyMsg()

    def Reset(self, CurrentSimNanos):
        pass

    def UpdateState(self, CurrentSimNanos):
        if self.cmdForceBodyInMsg.isWritten():
            input_msg = self.cmdForceBodyInMsg()
            print(f"[EchoBuffer] <<< Read from rosHandlerModule.cmdForceBodyOutMsg: {input_msg.forceRequestBody}")
            self.cmdForceBodyOutMsg.write(input_msg, CurrentSimNanos, self.moduleID)

def test_RosBridgeHandlerAllTest(function):
    """rosHandlerModule Unit Test"""
    [testResults, testMessage] = eval(function + '()')
    assert testResults < 1, testMessage
    
def test_RosBridgeHandler(test_rate=0.01, sim_time=300., namespace="test_sat1"):
    """
    rosHandlerModule Unit Test
    
    Args:
        test_rate (float): Simulation update rate in seconds
        sim_time (float): Total simulation time in seconds  
        namespace (str): Spacecraft namespace for the bridge handler
    """
    __tracebackhide__ = True
    
    scSimName = "unitTask"
    unitProcessName = "TestProcess"

    # Create a sim rosHandlerModule as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(test_rate)
    testProc = scSim.CreateNewProcess(unitProcessName)
    testProc.addTask(scSim.CreateNewTask(scSimName, testProcessRate))

    # runRealtime setting
    clockSync = simSynch.ClockSynch()
    clockSync.accelFactor = 1.0
    scSim.AddModelToTask(scSimName, clockSync)

    # Create spacecraft dynamics (free-flying in space, no gravity)
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "test_sat1"
    
    # Set spacecraft properties
    scObject.hub.mHub = 17.8  # kg, spacecraft mass
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m, center of mass offset
    scObject.hub.IHubPntBc_B = [[0.314, 0.0, 0.0],
                                [0.0, 0.314, 0.0],
                                [0.0, 0.0, 0.314]]  # kg*m^2, spacecraft inertia
    
    # Set initial spacecraft states - start at origin for simplicity
    scObject.hub.r_CN_NInit = [0.0, 0.0, 0.0]  # m, initial position at origin
    scObject.hub.v_CN_NInit = [0.0, 0.0, 0.0]  # m/s, initial velocity at rest
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]  # MRP initial attitude (identity)
    scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]  # rad/s, initial angular velocity at rest

    # Create ROS bridge handler - it will automatically discover BSK message types
    rosHandlerModule = RosBridgeHandler(namespace=namespace)
    rosHandlerModule.ModelTag = "ros_bridge_handler"
    rosHandlerModule.bskLogger = bskLogging.BSKLogger(bskLogging.BSK_DEBUG)

    # Add message readers/writers only if the types were discovered
    scstate_reader = rosHandlerModule.add_bsk_msg_reader('SCStatesMsgPayload', 'scStateInMsg', 'sc_states')
    # force_reader = rosHandlerModule.add_bsk_msg_reader('CmdForceBodyMsgPayload', 'cmdForceBodyInMsg', 'cmd_force_body')
    
    # Add writers for command topics (receiving commands from ROS2)
    force_writer = rosHandlerModule.add_bsk_msg_writer('CmdForceBodyMsgPayload', 'cmdForceBodyOutMsg', 'cmd_force_body')
    torque_writer = rosHandlerModule.add_bsk_msg_writer('CmdTorqueBodyMsgPayload', 'cmdTorqueBodyOutMsg', 'cmd_torque_body')

    # Echo Buffer Module
    echoBufferModule = CmdEchoBuffer()
    echoBufferModule.ModelTag = "echoBuffer"
    echoBufferModule.cmdForceBodyInMsg.subscribeTo(rosHandlerModule.cmdForceBodyOutMsg)

    force_echo_reader = rosHandlerModule.add_bsk_msg_reader('CmdForceBodyMsgPayload', 'cmdForceBodyEchoInMsg', 'cmd_force_body')
    force_echo_reader.subscribeTo(echoBufferModule.cmdForceBodyOutMsg)

    # Create external force and torque rosHandlerModule to apply ROS commands
    extForceTorqueModule = extForceTorque.ExtForceTorque()
    extForceTorqueModule.ModelTag = "externalDisturbance"
    extForceTorqueModule.cmdForceBodyInMsg.subscribeTo(rosHandlerModule.cmdForceBodyOutMsg)
    scObject.addDynamicEffector(extForceTorqueModule)

    # Add spacecraft and rosHandlerModule to the simulation task
    scSim.AddModelToTask(scSimName, rosHandlerModule, 10)
    scSim.AddModelToTask(scSimName, echoBufferModule, 5)
    scSim.AddModelToTask(scSimName, extForceTorqueModule, 1)
    scSim.AddModelToTask(scSimName, scObject, 0)
    
    # Connect spacecraft state output to bridge handler input
    rosHandlerModule.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    # rosHandlerModule.cmdForceBodyInMsg.subscribeTo(rosHandlerModule.cmdForceBodyOutMsg)

    # Start simulation
    print(f"Initializing Basilisk simulation for namespace '{namespace}'...")
    scSim.InitializeSimulation()

    print(f"Running simulation for {sim_time}s at {1/test_rate:.1f} Hz...")
    scSim.ConfigureStopTime(macros.sec2nano(sim_time))
    scSim.ExecuteSimulation()
    
    print(f"Simulation complete. Sending kill signal to bridge...")
    rosHandlerModule.send_kill_signal()
    
    print(f"DONE: Simulation finished for spacecraft '{namespace}'")
    
    return [0, ""]

if __name__ == "__main__":
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Run Basilisk ROS Bridge Handler test')
    parser.add_argument('--namespace', '-n', type=str, default='test_sat1',
                       help='Spacecraft namespace for the bridge handler (default: test_sat1)')
    parser.add_argument('--rate', '-r', type=float, default=0.01,
                       help='Simulation update rate in seconds (default: 0.01 = 100 Hz)')
    parser.add_argument('--time', '-t', type=float, default=300.0,
                       help='Total simulation time in seconds (default: 300.0)')
    
    args = parser.parse_args()
    
    print(f"Starting Basilisk ROS Bridge Handler test with:")
    print(f"  Namespace: {args.namespace}")
    print(f"  Update rate: {args.rate:.6f}s ({1/args.rate:.1f} Hz)")
    print(f"  Simulation time: {args.time}s")
    print(f"  BSK messages will be automatically published to /{args.namespace}/bsk/out/")
    print(f"  BSK commands will be automatically subscribed from /{args.namespace}/bsk/in/")
    print("")
    
    test_RosBridgeHandler(
        test_rate=args.rate,
        sim_time=args.time,
        namespace=args.namespace
    )