import inspect, os, sys
import argparse

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
# Add the parent directory to the path to access bsk_module
sys.path.append(os.path.join(path, '..'))

from Basilisk.architecture import bskLogging
from Basilisk.utilities import SimulationBaseClass, macros
from Basilisk.simulation import simSynch, spacecraft, extForceTorque
from bsk_module.rosBridgeHandler import RosBridgeHandler
    
def run(sim_rate=0.01, sim_time=300., namespace="test_sat1"):
    """
    rosHandlerModule Unit Test
    
    Args:
        sim_rate (float): Simulation update rate in seconds
        sim_time (float): Total simulation time in seconds  
        namespace (str): Spacecraft namespace for the bridge handler
    """
    
    simTaskName = "simTask"
    simProcessName = "simProcess"

    # Create test thread
    processRate = macros.sec2nano(sim_rate)
    process = scSim.CreateNewProcess(simProcessName)
    process.addTask(scSim.CreateNewTask(simTaskName, processRate))

    # Create a sim rosHandlerModule as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    # runRealtime setting
    clockSync = simSynch.ClockSynch()
    clockSync.accelFactor = 1.0
    scSim.AddModelToTask(simTaskName, clockSync)

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

    # Add message readers/writers (Msg type, direction, handler name, topic name)
    rosHandlerModule.add_bsk_msg_handler('SCStatesMsgPayload', 'out', 'scStateInMsg', 'sc_states')
    rosHandlerModule.add_bsk_msg_handler('CmdForceBodyMsgPayload', 'out', 'cmdForceBodyInMsg', 'cmd_force_body')
    rosHandlerModule.add_bsk_msg_handler('CmdTorqueBodyMsgPayload', 'out', 'cmdTorqueBodyInMsg', 'cmd_torque_body')
    rosHandlerModule.add_bsk_msg_handler('CmdForceBodyMsgPayload', 'in', 'cmdForceBodyOutMsg', 'cmd_force_body')
    rosHandlerModule.add_bsk_msg_handler('CmdTorqueBodyMsgPayload', 'in', 'cmdTorqueBodyOutMsg', 'cmd_torque_body')

    # Connect BSK messages to ROS bridge handler
    rosHandlerModule.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    rosHandlerModule.cmdForceBodyInMsg.subscribeTo(rosHandlerModule.cmdForceBodyOutMsg)
    rosHandlerModule.cmdTorqueBodyInMsg.subscribeTo(rosHandlerModule.cmdTorqueBodyOutMsg)

    # Create external force and torque module
    extForceTorqueModule = extForceTorque.ExtForceTorque()
    extForceTorqueModule.ModelTag = "externalDisturbance"
    extForceTorqueModule.cmdForceBodyInMsg.subscribeTo(rosHandlerModule.cmdForceBodyOutMsg)
    scObject.addDynamicEffector(extForceTorqueModule)

    # Add spacecraft and rosHandlerModule to the simulation task
    scSim.AddModelToTask(simTaskName, rosHandlerModule, 10)
    scSim.AddModelToTask(simTaskName, extForceTorqueModule, 5)
    scSim.AddModelToTask(simTaskName, scObject, 0)

    # Start simulation
    print(f"Initializing Basilisk simulation for namespace '{namespace}'...")
    scSim.InitializeSimulation()

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
    print(f"  Simulation time: {args.time} s")
    print(f"  Update rate: {1/args.rate:.1f} Hz")
    print(f"  BSK messages will be automatically published to /{args.namespace}/bsk/out/")
    print(f"  BSK commands will be automatically subscribed from /{args.namespace}/bsk/in/")
    print("")
    
    run(
        sim_rate=args.rate,
        sim_time=args.time,
        namespace=args.namespace
    )