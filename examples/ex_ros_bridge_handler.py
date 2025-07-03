import pytest
import inspect, os, sys
import argparse

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
# Add the parent directory to the path to access bsk_module
sys.path.append(os.path.join(path, '..'))

from Basilisk.architecture import messaging, bskLogging
from Basilisk.utilities import SimulationBaseClass, macros, unitTestSupport # general support file with common unit test functions
from Basilisk.simulation import simSynch
from bsk_module.ros_bridge_handler import RosBridgeHandler


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_

@pytest.mark.parametrize("function", ["test_RosBridgeHandler"
                                    #   , "extForceInertialAndTorque"
                                      ])

def test_RosBridgeHandlerAllTest(function):
    """Module Unit Test"""
    [testResults, testMessage] = eval(function + '()')
    assert testResults < 1, testMessage
    
def test_RosBridgeHandler(test_rate=0.01, sim_time=300., namespace="test_sat1"):
    """
    Module Unit Test
    
    Args:
        test_rate (float): Simulation update rate in seconds
        sim_time (float): Total simulation time in seconds  
        namespace (str): Spacecraft namespace for the bridge handler
    """
    __tracebackhide__ = True
    
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(test_rate)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # runRealtime setting
    clockSync = simSynch.ClockSynch()
    clockSync.accelFactor = 1.0
    unitTestSim.AddModelToTask(unitTaskName, clockSync)

    # Create ROS bridge handler - it will automatically discover BSK message types
    module = RosBridgeHandler(namespace=namespace)
    module.ModelTag = "ros_bridge_handler"
    module.bskLogger = bskLogging.BSKLogger(bskLogging.BSK_DEBUG)

    # Add message readers/writers only if the types were discovered
    if 'SCStatesMsgPayload' in module.bsk_msg_types:
        scstate_reader = module.add_bsk_msg_reader('SCStatesMsgPayload', 'scStateInMsg', 'sc_states')
    else:
        print("ERROR: SCStatesMsgPayload not discovered")
        return [1, "SCStatesMsgPayload not found"]
    
    if 'CmdForceBodyMsgPayload' in module.bsk_msg_types:
        force_writer = module.add_bsk_msg_writer('CmdForceBodyMsgPayload', 'cmdForceOutMsg', 'cmd_force')
        
    if 'CmdTorqueBodyMsgPayload' in module.bsk_msg_types:
        torque_writer = module.add_bsk_msg_writer('CmdTorqueBodyMsgPayload', 'cmdTorqueOutMsg', 'cmd_torque')

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, module)
    
    # Create test spacecraft state data using the discovered types
    scStateInData = messaging.SCStatesMsgPayload()
    scStateInData.r_BN_N = [1000., 100000., -10000.]
    scStateInData.v_BN_N = [200., 300., -400.]
    scStateInData.sigma_BN = [0.1, -0.2, 0.3]
    scStateInData.omega_BN_B = [0.01, -0.02, 0.03]
    scStateInData.r_CN_N = [1000., 100000., -10000.]
    scStateInData.v_CN_N = [200., 300., -400.]
    scStateInData.omegaDot_BN_B = [0.001, -0.002, 0.003]
    scStateInData.TotalAccumDVBdy = [0.0, 0.0, 0.0]
    scStateInData.TotalAccumDV_BN_B = [0.0, 0.0, 0.0]
    scStateInData.TotalAccumDV_CN_N = [0.0, 0.0, 0.0]
    scStateInData.nonConservativeAccelpntB_B = [0.0, 0.0, 0.0]
    scStateInData.MRPSwitchCount = 0
    
    scStateInMsg = messaging.SCStatesMsg().write(scStateInData)
    
    # Connect the state message to the bridge handler
    module.scStateInMsg.subscribeTo(scStateInMsg)
    
    # Start simulation
    print(f"Initializing Basilisk simulation for namespace '{namespace}'...")
    unitTestSim.InitializeSimulation()

    print(f"Running simulation for {sim_time}s at {1/test_rate:.1f} Hz...")
    unitTestSim.ConfigureStopTime(macros.sec2nano(sim_time))
    unitTestSim.ExecuteSimulation()
    
    print(f"Simulation complete. Sending kill signal to bridge...")
    module.send_kill_signal()
    
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