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
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True
    
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(test_rate)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    ## runRealtime setting: Enslave simulation to run in Realtime by the simSync.ClockSync module function:
    clockSync = simSynch.ClockSynch()
    clockSync.accelFactor = 1.0
    # clockSync.accelFactor = 50.0
    unitTestSim.AddModelToTask(unitTaskName, clockSync) # Check if this task name is valid later...

    # Include test module with configurable namespace:
    module = RosBridgeHandler(namespace=namespace)
    module.ModelTag = "ros_bridge_handler"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, module)
    
    scStateInData = messaging.SCStatesMsgPayload()
    scStateInData.r_BN_N = [1000., 100000., -10000.]
    scStateInData.v_BN_N = [200., 300., -400.]
    scStateInData.sigma_BN = [0.,0.,0.]
    scStateInData.omega_BN_B = [1.,-2.,3.]
    
    scStateInMsg = messaging.SCStatesMsg().write(scStateInData)
    
    module.scStateInMsg.subscribeTo(scStateInMsg)
    module.bskLogger = bskLogging.BSKLogger(bskLogging.BSK_DEBUG)
    
    """ TO REMOVE:
    cmdMsgData = {
        "time": 0.,
        "Fcmd": [0.1, 2.0, -4.0],
        "lrCmd": [2.5, -3.0, 0.05],
    }

    cmdMsgDataJSON = json.dumps(cmdMsgData)

    # TODO modify in test
    context, sub_socket, pub_socket = __set_fake_bridge_send_receive_sockets(module)
    # 1) Sub socket listens to RosBridgeHandler.send_socket for fake JSON data
    try: 
        sub_socket.recv_string(flags=zmq.NOBLOCK)
    except zmq.Again:
        # time.sleep(0.1)  # Avoid busy-waiting
        pass # No new messages, continue loop    
    # 2) Pub socket publish fake JSON data to RosBridgeHandler.receive_socket
    pub_socket.send_string(cmdMsgDataJSON) # TODO make this to publish data during the BSK sim.
    """
    
    # Start simulation:
    print(f"Initializing Basilisk simulation for namespace '{namespace}'...")
    unitTestSim.InitializeSimulation()

    # Step the simulation to 3*process rate so 4 total steps including zero
    print(f"Running simulation for {sim_time}s at {1/test_rate:.1f} Hz...")
    unitTestSim.ConfigureStopTime(macros.sec2nano(sim_time))  # seconds to stop simulation
    unitTestSim.ExecuteSimulation()
    
    print(f"Simulation complete. Sending kill signal to bridge...")
    # Send kill signal to bridge
    module.send_kill_signal()
    
    # Done with simulation
    print(f"DONE: Simulation finished")
    
if __name__ == "__main__":
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Run Basilisk ROS Bridge Handler test')
    parser.add_argument('--namespace', '-n', type=str, default='test_sat1',
                       help='Spacecraft namespace for the bridge handler (default: test_sat1)')
    parser.add_argument('--rate', '-r', type=float, default=1/100,
                       help='Simulation update rate in seconds (default: 1/100 = 100 Hz)')
    parser.add_argument('--time', '-t', type=float, default=300.0,
                       help='Total simulation time in seconds (default: 300.0)')
    
    args = parser.parse_args()
    
    print(f"Starting Basilisk ROS Bridge Handler test with:")
    print(f"  Namespace: {args.namespace}")
    print(f"  Update rate: {args.rate:.6f}s ({1/args.rate:.1f} Hz)")
    print(f"  Simulation time: {args.time}s")
    print("")
    
    test_RosBridgeHandler(
        test_rate=args.rate,
        sim_time=args.time,
        namespace=args.namespace
    )