import pytest
import inspect, os, sys

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
# Add the parent directory to the path to access bsk_module
sys.path.append(os.path.join(path, '..'))

from Basilisk.architecture import messaging, bskLogging
from Basilisk.utilities import SimulationBaseClass, macros, unitTestSupport
from Basilisk.simulation import simSynch
from bsk_module.ros_bridge_handler import RosBridgeHandler


@pytest.mark.parametrize("function", ["test_MultiSpacecraft"])

def test_MultiSpacecraftAllTest(function):
    """Module Unit Test"""
    [testResults, testMessage] = eval(function + '()')
    assert testResults < 1, testMessage


def test_MultiSpacecraft(sim_time=300.):
    """
    Three-Spacecraft Basilisk Simulation Test
    
    This example demonstrates how to extend the single spacecraft example (ex_ros_bridge_handler.py)
    to multiple spacecraft. It shows how to run three spacecraft (test_sat1, test_sat2, test_sat3)
    in a single Basilisk simulation, each with their own ROS bridge handler but sharing the same
    ZMQ ports through the bridge.
    
    Key differences from single spacecraft example:
    - Multiple processes/tasks for different update rates
    - Multiple RosBridgeHandler instances sharing ports
    - Slightly varied initial conditions for realistic scenarios
    
    Args:
        sim_time (float): Total simulation time in seconds
    """
    __tracebackhide__ = True

    testFailCount = 0
    testMessages = []
    
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Define different update rates for each spacecraft (in seconds)
    spacecraft_configs = [
        {"namespace": "test_sat1", "rate": 0.01},   # 100 Hz
        {"namespace": "test_sat2", "rate": 0.02},   # 50 Hz  
        {"namespace": "test_sat3", "rate": 0.005}   # 200 Hz
    ]
    
    # Create separate processes and tasks for each spacecraft to have different rates
    spacecraft_data = []
    
    for i, config in enumerate(spacecraft_configs):
        # Create separate process for each spacecraft to allow different rates
        processName = f"spacecraft_{i+1}_process"
        taskName = f"spacecraft_{i+1}_task"
        
        testProcessRate = macros.sec2nano(config["rate"])
        testProc = unitTestSim.CreateNewProcess(processName)
        testProc.addTask(unitTestSim.CreateNewTask(taskName, testProcessRate))
        
        # Add clock sync to each process
        clockSync = simSynch.ClockSynch()
        clockSync.accelFactor = 1.0
        unitTestSim.AddModelToTask(taskName, clockSync)
        
        print(f"Setting up spacecraft {i+1} with namespace '{config['namespace']}'")
        print(f"  Update rate: {config['rate']:.3f}s ({1/config['rate']:.1f} Hz)")
        print(f"  Using standard ZMQ ports: 5550-5552")
        
        # Create bridge handler - same as single example but with different namespace
        module = RosBridgeHandler(namespace=config["namespace"])
        module.ModelTag = f"ros_bridge_handler_{i+1}"
        
        # Add to the spacecraft's specific task
        unitTestSim.AddModelToTask(taskName, module)
        
        # Create unique spacecraft state data for each spacecraft
        scStateInData = messaging.SCStatesMsgPayload()
        
        # Base spacecraft state data (same for all but with variations)
        base_pos = [1000., 100000., -10000.]
        base_vel = [200., 300., -400.]
        base_attitude = [0., 0., 0.]
        base_omega = [1., -2., 3.]
        
        # Add small variations to avoid identical spacecraft (realistic for formation flying)
        pos_variation = i * 1000    # 1 km spacing
        vel_variation = i * 10      # 10 m/s variation
        att_variation = i * 0.01    # Small attitude differences
        omega_variation = i * 0.1   # Small angular velocity differences
        
        scStateInData.r_BN_N = [
            base_pos[0] + pos_variation,
            base_pos[1] + pos_variation * 0.5,
            base_pos[2] + pos_variation * 0.2
        ]
        scStateInData.v_BN_N = [
            base_vel[0] + vel_variation,
            base_vel[1] + vel_variation * 0.8,
            base_vel[2] + vel_variation * 0.6
        ]
        scStateInData.sigma_BN = [
            base_attitude[0] + att_variation,
            base_attitude[1] + att_variation * 0.5,
            base_attitude[2] - att_variation * 0.2
        ]
        scStateInData.omega_BN_B = [
            base_omega[0] + omega_variation * 0.1,
            base_omega[1] - omega_variation * 0.2,
            base_omega[2] + omega_variation * 0.3
        ]
        
        scStateInMsg = messaging.SCStatesMsg().write(scStateInData)
        
        # Connect the message to the module
        module.scStateInMsg.subscribeTo(scStateInMsg)
        module.bskLogger = bskLogging.BSKLogger(bskLogging.BSK_DEBUG)
        
        spacecraft_data.append({
            'module': module,
            'namespace': config["namespace"],
            'rate': config["rate"],
            'process': processName,
            'task': taskName
        })

    print(f"\nInitializing Basilisk simulation with 3 spacecraft...")
    print(f"Simulation time: {sim_time}s")
    print("Each spacecraft operates at its own update rate but shares ZMQ ports via bridge.")
    print("")
    
    # Start simulation
    unitTestSim.InitializeSimulation()
    print("Simulation initialized successfully!")
    
    # Run simulation
    print(f"Running simulation...")
    unitTestSim.ConfigureStopTime(macros.sec2nano(sim_time))
    unitTestSim.ExecuteSimulation()
    
    print("Simulation complete. Sending kill signals to all bridges...")
    
    # Send kill signals to all spacecraft bridges
    for spacecraft in spacecraft_data:
        try:
            print(f"Killing bridge for {spacecraft['namespace']}...")
            spacecraft['module'].send_kill_signal()
        except Exception as e:
            print(f"Warning: Error killing bridge for {spacecraft['namespace']}: {e}")
    
    # Done with simulation
    print(f"DONE: Multi-spacecraft simulation finished with {len(spacecraft_data)} spacecraft")


if __name__ == "__main__":
    print("=== Multi-Spacecraft Basilisk ROS Bridge Test ===")
    print("Fixed configuration: 3 spacecraft (test_sat1, test_sat2, test_sat3)")
    print("Each spacecraft runs at different update rates but shares ZMQ ports:")
    print("  test_sat1: 100 Hz")
    print("  test_sat2: 50 Hz") 
    print("  test_sat3: 200 Hz")
    print("All spacecraft share the same ZMQ ports: 5550-5552")
    print("")
    
    # Fixed simulation time
    sim_time = 10.0
    print(f"Simulation time: {sim_time}s")
    print("")
    
    test_MultiSpacecraft(sim_time=sim_time)
