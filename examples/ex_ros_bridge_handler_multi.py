import pytest
import inspect, os, sys

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
# Add the parent directory to the path to access bsk_module
sys.path.append(os.path.join(path, '..'))

from Basilisk.architecture import messaging, bskLogging
from Basilisk.utilities import SimulationBaseClass, macros, unitTestSupport
from Basilisk.simulation import simSynch
from bsk_module.rosBridgeHandler import RosBridgeHandler


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

    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Define different update rates for each spacecraft (in seconds)
    spacecraft_configs = [
        {"namespace": "test_sat1", "rate": 0.01},   # 100 Hz
        {"namespace": "test_sat2", "rate": 0.02},   # 50 Hz  
        {"namespace": "test_sat3", "rate": 0.005}   # 200 Hz
    ]
    
    spacecraft_data = []
    
    for i, config in enumerate(spacecraft_configs):
        processName = f"spacecraft_{i+1}_process"
        taskName = f"spacecraft_{i+1}_task"
        testProcessRate = macros.sec2nano(config["rate"])
        testProc = unitTestSim.CreateNewProcess(processName)
        testProc.addTask(unitTestSim.CreateNewTask(taskName, testProcessRate))
        clockSync = simSynch.ClockSynch()
        clockSync.accelFactor = 1.0
        unitTestSim.AddModelToTask(taskName, clockSync)

        print(f"Setting up spacecraft {i+1} with namespace '{config['namespace']}'")
        print(f"  Update rate: {config['rate']:.3f}s ({1/config['rate']:.1f} Hz)")
        print(f"  Using standard ZMQ ports: 5550-5552")

        # Create bridge handler
        module = RosBridgeHandler(namespace=config["namespace"])
        module.ModelTag = f"ros_bridge_handler_{i+1}"
        module.bskLogger = bskLogging.BSKLogger(bskLogging.BSK_DEBUG)

        # Add message readers/writers if types are discovered
        if 'SCStatesMsgPayload' in module.bsk_msg_types:
            scstate_reader = module.add_bsk_msg_reader('SCStatesMsgPayload', 'scStateInMsg', 'sc_states')
        else:
            print(f"ERROR: SCStatesMsgPayload not discovered for {config['namespace']}")
            continue
        
        if 'CmdForceBodyMsgPayload' in module.bsk_msg_types:
            force_writer = module.add_bsk_msg_writer('CmdForceBodyMsgPayload', 'cmdForceOutMsg', 'cmd_force_body')
        if 'CmdTorqueBodyMsgPayload' in module.bsk_msg_types:
            torque_writer = module.add_bsk_msg_writer('CmdTorqueBodyMsgPayload', 'cmdTorqueOutMsg', 'cmd_torque_body')

        unitTestSim.AddModelToTask(taskName, module)

        # Create unique spacecraft state data for each spacecraft
        scStateInData = messaging.SCStatesMsgPayload()
        # Base state with variations for each spacecraft
        base_pos = [1000., 100000., -10000.]
        base_vel = [200., 300., -400.]
        base_attitude = [0.1, -0.2, 0.3]
        base_omega = [0.01, -0.02, 0.03]
        base_r_cn_n = [1000., 100000., -10000.]
        base_v_cn_n = [200., 300., -400.]
        base_omega_dot = [0.0, 0.0, 0.0]
        base_total_accum_dv_bdy = [0.0, 0.0, 0.0]
        base_total_accum_dv_bn_b = [0.0, 0.0, 0.0]
        base_total_accum_dv_cn_n = [0.0, 0.0, 0.0]
        base_non_conservative_accelpnt_b_b = [0.0, 0.0, 0.0]
        base_mrp_switch_count = 0

        # Add small variations to avoid identical spacecraft
        pos_variation = i * 1000
        vel_variation = i * 10
        att_variation = i * 0.01
        omega_variation = i * 0.001

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
        scStateInData.r_CN_N = base_r_cn_n
        scStateInData.v_CN_N = base_v_cn_n
        scStateInData.omegaDot_BN_B = base_omega_dot
        scStateInData.TotalAccumDVBdy = base_total_accum_dv_bdy
        scStateInData.TotalAccumDV_BN_B = base_total_accum_dv_bn_b
        scStateInData.TotalAccumDV_CN_N = base_total_accum_dv_cn_n
        scStateInData.nonConservativeAccelpntB_B = base_non_conservative_accelpnt_b_b
        scStateInData.MRPSwitchCount = base_mrp_switch_count

        scStateInMsg = messaging.SCStatesMsg().write(scStateInData)
        module.scStateInMsg.subscribeTo(scStateInMsg)

        spacecraft_data.append({
            'module': module,
            'namespace': config["namespace"],
            'rate': config["rate"],
            'process': processName,
            'task': taskName
        })

    print(f"\nInitializing Basilisk simulation with 3 spacecraft...")
    print(f"Simulation time: {sim_time}s")
    print("Each spacecraft operates at its own update rate but shares ZMQ ports via bridge.\n")
    
    unitTestSim.InitializeSimulation()
    print("Simulation initialized successfully!")
    print(f"Running simulation...")
    unitTestSim.ConfigureStopTime(macros.sec2nano(sim_time))
    unitTestSim.ExecuteSimulation()
    print("Simulation complete. Sending kill signals to all bridges...")

    for spacecraft in spacecraft_data:
        try:
            print(f"Killing bridge for {spacecraft['namespace']}...")
            spacecraft['module'].send_kill_signal()
        except Exception as e:
            print(f"Warning: Error killing bridge for {spacecraft['namespace']}: {e}")

    print(f"DONE: Multi-spacecraft simulation finished with {len(spacecraft_data)} spacecraft")

if __name__ == "__main__":
    print("=== Multi-Spacecraft Basilisk ROS Bridge Test ===")
    print("Fixed configuration: 3 spacecraft (test_sat1, test_sat2, test_sat3)")
    print("Each spacecraft runs at different update rates but shares ZMQ ports:")
    print("  test_sat1: 100 Hz")
    print("  test_sat2: 50 Hz") 
    print("  test_sat3: 200 Hz")
    print("All spacecraft share the same ZMQ ports: 5550-5552\n")
    sim_time = 10.0
    print(f"Simulation time: {sim_time}s\n")
    test_MultiSpacecraft(sim_time=sim_time)
    test_MultiSpacecraft(sim_time=sim_time)
