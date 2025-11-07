import sys, inspect, os
current_frame = inspect.currentframe()
if current_frame is not None:
    filename = inspect.getframeinfo(current_frame).filename
else:
    filename = __file__
path = os.path.dirname(os.path.abspath(filename))
sys.path.append(os.path.join(path, '..'))

from bsk_module.rosBridgeHandler import RosBridgeHandler
from Basilisk import __path__
from Basilisk.utilities import SimulationBaseClass, macros
from Basilisk.simulation import simSynch
from Basilisk.architecture import bskLogging, sysModel
from utils.tools import get_initial_conditions_from_hill

def run(simTimeStep=0.1, simTime=60.0, accelFactor=1.0, fswTimeStep=0.1, num_spacecraft=1):
    # Set up simulation classes and processes
    scSim = SimulationBaseClass.SimBaseClass()
    simTaskName = "simTask"
    simProcess = scSim.CreateNewProcess("simProcess", 100)
    simTimeStep = macros.sec2nano(simTimeStep)
    simProcess.addTask(scSim.CreateNewTask(simTaskName, simTimeStep))
    
    ros_bridge = RosBridgeHandler(accelFactor=accelFactor)
    ros_bridge.ModelTag = "ros_bridge"
    ros_bridge.bskLogger = sysModel.BSKLogger(bskLogging.BSK_DEBUG)
    
    for i in range(num_spacecraft):        
        # Setup ROS bridge - Set up subscribers and publishers            
        ros_bridge.add_ros_subscriber('THRArrayConfigMsgPayload', 'MsgIn', 'msg_in', f'bskSat{i}')
        ros_bridge.add_ros_publisher('THRArrayConfigMsgPayload', 'MsgOut', 'msg_out', f'bskSat{i}')

    # Connect messages
    for i in range(num_spacecraft):
        # Get the appropriate bridge and namespace object
        ros_bridge_i = getattr(ros_bridge, f'bskSat{i}')
        
        # Connect spacecraft state messages using namespace syntax
        ros_bridge_i.MsgOut.subscribeTo(ros_bridge_i.MsgIn)

    # Add models to simulation tasks
    scSim.AddModelToTask(simTaskName, ros_bridge)

    clockSync = simSynch.ClockSynch()
    clockSync.accelFactor = accelFactor
    scSim.AddModelToTask(simTaskName, clockSync)
            
    # Run the simulation
    scSim.InitializeSimulation()
    incrementalStopTime = 0
    while incrementalStopTime < macros.sec2nano(simTime):
        incrementalStopTime += simTimeStep
        scSim.ConfigureStopTime(incrementalStopTime)
        scSim.ExecuteSimulation()
    return

if __name__ == "__main__":
    run(
        simTimeStep=1.,
        simTime=10*3600.0,
        accelFactor=100.0,
        num_spacecraft=1,
    )
