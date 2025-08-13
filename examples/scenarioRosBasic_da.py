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
from Basilisk.simulation import spacecraft, thrusterDynamicEffector
from Basilisk.utilities import SimulationBaseClass, macros, unitTestSupport, vizSupport, simIncludeThruster
from Basilisk.fswAlgorithms import thrFiringSchmitt
from Basilisk.simulation import simSynch
from Basilisk.architecture import bskLogging, sysModel
from Basilisk.simulation import vizInterface

def run(liveStream=True, broadcastStream=True, simTimeStep=0.1, simTime=60.0, accelFactor=1.0, fswTimeStep=0.1):
    # Set up simulation classes and processes
    scSim = SimulationBaseClass.SimBaseClass()
    simTaskName = "simTask"
    fswTaskName = "fswTask"
    simProcess = scSim.CreateNewProcess("simProcess", 100)
    fswProcess = scSim.CreateNewProcess("fswProcess", 200)
    simTimeStep = macros.sec2nano(simTimeStep)
    simProcess.addTask(scSim.CreateNewTask(simTaskName, simTimeStep))
    fswTimeStep = macros.sec2nano(fswTimeStep)
    fswProcess.addTask(scSim.CreateNewTask(fswTaskName, fswTimeStep))

    # Add spacecraft definitions
    m = 17.8  # kg, spacecraft mass
    I = [0.315, 0, 0, 0, 0.315, 0, 0, 0, 0.315]

    # Create a single shared ROS bridge handler for all spacecraft
    ros_bridge = RosBridgeHandler()
    ros_bridge.ModelTag = "ros_bridge"
    ros_bridge.bskLogger = sysModel.BSKLogger(bskLogging.BSK_DEBUG)

    # Define thrusters
    thruster_defs = [
        ([0, 0, 0.17], [1, 0, 0]),
        ([0, 0, 0.17], [-1, 0, 0]),
        ([0, 0, -0.17], [1, 0, 0]),
        ([0, 0, -0.17], [-1, 0, 0]),
        ([0.17, 0, 0], [0, 1, 0]),
        ([0.17, 0, 0], [0, -1, 0]),
        ([-0.17, 0, 0], [0, 1, 0]),
        ([-0.17, 0, 0], [0, -1, 0]),
        ([0, 0.17, 0], [0, 0, 1]),
        ([0, 0.17, 0], [0, 0, -1]),
        ([0, -0.17, 0], [0, 0, 1]),
        ([0, -0.17, 0], [0, 0, -1]),
    ]
    
    # Create spacecraft
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bskSat0"
    scObject.hub.mHub = m
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)
    scObject.hub.r_CN_NInit = [0, 0, 0]  # m
    scObject.hub.v_CN_NInit = [0, 0, 0]  # m/s
    
    # Setup thrusters for spacecraft
    thrusterSet = thrusterDynamicEffector.ThrusterDynamicEffector()
    thFactory = simIncludeThruster.thrusterFactory()
    for loc, dir in thruster_defs:
        thFactory.create(
            'MOOG_Monarc_1',
            loc,
            dir,
            MaxThrust=1.5,
            cutoffFrequency=63.83,
            MinOnTime=1e-3,
        )
    thFactory.addToSpacecraft("ThrusterDynamics", thrusterSet, scObject)
    fswThrConfigMsg = thFactory.getConfigMessage()
    
    # Setup ROS bridge - Set up subscribers and publishers
    ros_bridge.add_ros_subscriber('THRArrayCmdForceMsgPayload', 'THRArrayCmdForceMsgOut', 'thr_array_cmd_force', 'bskSat0')
    ros_bridge.add_ros_publisher('SCStatesMsgPayload', 'SCStatesMsgIn', 'sc_states', 'bskSat0')
    ros_bridge.add_ros_publisher('THRArrayCmdForceMsgPayload', 'THRArrayCmdForceMsgIn', 'thr_array_cmd_force', 'bskSat0')

    # Setup the Schmitt trigger thruster firing logic module
    thrFiringSchmittObj = thrFiringSchmitt.thrFiringSchmitt()
    thrFiringSchmittObj.ModelTag = "thrFiringSchmitt"
    thrFiringSchmittObj.thrMinFireTime = 1e-3
    thrFiringSchmittObj.level_on = 0.95
    thrFiringSchmittObj.level_off = 0.05

    # Connect messages
    ros_bridge.bskSat0.SCStatesMsgIn.subscribeTo(scObject.scStateOutMsg)
    ros_bridge.bskSat0.THRArrayCmdForceMsgIn.subscribeTo(ros_bridge.bskSat0.THRArrayCmdForceMsgOut)

    # Connect thruster logic
    thrFiringSchmittObj.thrConfInMsg.subscribeTo(fswThrConfigMsg)
    thrFiringSchmittObj.thrForceInMsg.subscribeTo(ros_bridge.bskSat0.THRArrayCmdForceMsgOut)
    thrusterSet.cmdsInMsg.subscribeTo(thrFiringSchmittObj.onTimeOutMsg)

    # Add models to simulation tasks
    scSim.AddModelToTask(simTaskName, thrusterSet, 10)
    scSim.AddModelToTask(simTaskName, scObject, 10)

    scSim.AddModelToTask(fswTaskName, ros_bridge, 100)
    scSim.AddModelToTask(fswTaskName, thrFiringSchmittObj, 10)

    # Vizard support (optional)
    if vizSupport.vizFound:
        # Collect spacecraft data and thruster sets for all spacecraft
        scData = vizInterface.VizSpacecraftData()
        scData.spacecraftName = scObject.ModelTag
        scData.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
            
        clockSync = simSynch.ClockSynch()
        clockSync.accelFactor = accelFactor
        scSim.AddModelToTask(simTaskName, clockSync)
        
        viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject,
                              thrEffectorList=thrusterSet,
                              liveStream=liveStream,
                              broadcastStream=broadcastStream,
                              )
        vizSupport.createCustomModel(viz,
                                    simBodiesToModify=[scObject.ModelTag],
                                    modelPath='bskSat',
                                    scale=[0.1, 0.1, 0.1])
            
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
        liveStream=False,
        broadcastStream=True,
        simTimeStep=1/100.,
        simTime=3600.0,
        accelFactor=1.0,
        fswTimeStep=1/10.
    )
