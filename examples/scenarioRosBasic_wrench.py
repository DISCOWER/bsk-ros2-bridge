import os
from Basilisk import __path__
from Basilisk.simulation import spacecraft, thrusterDynamicEffector
from Basilisk.utilities import SimulationBaseClass, macros, unitTestSupport, vizSupport, simIncludeThruster, fswSetupThrusters, simIncludeGravBody, orbitalMotion
from Basilisk.fswAlgorithms import thrFiringSchmitt, forceTorqueThrForceMapping
from Basilisk.simulation import simSynch
from Basilisk.architecture import messaging, bskLogging, sysModel
try:
    from Basilisk.simulation import vizInterface
except ImportError:
    pass

import sys, inspect
current_frame = inspect.currentframe()
if current_frame is not None:
    filename = inspect.getframeinfo(current_frame).filename
else:
    filename = __file__
path = os.path.dirname(os.path.abspath(filename))
# Add the parent directory to the path to access bsk_module
sys.path.append(os.path.join(path, '..'))
from bsk_module.rosBridgeHandler import RosBridgeHandler

def run(liveStream=True, broadcastStream=True, simTimeStep=0.1, simTime=60.0, accelFactor=1.0, fswTimeStep=0.1):
    # Set up simulation classes and processes
    scSim = SimulationBaseClass.SimBaseClass()
    simTaskName = "simTask"
    fswTaskName = "fswTask"
    simProcess = scSim.CreateNewProcess("simProcess")
    fswProcess = scSim.CreateNewProcess("fswProcess")
    simTimeStep = macros.sec2nano(simTimeStep)
    simProcess.addTask(scSim.CreateNewTask(simTaskName, simTimeStep))
    fswTimeStep = macros.sec2nano(fswTimeStep)
    fswProcess.addTask(scSim.CreateNewTask(fswTaskName, fswTimeStep))

    # Add spacecraft definitions
    m = 17.8  # kg, spacecraft mass
    I = [0.314, 0, 0, 0, 0.314, 0, 0, 0, 0.314]
    
    # Create a single shared ROS bridge handler for all spacecraft
    ros_bridge = RosBridgeHandler()
    ros_bridge.ModelTag = "ros_bridge"
    ros_bridge.bskLogger = sysModel.BSKLogger(bskLogging.BSK_DEBUG)

    # Define thrusters
    thruster_defs = [
        ([0, 0, 0.12], [1, 0, 0]),
        ([0, 0, 0.12], [-1, 0, 0]),
        ([0, 0, -0.12], [1, 0, 0]),
        ([0, 0, -0.12], [-1, 0, 0]),
        ([0.12, 0, 0], [0, 1, 0]),
        ([0.12, 0, 0], [0, -1, 0]),
        ([-0.12, 0, 0], [0, 1, 0]),
        ([-0.12, 0, 0], [0, -1, 0]),
        ([0, 0.12, 0], [0, 0, 1]),
        ([0, 0.12, 0], [0, 0, -1]),
        ([0, -0.12, 0], [0, 0, 1]),
        ([0, -0.12, 0], [0, 0, -1]),
    ]
    
    # Create spacecraft
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bskSat"
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
            cutoffFrequency=6.28
        )
    thFactory.addToSpacecraft("ThrusterDynamics", thrusterSet, scObject)
    fswThrConfigMsg = fswSetupThrusters.writeConfigMessage()
    fswThrConfigMsg = thFactory.getConfigMessage()
    
    # Setup force/torque to thruster mapping
    thrForceMapping = forceTorqueThrForceMapping.forceTorqueThrForceMapping()
    thrForceMapping.ModelTag = "thrForceMapping"
    
    # Setup ROS bridge - Set up subscribers and publishers
    ros_bridge.add_ros_subscriber('CmdForceBodyMsgPayload', 'CmdForceBodyMsgOut', 'cmd_force', 'bskSat')
    ros_bridge.add_ros_subscriber('CmdTorqueBodyMsgPayload', 'CmdTorqueBodyMsgOut', 'cmd_torque', 'bskSat')
    ros_bridge.add_ros_publisher('SCStatesMsgPayload', 'SCStatesMsgIn', 'sc_states', 'bskSat')
    ros_bridge.add_ros_publisher('CmdForceBodyMsgPayload', 'CmdForceBodyMsgIn', 'cmd_force', 'bskSat')
    ros_bridge.add_ros_publisher('CmdTorqueBodyMsgPayload', 'CmdTorqueBodyMsgIn', 'cmd_torque', 'bskSat')
    ros_bridge.add_ros_publisher('THRArrayCmdForceMsgPayload', 'THRArrayCmdForceMsgIn', 'thr_array_cmd_force', 'bskSat')

    # Setup the Schmitt trigger thruster firing logic module
    thrFiringSchmittObj = thrFiringSchmitt.thrFiringSchmitt()
    thrFiringSchmittObj.ModelTag = "thrFiringSchmitt"

    # Connect messages
    # Create vehicle configuration messages for each spacecraft
    vehicleConfigOut = messaging.VehicleConfigMsgPayload()
    vehicleConfigOut.ISCPntB_B = I
    vcMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)
    
    # Connect spacecraft state messages using namespace syntax
    ros_bridge.bskSat.SCStatesMsgIn.subscribeTo(scObject.scStateOutMsg)
    ros_bridge.bskSat.CmdForceBodyMsgIn.subscribeTo(ros_bridge.bskSat.CmdForceBodyMsgOut)
    ros_bridge.bskSat.CmdTorqueBodyMsgIn.subscribeTo(ros_bridge.bskSat.CmdTorqueBodyMsgOut)
    ros_bridge.bskSat.THRArrayCmdForceMsgIn.subscribeTo(thrForceMapping.thrForceCmdOutMsg)
    
    # Connect force/torque mapping
    thrForceMapping.thrConfigInMsg.subscribeTo(fswThrConfigMsg)
    thrForceMapping.vehConfigInMsg.subscribeTo(vcMsg)
    thrForceMapping.cmdForceInMsg.subscribeTo(ros_bridge.bskSat.CmdForceBodyMsgOut)
    thrForceMapping.cmdTorqueInMsg.subscribeTo(ros_bridge.bskSat.CmdTorqueBodyMsgOut)
    
    # Connect thruster logic
    thrFiringSchmittObj.thrConfInMsg.subscribeTo(fswThrConfigMsg)
    thrFiringSchmittObj.thrForceInMsg.subscribeTo(thrForceMapping.thrForceCmdOutMsg)
    thrusterSet.cmdsInMsg.subscribeTo(thrFiringSchmittObj.onTimeOutMsg)

    # Add models to simulation tasks
    scSim.AddModelToTask(simTaskName, ros_bridge, 100)
    scSim.AddModelToTask(simTaskName, scObject, 1)
    scSim.AddModelToTask(simTaskName, thrusterSet, 5)
    
    scSim.AddModelToTask(fswTaskName, thrFiringSchmittObj, 6)
    scSim.AddModelToTask(fswTaskName, thrForceMapping, 80)

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
        liveStream=True,
        broadcastStream=False,
        simTimeStep=1/20.,
        simTime=3600.0,
        accelFactor=50.0,
        fswTimeStep=1/10.
    )
