import sys, inspect, os
current_frame = inspect.currentframe()
if current_frame is not None:
    filename = inspect.getframeinfo(current_frame).filename
else:
    filename = __file__
path = os.path.dirname(os.path.abspath(filename))
sys.path.append(os.path.join(path, '..'))

from Basilisk import __path__
from Basilisk.simulation import spacecraft, thrusterDynamicEffector, simSynch, vizInterface
from Basilisk.utilities import SimulationBaseClass, macros, unitTestSupport, vizSupport, simIncludeThruster
from Basilisk.fswAlgorithms import thrFiringSchmitt, forceTorqueThrForceMapping
from Basilisk.architecture import bskLogging, sysModel, messaging
from bsk_module.rosBridgeHandler import RosBridgeHandler

def run(liveStream=True, broadcastStream=True, simTimeStep=0.1, simTime=60.0, accelFactor=1.0, fswTimeStep=0.1):
    # --- Set up simulation classes and processes ---
    scSim = SimulationBaseClass.SimBaseClass()
    simTaskName = "simTask"
    fswTaskName = "fswTask"
    simProcess = scSim.CreateNewProcess("simProcess", 100)
    fswProcess = scSim.CreateNewProcess("fswProcess", 200)
    simTimeStep = macros.sec2nano(simTimeStep)
    simProcess.addTask(scSim.CreateNewTask(simTaskName, simTimeStep))
    fswTimeStep = macros.sec2nano(fswTimeStep)
    fswProcess.addTask(scSim.CreateNewTask(fswTaskName, fswTimeStep))

    # --- Create the ROS 2 bridge handler ---
    ros_bridge = RosBridgeHandler(accelFactor=accelFactor)
    ros_bridge.ModelTag = "ros_bridge"
    ros_bridge.bskLogger = sysModel.BSKLogger(bskLogging.BSK_DEBUG)

    # --- Set up all spacecraft ---
    scName = ["bskSat0"]

    # Shared spacecraft inertial properties
    m = 17.8  # kg, spacecraft mass
    I = [0.315, 0, 0, 0, 0.315, 0, 0, 0, 0.315] # kg m^2, inertia tensor

    # Shared spacecraft thruster definitions
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
    
    # Create spacecraft object
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
            cutoffFrequency=3141.6,
            MinOnTime=0.0,
        )
    thFactory.addToSpacecraft("ThrusterDynamics", thrusterSet, scObject)
    fswThrConfigMsg = thFactory.getConfigMessage()
    
    # Setup force/torque to thruster mapping
    thrForceMapping = forceTorqueThrForceMapping.forceTorqueThrForceMapping()
    thrForceMapping.ModelTag = "thrForceMapping"
    
    # Setup ROS bridge - Set up subscribers and publishers
    ros_bridge.add_ros_subscriber('CmdForceBodyMsgPayload', 'CmdForceBodyMsgOut', 'cmd_force', scName[0])
    ros_bridge.add_ros_subscriber('CmdTorqueBodyMsgPayload', 'CmdTorqueBodyMsgOut', 'cmd_torque', scName[0])
    ros_bridge.add_ros_publisher('SCStatesMsgPayload', 'SCStatesMsgIn', 'sc_states', scName[0], max_rate=50)
    ros_bridge.add_ros_publisher('THRArrayCmdForceMsgPayload', 'THRArrayCmdForceMsgIn', 'thr_array_cmd_force', scName[0], max_rate=10)

    # Setup the Schmitt trigger thruster firing logic module
    thrFiringSchmittObj = thrFiringSchmitt.thrFiringSchmitt()
    thrFiringSchmittObj.ModelTag = "thrFiringSchmitt"
    thrFiringSchmittObj.thrMinFireTime = 0.0
    thrFiringSchmittObj.level_on = 0.998
    thrFiringSchmittObj.level_off = 0.002

    # Vehicle configuration message (shared)
    vehicleConfigOut = messaging.VehicleConfigMsgPayload()
    vehicleConfigOut.ISCPntB_B = I
    vcMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)

    # --- Connect messages ---
    ros_bridge_i = getattr(ros_bridge, scName[0])

    # Connect spacecraft state messages using namespace syntax
    ros_bridge_i.SCStatesMsgIn.subscribeTo(scObject.scStateOutMsg)
    ros_bridge_i.THRArrayCmdForceMsgIn.subscribeTo(thrForceMapping.thrForceCmdOutMsg)

    # Connect force/torque mapping
    thrForceMapping.thrConfigInMsg.subscribeTo(fswThrConfigMsg)
    thrForceMapping.vehConfigInMsg.subscribeTo(vcMsg)
    thrForceMapping.cmdForceInMsg.subscribeTo(ros_bridge_i.CmdForceBodyMsgOut)
    thrForceMapping.cmdTorqueInMsg.subscribeTo(ros_bridge_i.CmdTorqueBodyMsgOut)
    
    # Connect thruster logic
    thrFiringSchmittObj.thrConfInMsg.subscribeTo(fswThrConfigMsg)
    thrFiringSchmittObj.thrForceInMsg.subscribeTo(thrForceMapping.thrForceCmdOutMsg)
    thrusterSet.cmdsInMsg.subscribeTo(thrFiringSchmittObj.onTimeOutMsg)

    # --- Add models to simulation tasks ---
    scSim.AddModelToTask(simTaskName, thrusterSet, 10)
    scSim.AddModelToTask(simTaskName, scObject, 10)
    scSim.AddModelToTask(simTaskName, ros_bridge, 100)
    
    scSim.AddModelToTask(fswTaskName, thrForceMapping, 20)
    scSim.AddModelToTask(fswTaskName, thrFiringSchmittObj, 10)
    
    # --- Set up Vizard support ---
    if vizSupport.vizFound:
        scData = vizInterface.VizSpacecraftData()
        scData.spacecraftName = scObject.ModelTag
        scData.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
            
        clockSync = simSynch.ClockSynch()
        clockSync.accelFactor = accelFactor
        scSim.AddModelToTask(simTaskName, clockSync)
        
        viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject,
                              liveStream=liveStream, broadcastStream=broadcastStream)
        vizSupport.createCustomModel(viz, simBodiesToModify=[scObject.ModelTag],
                                    modelPath='bskSat', scale=[0.1]*3)
            
    # --- Run the simulation ---
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
        simTimeStep=1/50.,
        simTime=3600.0,
        accelFactor=1.0,
        fswTimeStep=1/10.
    )
