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
    # Number of spacecraft are defined by relative positions in inertial frame
    scName = ["leaderSc", "followerSc_1", "followerSc_2"]
    relative_positions = [
        [1.5, 0, 0],
        [0.5, -0.3, 0],
        [0.5, 0.3, 0],
    ]
    num_spacecraft = len(scName)

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
    
    # Loop over each spacecraft to set up properties and modules
    scObject = []
    thFactory = []
    thrusterSet = []
    thrFiringSchmittObj = []
    fswThrConfigMsg = []
    thrForceMapping = []
    for i in range(num_spacecraft):
        scObject_i = spacecraft.Spacecraft()
        scObject_i.ModelTag = scName[i]
        scObject.append(scObject_i)
        scObject_i.hub.mHub = m
        scObject_i.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)
        scObject_i.hub.r_CN_NInit = relative_positions[i]  # m
        scObject_i.hub.v_CN_NInit = [0, 0, 0]  # m/s
        
        # Setup thrusters for spacecraft
        thrusterSet_i = thrusterDynamicEffector.ThrusterDynamicEffector()
        thrusterSet.append(thrusterSet_i)
        thFactory_i = simIncludeThruster.thrusterFactory()
        for loc, dir in thruster_defs:
            thFactory_i.create(
                'MOOG_Monarc_1',
                loc,
                dir,
                MaxThrust=1.5,
                cutoffFrequency=3141.6,
                MinOnTime=0.0,
            )
        thFactory_i.addToSpacecraft(f"ThrusterDynamics{i}", thrusterSet_i, scObject_i)
        thFactory.append(thFactory_i)
        fswThrConfigMsg_i = thFactory_i.getConfigMessage()
        fswThrConfigMsg.append(fswThrConfigMsg_i)
        
        # Setup force/torque to thruster mapping
        thrForceMapping_i = forceTorqueThrForceMapping.forceTorqueThrForceMapping()
        thrForceMapping_i.ModelTag = f"thrForceMapping{i}"
        thrForceMapping.append(thrForceMapping_i)
        
        # Setup ROS bridge - Set up subscribers and publishers
        ros_bridge.add_ros_subscriber('CmdForceBodyMsgPayload', 'CmdForceBodyMsgOut', 'cmd_force', scName[i])
        ros_bridge.add_ros_subscriber('CmdTorqueBodyMsgPayload', 'CmdTorqueBodyMsgOut', 'cmd_torque', scName[i])
        ros_bridge.add_ros_publisher('SCStatesMsgPayload', 'SCStatesMsgIn', 'sc_states', scName[i], max_rate=50)
        ros_bridge.add_ros_publisher('THRArrayCmdForceMsgPayload', 'THRArrayCmdForceMsgIn', 'thr_array_cmd_force', scName[i], max_rate=10)

        # Setup the Schmitt trigger thruster firing logic module
        thrFiringSchmittObj_i = thrFiringSchmitt.thrFiringSchmitt()
        thrFiringSchmittObj_i.ModelTag = f"thrFiringSchmitt{i}"
        thrFiringSchmittObj_i.thrMinFireTime = 0.0
        thrFiringSchmittObj_i.level_on = 0.998
        thrFiringSchmittObj_i.level_off = 0.002
        thrFiringSchmittObj.append(thrFiringSchmittObj_i)

    # Vehicle configuration message (shared)
    vehicleConfigOut = messaging.VehicleConfigMsgPayload()
    vehicleConfigOut.ISCPntB_B = I
    vcMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)

    # --- Connect messages ---
    for i in range(num_spacecraft):
        # Get ROS bridge spacecraft interface
        ros_bridge_i = getattr(ros_bridge, scName[i])
        
        # Connect spacecraft state messages using namespace syntax
        ros_bridge_i.SCStatesMsgIn.subscribeTo(scObject[i].scStateOutMsg)
        ros_bridge_i.THRArrayCmdForceMsgIn.subscribeTo(thrForceMapping[i].thrForceCmdOutMsg)
        
        # Connect force/torque mapping
        thrForceMapping[i].thrConfigInMsg.subscribeTo(fswThrConfigMsg[i])
        thrForceMapping[i].vehConfigInMsg.subscribeTo(vcMsg)
        thrForceMapping[i].cmdForceInMsg.subscribeTo(ros_bridge_i.CmdForceBodyMsgOut)
        thrForceMapping[i].cmdTorqueInMsg.subscribeTo(ros_bridge_i.CmdTorqueBodyMsgOut)
        
        # Connect thruster logic
        thrFiringSchmittObj[i].thrConfInMsg.subscribeTo(fswThrConfigMsg[i])
        thrFiringSchmittObj[i].thrForceInMsg.subscribeTo(thrForceMapping[i].thrForceCmdOutMsg)
        thrusterSet[i].cmdsInMsg.subscribeTo(thrFiringSchmittObj[i].onTimeOutMsg)

    # --- Add models to simulation tasks ---
    scSim.AddModelToTask(simTaskName, ros_bridge, 1000)
    for i in range(num_spacecraft):
        scSim.AddModelToTask(simTaskName, thrusterSet[i], 10)
        scSim.AddModelToTask(simTaskName, scObject[i], 10)

        scSim.AddModelToTask(fswTaskName, thrForceMapping[i], 20)
        scSim.AddModelToTask(fswTaskName, thrFiringSchmittObj[i], 10)
    
    # --- Set up Vizard support ---
    if vizSupport.vizFound:        
        # Collect spacecraft data and thruster sets for all spacecraft
        scDataList = []
        for i, scObject_i in enumerate(scObject):
            scData = vizInterface.VizSpacecraftData()
            scData.spacecraftName = scObject_i.ModelTag
            scData.scStateInMsg.subscribeTo(scObject_i.scStateOutMsg)
            scDataList.append(scData)
            
        clockSync = simSynch.ClockSynch()
        clockSync.accelFactor = accelFactor
        scSim.AddModelToTask(simTaskName, clockSync)
        
        viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject,
                              liveStream=liveStream, broadcastStream=broadcastStream)
        for scObject_i in scObject:
            vizSupport.createCustomModel(viz, simBodiesToModify=[scObject_i.ModelTag],
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
