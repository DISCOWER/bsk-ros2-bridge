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
from Basilisk.utilities import SimulationBaseClass, macros, unitTestSupport, vizSupport, simIncludeThruster, fswSetupThrusters, simIncludeGravBody, orbitalMotion
from Basilisk.fswAlgorithms import thrFiringSchmitt, forceTorqueThrForceMapping
from Basilisk.simulation import simSynch
from Basilisk.architecture import messaging, bskLogging, sysModel
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

    # Initial positions for formation (relative to base orbit)
    relative_positions = [
        [1.5, 0, 0],
        [0.5, -0.3, 0],
        [0.5, 0.3, 0],
    ]
    num_spacecraft = len(relative_positions)

    scName = ['leaderSc', 'followerSc_1', 'followerSc_2']

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
    
    # Lists to store spacecraft and thruster objects
    scObject = []
    thFactory = []
    thrusterSet = []
    thrFiringSchmittObj = []
    fswThrConfigMsg = []
    thrForceMapping = []
    
    for i in range(num_spacecraft):
        # Create spacecraft
        scObject_i = spacecraft.Spacecraft()
        scObject_i.ModelTag = scName[i]
        scObject_i.hub.mHub = m
        scObject_i.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)
        scObject_i.hub.r_CN_NInit = relative_positions[i]  # m
        scObject_i.hub.v_CN_NInit = [0, 0, 0]  # m/s
        scObject.append(scObject_i)
        
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
                cutoffFrequency=63.83,
                MinOnTime=1e-3,
            )
        thFactory_i.addToSpacecraft(f"ThrusterDynamics{i}", thrusterSet_i, scObject_i)
        thFactory.append(thFactory_i)
        fswThrConfigMsg_i = fswSetupThrusters.writeConfigMessage()
        fswThrConfigMsg_i = thFactory_i.getConfigMessage()
        fswThrConfigMsg.append(fswThrConfigMsg_i)
        
        # Setup force/torque to thruster mapping
        thrForceMapping_i = forceTorqueThrForceMapping.forceTorqueThrForceMapping()
        thrForceMapping_i.ModelTag = f"thrForceMapping{i}"
        thrForceMapping.append(thrForceMapping_i)
        
        # Setup ROS bridge - Set up subscribers and publishers
        ros_bridge.add_ros_subscriber('CmdForceBodyMsgPayload', 'CmdForceBodyMsgOut', 'cmd_force', f'bskSat{i}')
        ros_bridge.add_ros_subscriber('CmdTorqueBodyMsgPayload', 'CmdTorqueBodyMsgOut', 'cmd_torque', f'bskSat{i}')
        ros_bridge.add_ros_publisher('SCStatesMsgPayload', 'SCStatesMsgIn', 'sc_states', f'bskSat{i}')
        ros_bridge.add_ros_publisher('THRArrayCmdForceMsgPayload', 'THRArrayCmdForceMsgIn', 'thr_array_cmd_force', f'bskSat{i}')

        # Setup the Schmitt trigger thruster firing logic module
        thrFiringSchmittObj_i = thrFiringSchmitt.thrFiringSchmitt()
        thrFiringSchmittObj_i.ModelTag = f"thrFiringSchmitt{i}"
        thrFiringSchmittObj_i.thrMinFireTime = 1e-3
        thrFiringSchmittObj_i.level_on = 0.95
        thrFiringSchmittObj_i.level_off = 0.05
        thrFiringSchmittObj.append(thrFiringSchmittObj_i)

    # Connect messages    
    for i in range(num_spacecraft):
        # Create vehicle configuration messages for each spacecraft
        vehicleConfigOut = messaging.VehicleConfigMsgPayload()
        vehicleConfigOut.ISCPntB_B = I
        vcMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)
        
        # Get namespace object dynamically
        ros_bridge_i = getattr(ros_bridge, f'bskSat{i}')
        
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

    # Add models to simulation tasks
    scSim.AddModelToTask(fswTaskName, ros_bridge, 1000)
    for i in range(num_spacecraft):
        scSim.AddModelToTask(simTaskName, thrusterSet[i], 10)
        scSim.AddModelToTask(simTaskName, scObject[i], 10)

        scSim.AddModelToTask(fswTaskName, thrForceMapping[i], 11)
        scSim.AddModelToTask(fswTaskName, thrFiringSchmittObj[i], 10)
        

    # Vizard support (optional)
    if vizSupport.vizFound:
        scDataList = []
        thrEffectorList = []
        for i in range(num_spacecraft):
            thrEffectorList.append([thrusterSet[i]])
        
        # Collect spacecraft data and thruster sets for all spacecraft
        for i, scObject_i in enumerate(scObject):
            scData = vizInterface.VizSpacecraftData()
            scData.spacecraftName = scObject_i.ModelTag
            scData.scStateInMsg.subscribeTo(scObject_i.scStateOutMsg)
            scDataList.append(scData)
            
        clockSync = simSynch.ClockSynch()
        clockSync.accelFactor = accelFactor
        scSim.AddModelToTask(simTaskName, clockSync)
        
        viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject,
                              thrEffectorList=thrEffectorList,
                              liveStream=liveStream,
                              broadcastStream=broadcastStream,
                              )
        for scObject_i in scObject:
            vizSupport.createCustomModel(viz,
                                        simBodiesToModify=[scObject_i.ModelTag],
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
