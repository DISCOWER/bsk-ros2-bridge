import os
from Basilisk import __path__
from Basilisk.simulation import spacecraft, thrusterDynamicEffector
from Basilisk.utilities import SimulationBaseClass, macros, unitTestSupport, vizSupport, simIncludeThruster, fswSetupThrusters, simIncludeGravBody, orbitalMotion
from Basilisk.fswAlgorithms import thrForceMapping, thrFiringSchmitt, hillStateConverter
from Basilisk.simulation import simSynch, simpleNav
from Basilisk.architecture import messaging, bskLogging
try:
    from Basilisk.simulation import vizInterface
except ImportError:
    pass

import sys, inspect
filename = inspect.getframeinfo(inspect.currentframe()).filename
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

    # Setup Gravity Body
    gravFactory = simIncludeGravBody.gravBodyFactory()
    planet = gravFactory.createEarth()
    planet.isCentralBody = True
    mu = planet.mu
    oe = orbitalMotion.ClassicElements()
    oe.a = (6378 + 800) * 1000 # m, Semi-major axis
    oe.e = 0.01
    oe.i = 51.6342 * macros.D2R
    oe.Omega = 270 * macros.D2R
    oe.omega = 9.5212 * macros.D2R
    oe.f = 90 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    oe = orbitalMotion.rv2elem(mu, rN, vN)

    # Virtual Hill spacecraft
    scObjectHill = spacecraft.Spacecraft()
    scObjectHill.ModelTag = "bskSatHill"
    scObjectHill.hub.r_CN_NInit = rN  # m   - r_BN_N
    scObjectHill.hub.v_CN_NInit = vN  # m/s - v_BN_N
    gravFactory.addBodiesTo(scObjectHill)
    scChiefNav = simpleNav.SimpleNav()
    scChiefNav.ModelTag = f"chiefNav"

    # Add spacecraft definitions
    m = 17.8  # kg, spacecraft mass
    I = [0.314, 0, 0, 0, 0.314, 0, 0, 0, 0.314]
    
    # Create a single shared ROS bridge handler for all spacecraft
    ros_bridge = RosBridgeHandler(namespace="rosBridge")
    ros_bridge.ModelTag = "ros_bridge"
    ros_bridge.bskLogger = bskLogging.BSKLogger(bskLogging.BSK_DEBUG)
    
    # Initial positions for formation (relative to base orbit)
    relative_positions = [
        [0, 3, 0],
        [3, 0, 0],
        [-3, 0, 0],
        [0, -3, 0],
        [2.5, 2.5, 0],
        [2.5, -2.5, 0],
        [-2.5, 2.5, 0]
    ]
    num_spacecraft = len(relative_positions)

    # Define thrusters
    thruster_defs = [
        ([0, 0, 0.12], [1.5, 0, 0]),
        ([0, 0, 0.12], [-1.5, 0, 0]),
        ([0, 0, -0.12], [1.5, 0, 0]),
        ([0, 0, -0.12], [-1.5, 0, 0]),
        ([0.12, 0, 0], [0, 1.5, 0]),
        ([0.12, 0, 0], [0, -1.5, 0]),
        ([-0.12, 0, 0], [0, 1.5, 0]),
        ([-0.12, 0, 0], [0, -1.5, 0]),
        ([0, 0.12, 0], [0, 0, 1.5]),
        ([0, 0.12, 0], [0, 0, -1.5]),
        ([0, -0.12, 0], [0, 0, 1.5]),
        ([0, -0.12, 0], [0, 0, -1.5]),
    ]
    
    # Lists to store spacecraft and thruster objects
    scObject = []
    thFactory = []
    thrusterSet = []
    thrFiringSchmittObj = []
    fswThrConfigMsg = []
    hillStateNavObj = []
    scNavObj = []
    
    for i in range(num_spacecraft):
        # Create spacecraft
        scObject_i = spacecraft.Spacecraft()
        scObject_i.ModelTag = f"bskSat{i}"
        scObject_i.hub.mHub = m
        scObject_i.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)
        scObject_i.hub.r_CN_NInit = [rN[j] + relative_positions[i][j] for j in range(3)]
        scObject_i.hub.v_CN_NInit = vN
        gravFactory.addBodiesTo(scObject_i)
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
                cutoffFrequency=6.28
            )
        thFactory_i.addToSpacecraft(f"ThrusterDynamics{i}", thrusterSet_i, scObject_i)
        thFactory.append(thFactory_i)
        fswThrConfigMsg_i = thFactory_i.getConfigMessage()
        fswThrConfigMsg.append(fswThrConfigMsg_i)
        
        # Setup ROS bridge - Set up subscribers and publishers
        ros_bridge.add_ros_subscriber('THRArrayCmdForceMsgPayload', 'THRArrayCmdForceMsgOut', 'thr_array_cmd_force', f'bskSat{i}')
        ros_bridge.add_ros_publisher('SCStatesMsgPayload', 'SCStatesMsgIn', 'sc_states', f'bskSat{i}')
        ros_bridge.add_ros_publisher('THRArrayCmdForceMsgPayload', 'THRArrayCmdForceMsgIn', 'thr_array_cmd_force', f'bskSat{i}')
        ros_bridge.add_ros_publisher('HillRelStateMsgPayload', 'HillRelStateMsgIn', 'hill_rel_state', f'bskSat{i}')

        # Setup the Schmitt trigger thruster firing logic module
        thrFiringSchmittObj_i = thrFiringSchmitt.thrFiringSchmitt()
        thrFiringSchmittObj_i.ModelTag = f"thrFiringSchmitt{i}"
        thrFiringSchmittObj.append(thrFiringSchmittObj_i)

        # Navigation modules with proper priorities to avoid NavTransMsg errors
        scNavObj_i = simpleNav.SimpleNav()
        scNavObj_i.ModelTag = f"nav{i}"
        scNavObj.append(scNavObj_i)
        
        hillStateNavObj_i = hillStateConverter.hillStateConverter()
        hillStateNavObj_i.ModelTag = f"hillStateNavObj{i}"
        hillStateNavObj.append(hillStateNavObj_i)

    # Connect messages
    scChiefNav.scStateInMsg.subscribeTo(scObjectHill.scStateOutMsg)
    for i in range(num_spacecraft):
        # Get namespace object dynamically
        ros_bridge_i = getattr(ros_bridge, f'bskSat{i}')
        
        # Connect spacecraft state messages using namespace syntax
        ros_bridge_i.SCStatesMsgIn.subscribeTo(scObject[i].scStateOutMsg)
        ros_bridge_i.THRArrayCmdForceMsgIn.subscribeTo(ros_bridge_i.THRArrayCmdForceMsgOut)
        ros_bridge_i.HillRelStateMsgIn.subscribeTo(hillStateNavObj[i].hillStateOutMsg)
        
        # Connect thruster logic
        thrFiringSchmittObj[i].thrConfInMsg.subscribeTo(fswThrConfigMsg[i])
        thrFiringSchmittObj[i].thrForceInMsg.subscribeTo(ros_bridge_i.THRArrayCmdForceMsgOut)
        thrusterSet[i].cmdsInMsg.subscribeTo(thrFiringSchmittObj[i].onTimeOutMsg)

        # Navigation connections
        scNavObj[i].scStateInMsg.subscribeTo(scObject[i].scStateOutMsg)
        hillStateNavObj[i].depStateInMsg.subscribeTo(scNavObj[i].transOutMsg)
        hillStateNavObj[i].chiefStateInMsg.subscribeTo(scChiefNav.transOutMsg)

    # Add models to simulation tasks
    scSim.AddModelToTask(simTaskName, scObjectHill, 5)
    scSim.AddModelToTask(simTaskName, scChiefNav, 51)
    scSim.AddModelToTask(simTaskName, ros_bridge, 100)
    for i in range(num_spacecraft):
        scSim.AddModelToTask(simTaskName, scObject[i], 5)
        scSim.AddModelToTask(simTaskName, thrusterSet[i], 6)
        scSim.AddModelToTask(fswTaskName, thrFiringSchmittObj[i], 7)
        scSim.AddModelToTask(simTaskName, scNavObj[i], 41)
        scSim.AddModelToTask(fswTaskName, hillStateNavObj[i], 40)

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
        # vizSupport.setActuatorGuiSetting(viz, showThrusterLabels=True)
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
        liveStream=True,
        broadcastStream=False,
        simTimeStep=1/20.,
        simTime=3600.0,
        accelFactor=1.0,
        fswTimeStep=1/10.
    )
