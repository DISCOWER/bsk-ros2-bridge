import sys, inspect, os
current_frame = inspect.currentframe()
if current_frame is not None:
    filename = inspect.getframeinfo(current_frame).filename
else:
    filename = __file__
path = os.path.dirname(os.path.abspath(filename))
sys.path.append(os.path.join(path, '..'))
sys.path.append(os.path.join(path, '../..'))

from Basilisk import __path__
from Basilisk.simulation import spacecraft, thrusterDynamicEffector, simSynch, simpleNav, vizInterface, mujoco, svIntegrators, NBodyGravity, pointMassGravityModel
from Basilisk.utilities import SimulationBaseClass, macros, vizSupport, simIncludeThruster, simIncludeGravBody, orbitalMotion
from Basilisk.fswAlgorithms import thrFiringSchmitt, hillStateConverter, forceTorqueThrForceMapping, hillPoint, attTrackingError
from Basilisk.architecture import bskLogging, sysModel, messaging
from bsk_module.rosBridgeHandler import RosBridgeHandler
from examples.utils.tools import get_hill_frame_attitude, get_initial_conditions_from_hill

# MuJoCo XML file path
CURRENT_FOLDER = os.path.dirname(__file__)
XML_PATH = f"{CURRENT_FOLDER}/mujoco_setup/multi_spacecraft.xml"

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

    # --- Set up gravitational bodies and orbit ---
    # Create gravitational bodies
    gravFactory = simIncludeGravBody.gravBodyFactory()
    planet = gravFactory.createEarth()
    planet.isCentralBody = True

    # Orbit initial conditions
    mu = planet.mu
    oe = orbitalMotion.ClassicElements()
    oe.a = (6378 + 400) * 1000 # m, Semi-major axis
    oe.e = 0.0001
    oe.i = 60 * macros.D2R
    oe.Omega = 270 * macros.D2R
    oe.omega = 90 * macros.D2R
    oe.f = 5 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    oe = orbitalMotion.rv2elem(mu, rN, vN)

    # Create virtual spacecraft to define local Hill frame reference
    scHillObj = spacecraft.Spacecraft()
    scHillObj.ModelTag = "bskSatHill"
    scHillObj.hub.r_CN_NInit = rN  # m   - r_BN_N
    scHillObj.hub.v_CN_NInit = vN  # m/s - v_BN_N
    gravFactory.addBodiesTo(scHillObj)
    hillNavObj = simpleNav.SimpleNav()
    hillNavObj.ModelTag = "chiefNav"
    hillPointObj = hillPoint.hillPoint()
    hillPointObj.ModelTag = "hillPoint"
    celBodyData = messaging.EphemerisMsgPayload()
    celBodyInMsg = messaging.EphemerisMsg().write(celBodyData)
    
    # --- Set up all spacecraft ---
    # Number of spacecraft are defined by relative positions in Hill frame
    positions_Hill = [
        [0, -2, 0.1],
        [0, 2, -0.1]
    ]
    velocities_Hill = [
        [0, 0.2, 0],
        [0, -0.2, 0]
    ]
    num_spacecraft = len(positions_Hill)

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
    
    # Create the MuJoCo scene for impact dynamics
    scene = mujoco.MJScene.fromFile(XML_PATH)
    
    # Set the integrator for the MuJoCo DynamicObject
    integ = svIntegrators.svIntegratorRKF45(scene)
    scene.setIntegrator(integ)
    
    # --- Add gravity to MuJoCo dynamics task ---
    gravity = NBodyGravity.NBodyGravity()
    gravity.ModelTag = "gravity"
    scene.AddModelToDynamicsTask(gravity)
    
    earthPm = pointMassGravityModel.PointMassGravityModel()
    earthPm.muBody = mu
    gravity.addGravitySource("earth", earthPm, isCentralBody=True)
    
    # Loop over each spacecraft to set up thruster dynamics and navigation modules
    scObject = []  # Basilisk spacecraft objects for thruster dynamics
    thFactory = []
    thrusterSet = []
    thrFiringSchmittObj = []
    fswThrConfigMsg = []
    hillStateNavObj = []
    scNavObj = []
    thrForceMapping = []
    attTrackObj = []
    mjBodies = []  # MuJoCo bodies for physics
    initial_conditions = []
    thrusterActuatorMsgs = []  # MuJoCo actuator messages
    
    # Create spacecraft objects first
    for i in range(num_spacecraft):
        # Create Basilisk spacecraft object for thruster dynamics (not for physics)
        scObject_i = spacecraft.Spacecraft()
        scObject_i.ModelTag = f"bskSat{i}"
        scObject.append(scObject_i)
        
        # Get MuJoCo body for physics (including collision dynamics)
        mjBody_i = scene.getBody(f"bskSat{i}")
        mjBodies.append(mjBody_i)
        
        # Apply gravity to MuJoCo body
        gravity.addGravityTarget(f"bskSat{i}", mjBody_i)
    
    # Configure each spacecraft
    for i in range(num_spacecraft):
        # Get already created objects
        scObject_i = scObject[i]
        mjBody_i = mjBodies[i]
        
        # Calculate initial conditions
        p_Hill, v_Hill = get_initial_conditions_from_hill(mu, rN, vN, positions_Hill[i], velocities_Hill[i])
        mrp_Hill = get_hill_frame_attitude(rN, vN)
        initial_conditions.append((p_Hill, v_Hill, mrp_Hill, [0, 0, 0])) # No initial angular velocity
        scObject_i.hub.sigma_BNInit = mrp_Hill
                
        # Setup Basilisk thruster dynamics with spacecraft object
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
        # Add thrusters to Basilisk spacecraft
        thFactory_i.addToSpacecraft(scObject_i.ModelTag, thrusterSet_i, scObject_i)
        thFactory.append(thFactory_i)
        fswThrConfigMsg_i = thFactory_i.getConfigMessage()
        fswThrConfigMsg.append(fswThrConfigMsg_i)
        
        # Create MuJoCo actuator messages for thrusters
        scThrusterMsgs = []
        for j in range(len(thruster_defs)):
            actuatorMsg = messaging.SingleActuatorMsg()
            scene.getSingleActuator(f"bskSat{i}_thr_{j}").actuatorInMsg.subscribeTo(actuatorMsg)
            scThrusterMsgs.append(actuatorMsg)
        thrusterActuatorMsgs.append(scThrusterMsgs)
        
        # Setup force/torque to thruster mapping
        thrForceMapping_i = forceTorqueThrForceMapping.forceTorqueThrForceMapping()
        thrForceMapping_i.ModelTag = f"thrForceMapping{i}"
        thrForceMapping.append(thrForceMapping_i)

        # Setup ROS bridge - Set up subscribers and publishers
        ros_bridge.add_ros_subscriber('CmdForceBodyMsgPayload', 'CmdForceBodyMsgOut', 'cmd_force', f'bskSat{i}')
        ros_bridge.add_ros_subscriber('CmdTorqueBodyMsgPayload', 'CmdTorqueBodyMsgOut', 'cmd_torque', f'bskSat{i}')
        ros_bridge.add_ros_publisher('SCStatesMsgPayload', 'SCStatesMsgIn', 'sc_states', f'bskSat{i}', max_rate=10)
        ros_bridge.add_ros_publisher('THRArrayCmdForceMsgPayload', 'THRArrayCmdForceMsgIn', 'thr_array_cmd_force', f'bskSat{i}', max_rate=10)
        ros_bridge.add_ros_publisher('HillRelStateMsgPayload', 'HillRelStateMsgIn', 'hill_trans_state', f'bskSat{i}', max_rate=20)
        ros_bridge.add_ros_publisher('AttGuidMsgPayload', 'AttGuidMsgIn', 'hill_rot_state', f'bskSat{i}', max_rate=20)

        # Setup the Schmitt trigger thruster firing logic module
        thrFiringSchmittObj_i = thrFiringSchmitt.thrFiringSchmitt()
        thrFiringSchmittObj_i.ModelTag = f"thrFiringSchmitt{i}"
        thrFiringSchmittObj_i.thrMinFireTime = 0.0
        thrFiringSchmittObj_i.level_on = 0.998
        thrFiringSchmittObj_i.level_off = 0.002
        thrFiringSchmittObj.append(thrFiringSchmittObj_i)

        # Navigation modules with proper priorities to avoid NavTransMsg errors
        scNavObj_i = simpleNav.SimpleNav()
        scNavObj_i.ModelTag = f"nav{i}"
        scNavObj.append(scNavObj_i)
        
        hillStateNavObj_i = hillStateConverter.hillStateConverter()
        hillStateNavObj_i.ModelTag = f"hillStateNavObj{i}"
        hillStateNavObj.append(hillStateNavObj_i)

        # Module to compute attitude tracking error sigma_B/R --> R is Hill-frame in our case.
        attTrackObj_i = attTrackingError.attTrackingError()
        attTrackObj_i.ModelTag = f"attTrackObj{i}"
        attTrackObj.append(attTrackObj_i)

    # Vehicle configuration message (shared)
    vehicleConfigOut = messaging.VehicleConfigMsgPayload()
    vehicleConfigOut.ISCPntB_B = I
    vcMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)

    # --- Connect messages ---
    hillNavObj.scStateInMsg.subscribeTo(scHillObj.scStateOutMsg)
    hillPointObj.transNavInMsg.subscribeTo(hillNavObj.transOutMsg)
    hillPointObj.celBodyInMsg.subscribeTo(celBodyInMsg)
    
    for i in range(num_spacecraft):
        # Get ROS bridge spacecraft interface
        ros_bridge_i = getattr(ros_bridge, f'bskSat{i}')
        
        # Connect spacecraft state messages from MuJoCo bodies (for collision physics)
        mjBodyStateOutMsg = mjBodies[i].getOrigin().stateOutMsg
        ros_bridge_i.SCStatesMsgIn.subscribeTo(mjBodyStateOutMsg)
        ros_bridge_i.THRArrayCmdForceMsgIn.subscribeTo(thrForceMapping[i].thrForceCmdOutMsg)
        ros_bridge_i.HillRelStateMsgIn.subscribeTo(hillStateNavObj[i].hillStateOutMsg)
        ros_bridge_i.AttGuidMsgIn.subscribeTo(attTrackObj[i].attGuidOutMsg)
        
        # Connect force/torque mapping
        thrForceMapping[i].thrConfigInMsg.subscribeTo(fswThrConfigMsg[i])
        thrForceMapping[i].vehConfigInMsg.subscribeTo(vcMsg)
        thrForceMapping[i].cmdForceInMsg.subscribeTo(ros_bridge_i.CmdForceBodyMsgOut)
        thrForceMapping[i].cmdTorqueInMsg.subscribeTo(ros_bridge_i.CmdTorqueBodyMsgOut)
        
        # Connect thruster logic to Basilisk spacecraft
        thrFiringSchmittObj[i].thrConfInMsg.subscribeTo(fswThrConfigMsg[i])
        thrFiringSchmittObj[i].thrForceInMsg.subscribeTo(thrForceMapping[i].thrForceCmdOutMsg)
        # Connect thruster on-time commands to the thruster system
        thrusterSet[i].cmdsInMsg.subscribeTo(thrFiringSchmittObj[i].onTimeOutMsg)
        
        # Navigation connections - use Basilisk spacecraft state
        scNavObj[i].scStateInMsg.subscribeTo(mjBodyStateOutMsg)
        hillStateNavObj[i].depStateInMsg.subscribeTo(scNavObj[i].transOutMsg)
        hillStateNavObj[i].chiefStateInMsg.subscribeTo(hillNavObj.transOutMsg)
        attTrackObj[i].attNavInMsg.subscribeTo(scNavObj[i].attOutMsg)
        attTrackObj[i].attRefInMsg.subscribeTo(hillPointObj.attRefOutMsg)

    # --- Add models to simulation tasks ---
    scSim.AddModelToTask(simTaskName, scHillObj, 90)
    scSim.AddModelToTask(simTaskName, hillNavObj, 92)
    scSim.AddModelToTask(simTaskName, hillPointObj, 91)
    scSim.AddModelToTask(simTaskName, scene, 50)  # Lower priority for MuJoCo scene
    scSim.AddModelToTask(simTaskName, ros_bridge, 1)
    for i in range(num_spacecraft):
        # Add only navigation and thruster modules (no Basilisk spacecraft physics)
        scSim.AddModelToTask(simTaskName, scNavObj[i], 75)
        scSim.AddModelToTask(simTaskName, hillStateNavObj[i], 70)
        scSim.AddModelToTask(simTaskName, attTrackObj[i], 60)
        
        scSim.AddModelToTask(fswTaskName, thrForceMapping[i], 20)
        scSim.AddModelToTask(fswTaskName, thrFiringSchmittObj[i], 10)
    
    # --- Set up Vizard support ---
    if vizSupport.vizFound:        
        # Collect spacecraft data and thruster sets for all spacecraft
        scData = vizInterface.VizSpacecraftData()
        scData.spacecraftName = scHillObj.ModelTag
        scData.scStateInMsg.subscribeTo(scHillObj.scStateOutMsg)
        scDataList = []
        scDataList.append(scData)
        for i, scObject_i in enumerate(scObject):
            scData = vizInterface.VizSpacecraftData()
            scData.spacecraftName = scObject_i.ModelTag
            scData.scStateInMsg.subscribeTo(scObject_i.scStateOutMsg)
            scDataList.append(scData)
            
        clockSync = simSynch.ClockSynch()
        clockSync.accelFactor = accelFactor
        scSim.AddModelToTask(simTaskName, clockSync)
        
        viz = vizSupport.enableUnityVisualization(scSim, simTaskName, [scHillObj] + scObject,
                            liveStream=liveStream, broadcastStream=broadcastStream)
        
        # Add VizSpacecraftData for each MuJoCo spacecraft
        for i in range(num_spacecraft):
            scData = vizInterface.VizSpacecraftData()
            scData.spacecraftName = f"bskSat{i}"
            scData.scStateInMsg.subscribeTo(mjBodies[i].getOrigin().stateOutMsg)
            viz.scData.push_back(scData)
        
        vizSupport.createCustomModel(viz, simBodiesToModify=[scHillObj.ModelTag],
                                    modelPath='SPHERE', scale=[0.01]*3)
        for i in range(num_spacecraft):
            vizSupport.createCustomModel(viz, simBodiesToModify=[f"bskSat{i}"],
                                        modelPath='bskSat', scale=[0.2]*3)
            
    # --- Run the simulation ---
    scSim.InitializeSimulation()
    
    # Set initial positions and velocities for MuJoCo bodies
    for i, (p_Hill, v_Hill, mrp_Hill, omega) in enumerate(initial_conditions):
        mjBodies[i].setPosition(p_Hill)
        mjBodies[i].setVelocity(v_Hill)
        mjBodies[i].setAttitude(mrp_Hill)
        mjBodies[i].setAttitudeRate(omega)
    
    # Run one simulation step to initialize all messages
    scSim.ConfigureStopTime(simTimeStep)
    scSim.ExecuteSimulation()
    
    # Main simulation loop with MuJoCo thruster control
    fswTimeStepSec = macros.NANO2SEC * fswTimeStep  # FSW timestep in seconds
    incrementalStopTime = simTimeStep
    while incrementalStopTime < macros.sec2nano(simTime):
        incrementalStopTime += simTimeStep
        scSim.ConfigureStopTime(incrementalStopTime)
        scSim.ExecuteSimulation()
        
        # Update MuJoCo thruster actuators from Basilisk thruster dynamics
        for i in range(num_spacecraft):
            # Read thruster on-time commands from Basilisk Schmitt trigger
            onTimePayload = thrFiringSchmittObj[i].onTimeOutMsg.read()
            for j in range(len(thruster_defs)):
                # Convert on-time to average thrust over the FSW timestep
                onTime = onTimePayload.OnTimeRequest[j] if j < len(onTimePayload.OnTimeRequest) else 0.0
                if onTime > 0 and fswTimeStepSec > 0:
                    # Scale on-time to get duty cycle, then multiply by max thrust
                    dutyCycle = min(onTime / fswTimeStepSec, 1.0)
                    thrust = 1.5 * dutyCycle
                else:
                    thrust = 0.0
                thrusterActuatorMsgs[i][j].write(messaging.SingleActuatorMsgPayload(input=thrust))
    return

if __name__ == "__main__":
    run(
        liveStream=True,
        broadcastStream=False,
        simTimeStep=1/100.,
        simTime=3600.0,
        accelFactor=1.0,
        fswTimeStep=1/10.
    )
