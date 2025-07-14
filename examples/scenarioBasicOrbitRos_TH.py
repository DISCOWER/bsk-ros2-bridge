import os
from Basilisk import __path__
from Basilisk.simulation import spacecraft, thrusterDynamicEffector
from Basilisk.utilities import SimulationBaseClass, macros, unitTestSupport, vizSupport, simIncludeThruster, fswSetupThrusters, simIncludeGravBody, orbitalMotion
from Basilisk.fswAlgorithms import thrForceMapping, thrFiringSchmitt
from Basilisk.simulation import simSynch
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

def run(liveStream=True, broadcastStream=True, simTimeStep=0.1, simTime=60.0, accelFactor=1.0, fswTimeStep=0.1, orbitCase='LEO'):
    simTaskName = "simTask"
    simProcessName = "simProcess"

    fswTaskName = "fswTask"
    fswProcessName = "fswProcess"

    scSim = SimulationBaseClass.SimBaseClass()

    simProcess = scSim.CreateNewProcess(simProcessName)
    fswProcess = scSim.CreateNewProcess(fswProcessName)
    simTimeStep = macros.sec2nano(simTimeStep)
    simProcess.addTask(scSim.CreateNewTask(simTaskName, simTimeStep))
    fswTimeStep = macros.sec2nano(fswTimeStep)
    fswProcess.addTask(scSim.CreateNewTask(fswTaskName, fswTimeStep))

    # Spacecraft definition
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bskSat"
    scObject.hub.mHub = 17.8
    I = [0.314, 0, 0, 0, 0.314, 0, 0, 0, 0.314]
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)

    # setup Gravity Body
    gravFactory = simIncludeGravBody.gravBodyFactory()
    planet = gravFactory.createEarth()
    planet.isCentralBody = True          # ensure this is the central gravitational body
    mu = planet.mu
    gravFactory.addBodiesTo(scObject)
    oe = orbitalMotion.ClassicElements()
    rLEO = 7000. * 1000      # meters
    rGEO = 42000. * 1000     # meters
    if orbitCase == 'GEO':
        oe.a = rGEO
        oe.e = 0.00001
        oe.i = 0.0 * macros.D2R
    elif orbitCase == 'GTO':
        oe.a = (rLEO + rGEO) / 2.0
        oe.e = 1.0 - rLEO / oe.a
        oe.i = 0.0 * macros.D2R
    else:                   # LEO case, default case 0
        oe.a = 2.5*rLEO
        oe.e = 0.10
        oe.i = 0.0 * macros.D2R
    oe.Omega = 0 * macros.D2R
    oe.omega = 0*347.8 * macros.D2R
    oe.f = 0*85.3 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    oe = orbitalMotion.rv2elem(mu, rN, vN)

    # initialize Spacecraft States with the initialization variables
    scObject.hub.r_CN_NInit = rN  # m   - r_BN_N
    scObject.hub.v_CN_NInit = vN  # m/s - v_BN_N


    scSim.AddModelToTask(simTaskName, scObject)

    # ROS Bridge Handler
    ros_bridge = RosBridgeHandler(namespace="bskSat")
    ros_bridge.bskLogger = bskLogging.BSKLogger(bskLogging.BSK_DEBUG)
    ros_bridge.add_bsk_msg_handler('SCStatesMsgPayload', 'out', 'scStateInMsg', 'sc_states')
    ros_bridge.add_bsk_msg_handler('THRArrayCmdForceMsgPayload', 'out', 'thrArrayCmdForceInMsg', 'thr_array_cmd_force')
    ros_bridge.add_bsk_msg_handler('THRArrayCmdForceMsgPayload', 'in', 'thrArrayCmdForceOutMsg', 'thr_array_cmd_force')
    scSim.AddModelToTask(simTaskName, ros_bridge)

    # Thrusters    
    # setup the Schmitt trigger thruster firing logic module
    thrFiringSchmittObj = thrFiringSchmitt.thrFiringSchmitt()
    thrFiringSchmittObj.ModelTag = "thrFiringSchmitt"
    scSim.AddModelToTask(fswTaskName, thrFiringSchmittObj)
    fswSetupThrusters.clearSetup()

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
    thrusterSet = thrusterDynamicEffector.ThrusterDynamicEffector()
    scSim.AddModelToTask(simTaskName, thrusterSet)
    thFactory = simIncludeThruster.thrusterFactory()
    for loc, dir in thruster_defs:
        thFactory.create(
            'MOOG_Monarc_1',
            loc,
            dir,
            MaxThrust=1.5,
            cutoffFrequency=62.8
        )
        fswSetupThrusters.create(loc, dir, 1.5)
    thrModelTag = "ThrusterDynamics"
    thFactory.addToSpacecraft(thrModelTag, thrusterSet, scObject)
    fswThrConfigMsg = fswSetupThrusters.writeConfigMessage()
    fswThrConfigMsg = thFactory.getConfigMessage()

    # Data logging
    dataLog = scObject.scStateOutMsg.recorder(simTimeStep)
    scSim.AddModelToTask(simTaskName, dataLog)

    # connect messages
    ros_bridge.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    ros_bridge.thrArrayCmdForceInMsg.subscribeTo(ros_bridge.thrArrayCmdForceOutMsg)
    thrFiringSchmittObj.thrConfInMsg.subscribeTo(fswThrConfigMsg)
    thrFiringSchmittObj.thrForceInMsg.subscribeTo(ros_bridge.thrArrayCmdForceOutMsg)
    thrusterSet.cmdsInMsg.subscribeTo(thrFiringSchmittObj.onTimeOutMsg)

    # Add second passive spacecraft
    scObject2 = spacecraft.Spacecraft()
    scObject2.ModelTag = "bskSat2"
    scObject2.hub.mHub = 17.8
    scObject2.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)
    scObject2.hub.r_CN_NInit = [rN[0] + 3, rN[1], rN[2]]
    scObject2.hub.v_CN_NInit = [vN[0], vN[1], vN[2]]
    gravFactory.addBodiesTo(scObject2)
    scSim.AddModelToTask(simTaskName, scObject2)

    # Add third passive spacecraft
    scObject3 = spacecraft.Spacecraft()
    scObject3.ModelTag = "bskSat3"
    scObject3.hub.mHub = 17.8
    scObject3.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)
    scObject3.hub.r_CN_NInit = [rN[0] - 3, rN[1], rN[2]]
    scObject3.hub.v_CN_NInit = [vN[0], vN[1], vN[2]]
    gravFactory.addBodiesTo(scObject3)
    scSim.AddModelToTask(simTaskName, scObject3)


    # Vizard support (optional)
    if vizSupport.vizFound:
        scData = vizInterface.VizSpacecraftData()
        scData.spacecraftName = scObject.ModelTag
        scData.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
        clockSync = simSynch.ClockSynch()
        clockSync.accelFactor = accelFactor
        scSim.AddModelToTask(simTaskName, clockSync)
        viz = vizSupport.enableUnityVisualization(scSim, simTaskName, [scObject, scObject2, scObject3],
                                                  thrEffectorList=[[thrusterSet], None, None],
                                                  thrColors=[[vizSupport.toRGBA255("white")], None, None],
                                                  liveStream=liveStream,
                                                  broadcastStream=broadcastStream,
                                                  )
        vizSupport.setActuatorGuiSetting(viz, showThrusterLabels=True)
        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=[scObject.ModelTag]
                                     , modelPath="/home/eliaskra/.gz/fuel/fuel.gazebosim.org/proque/models/atmos/1/meshes/atmos.glb"
                                     , offset=[0., 0., -0.2]
                                     , rotation=[0., 4.71238898038, 0.])
        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=[scObject2.ModelTag]
                                     , modelPath="/home/eliaskra/.gz/fuel/fuel.gazebosim.org/proque/models/atmos/1/meshes/atmos.glb"
                                     , offset=[0., 0., -0.2]
                                     , rotation=[0., 4.71238898038, 0.])
        
        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=[scObject3.ModelTag]
                                     , modelPath="/home/eliaskra/.gz/fuel/fuel.gazebosim.org/proque/models/atmos/1/meshes/atmos.glb"
                                     , offset=[0., 0., -0.2]
                                     , rotation=[0., 4.71238898038, 0.])

    scSim.InitializeSimulation()

    # Main simulation loop
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
        simTimeStep=1/50.,
        simTime=3600.0,
        accelFactor=1.0,
        fswTimeStep=1/10.
    )
