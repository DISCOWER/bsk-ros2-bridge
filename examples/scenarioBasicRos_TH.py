import os
from Basilisk import __path__
from Basilisk.simulation import spacecraft, thrusterDynamicEffector, extForceTorque
from Basilisk.utilities import SimulationBaseClass, macros, unitTestSupport, vizSupport, simIncludeThruster, fswSetupThrusters
from Basilisk.fswAlgorithms import thrForceMapping, thrFiringSchmitt
from Basilisk.simulation import simSynch
from Basilisk.architecture import messaging
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

def run(liveStream=True, simTimeStep=0.1, simTime=60.0, accelFactor=1.0, fswTimeStep=0.1):
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
    scObject.hub.r_CN_NInit = [0.0, 0.0, 0.0]
    scObject.hub.v_CN_NInit = [0.0, 0.0, 0.0]
    scSim.AddModelToTask(simTaskName, scObject)

    # ROS Bridge Handler
    ros_bridge = RosBridgeHandler(namespace="bskSat")
    ros_bridge.add_bsk_msg_reader('SCStatesMsgPayload', 'scStateInMsg', 'sc_states')
    ros_bridge.add_bsk_msg_reader('THRArrayCmdForceMsgPayload', 'cmdForceInMsg', 'cmd_force_body')
    ros_bridge.add_bsk_msg_reader('CmdTorqueBodyMsgPayload', 'cmdTorqueInMsg', 'cmd_torque_body')
    ros_bridge.add_bsk_msg_writer('THRArrayCmdForceMsgPayload', 'cmdForceOutMsg', 'cmd_force_body')
    ros_bridge.add_bsk_msg_writer('CmdTorqueBodyMsgPayload', 'cmdTorqueOutMsg', 'cmd_torque_body')
    scSim.AddModelToTask(simTaskName, ros_bridge)

    # Thrusters
    # setup the thruster force mapping module
    # thrForceMappingObj = thrForceMapping.thrForceMapping()
    # thrForceMappingObj.ModelTag = "thrForceMapping"
    # scSim.AddModelToTask(fswTaskName, thrForceMappingObj)
    # thrForceMappingObj.thrForceSign = +1
    # thrForceMappingObj.controlAxes_B = [1, 0, 0,
    #                                     0, 1, 0,
    #                                     0, 0, 1]
    
    # setup the Schmitt trigger thruster firing logic module
    thrFiringSchmittObj = thrFiringSchmitt.thrFiringSchmitt()
    thrFiringSchmittObj.ModelTag = "thrFiringSchmitt"
    scSim.AddModelToTask(fswTaskName, thrFiringSchmittObj)
    # thrFiringSchmittObj.thrMinFireTime = 0.001
    # thrFiringSchmittObj.level_on = 0.501
    # thrFiringSchmittObj.level_off = 0.499
    fswSetupThrusters.clearSetup()

    # Define thrusters
    thruster_defs = [
        ([0, 0.12, 0], [1.5, 0, 0]),
        ([0, 0.12, 0], [-1.5, 0, 0]),
        ([0, -0.12, 0], [1.5, 0, 0]),
        ([0, -0.12, 0], [-1.5, 0, 0]),
        ([0, 0, 0.12], [0, -1.5, 0]),
        ([0, 0, 0.12], [0, 1.5, 0]),
        ([0, 0, -0.12], [0, -1.5, 0]),
        ([0, 0, -0.12], [0, 1.5, 0]),
        ([-0.12, 0, 0], [0, 0, -1.5]),
        ([-0.12, 0, 0], [0, 0, 1.5]),
        ([0.12, 0, 0], [0, 0, -1.5]),
        ([0.12, 0, 0], [0, 0, 1.5]),
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

    # create the FSW vehicle configuration message
    # vehicleConfigOut = messaging.VehicleConfigMsgPayload()
    # vehicleConfigOut.ISCPntB_B = I
    # vcMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)

    # Data logging
    dataLog = scObject.scStateOutMsg.recorder(simTimeStep)
    scSim.AddModelToTask(simTaskName, dataLog)

    # connect messages
    ros_bridge.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    ros_bridge.cmdForceInMsg.subscribeTo(ros_bridge.cmdForceOutMsg)
    ros_bridge.cmdTorqueInMsg.subscribeTo(ros_bridge.cmdTorqueOutMsg)
    # thrForceMappingObj.thrConfigInMsg.subscribeTo(fswThrConfigMsg)
    # thrForceMappingObj.vehConfigInMsg.subscribeTo(vcMsg)
    # thrForceMappingObj.cmdTorqueInMsg.subscribeTo(ros_bridge.cmdTorqueOutMsg)
    thrFiringSchmittObj.thrConfInMsg.subscribeTo(fswThrConfigMsg)
    thrFiringSchmittObj.thrForceInMsg.subscribeTo(ros_bridge.cmdForceOutMsg)
    thrusterSet.cmdsInMsg.subscribeTo(thrFiringSchmittObj.onTimeOutMsg)


    # Vizard support (optional)
    if vizSupport.vizFound:
        scData = vizInterface.VizSpacecraftData()
        scData.spacecraftName = scObject.ModelTag
        scData.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
        clockSync = simSynch.ClockSynch()
        clockSync.accelFactor = accelFactor
        scSim.AddModelToTask(simTaskName, clockSync)
        viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject,
                                                  thrEffectorList=thrusterSet,
                                                  thrColors=vizSupport.toRGBA255("white"),
                                                  liveStream=liveStream,
                                                  )
        vizSupport.setActuatorGuiSetting(viz, showThrusterLabels=True)

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
        liveStream=True,
        simTimeStep=1/24.0,
        simTime=3600.0,
        accelFactor=1.0,
        fswTimeStep=0.1
    )
