import os
import matplotlib.pyplot as plt
from Basilisk import __path__
from Basilisk.simulation import spacecraft, thrusterDynamicEffector, extForceTorque
from Basilisk.utilities import (SimulationBaseClass, macros, unitTestSupport, vizSupport)
from Basilisk.architecture import messaging
from Basilisk.simulation import simSynch
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

def run(show_plots=True, liveStream=True, broadcastStream=True, timeStep=0.1, simTime=60.0, accelFactor=1.0):
    simTaskName = "simTask"
    simProcessName = "simProcess"
    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess(simProcessName)
    simulationTimeStep = macros.sec2nano(timeStep)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

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
    ros_bridge.add_bsk_msg_reader('CmdForceBodyMsgPayload', 'cmdForceBodyInMsg', 'cmd_force_body')
    ros_bridge.add_bsk_msg_reader('CmdTorqueBodyMsgPayload', 'cmdTorqueBodyInMsg', 'cmd_torque_body')
    ros_bridge.add_bsk_msg_writer('CmdForceBodyMsgPayload', 'cmdForceBodyOutMsg', 'cmd_force_body')
    ros_bridge.add_bsk_msg_writer('CmdTorqueBodyMsgPayload', 'cmdTorqueBodyOutMsg', 'cmd_torque_body')
    ros_bridge.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    ros_bridge.cmdForceBodyInMsg.subscribeTo(ros_bridge.cmdForceBodyOutMsg)
    ros_bridge.cmdTorqueBodyInMsg.subscribeTo(ros_bridge.cmdTorqueBodyOutMsg)
    scSim.AddModelToTask(simTaskName, ros_bridge)

    # Add external force/torque effector and connect to bridge handler
    extFT = extForceTorque.ExtForceTorque()
    extFT.ModelTag = "externalForceTorque"
    extFT.cmdForceBodyInMsg.subscribeTo(ros_bridge.cmdForceBodyOutMsg)
    extFT.cmdTorqueInMsg.subscribeTo(ros_bridge.cmdTorqueBodyOutMsg)
    scObject.addDynamicEffector(extFT)
    scSim.AddModelToTask(simTaskName, extFT)

    # Thrusters
    # thrusterSet = thrusterDynamicEffector.ThrusterDynamicEffector()
    # scSim.AddModelToTask(simTaskName, thrusterSet)
    # thruster_defs = [
    #     ([0, 0.12, 0], [1.5, 0, 0]),
    #     ([0, 0.12, 0], [-1.5, 0, 0]),
    #     ([0, -0.12, 0], [1.5, 0, 0]),
    #     ([0, -0.12, 0], [-1.5, 0, 0]),
    #     ([0, 0, 0.12], [0, -1.5, 0]),
    #     ([0, 0, 0.12], [0, 1.5, 0]),
    #     ([0, 0, -0.12], [0, -1.5, 0]),
    #     ([0, 0, -0.12], [0, 1.5, 0]),
    #     ([-0.12, 0, 0], [0, 0, -1.5]),
    #     ([-0.12, 0, 0], [0, 0, 1.5]),
    #     ([0.12, 0, 0], [0, 0, -1.5]),
    #     ([0.12, 0, 0], [0, 0, 1.5]),
    # ]
    # for pos, force in thruster_defs:
    #     thrConf = thrusterDynamicEffector.THRSimConfig()
    #     thrConf.thrLoc_B = pos
    #     thrConf.thrDir_B = force
    #     thrConf.MaxThrust = np.linalg.norm(force)
    #     thrConf.steadyIsp = 226.7  # Example value, adjust as needed
    #     thrusterSet.addThruster(thrConf)

    # Add this block to ensure the thruster command message is the correct size and is subscribed
    # thrMsgData = messaging.THRArrayOnTimeCmdMsgPayload()
    # thrMsgData.OnTimeRequest = [0.0] * len(thruster_defs)
    # thrMsg = messaging.THRArrayOnTimeCmdMsg()
    # thrMsg.write(thrMsgData)
    # thrusterSet.cmdsInMsg.subscribeTo(thrMsg)

    # Data logging
    dataLog = scObject.scStateOutMsg.recorder(simulationTimeStep)
    scSim.AddModelToTask(simTaskName, dataLog)

    # Vizard support (optional)
    if vizSupport.vizFound:
        scData = vizInterface.VizSpacecraftData()
        scData.spacecraftName = scObject.ModelTag
        scData.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
        clockSync = simSynch.ClockSynch()
        clockSync.accelFactor = accelFactor
        scSim.AddModelToTask(simTaskName, clockSync)
        viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject,
                                                #   thrEffectorList=thrusterSet,
                                                  thrColors=vizSupport.toRGBA255("white"),
                                                  liveStream=liveStream,
                                                  broadcastStream=broadcastStream)
        viz.settings.keyboardLiveInput = "bxpz"

    scSim.InitializeSimulation()

    # Main simulation loop
    incrementalStopTime = 0
    while incrementalStopTime < macros.sec2nano(simTime):
        incrementalStopTime += simulationTimeStep
        scSim.ConfigureStopTime(incrementalStopTime)
        scSim.ExecuteSimulation()

    # Retrieve and plot results
    posData = dataLog.r_BN_N
    plt.figure()
    plt.plot(dataLog.times() * macros.NANO2SEC, posData)
    plt.xlabel('Time [s]')
    plt.ylabel('Position [m]')
    plt.legend(['x', 'y', 'z'])
    if show_plots:
        plt.show()
    plt.close("all")
    return

if __name__ == "__main__":
    run(
        show_plots=False,
        liveStream=True,
        broadcastStream=True,
        timeStep=0.01,
        simTime=3600.0,
        accelFactor=1.0
    )
