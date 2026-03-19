from Basilisk.architecture import messaging, sysModel

class ThrusterToMujocoBridge(sysModel.SysModel):
    """Transfer Basilisk thruster output forces into MuJoCo actuators each sim tick."""

    def __init__(self, thruster_sets, thruster_actuator_msgs):
        super().__init__()
        self.ModelTag = "thrusterToMujocoBridge"
        self.thruster_sets = thruster_sets
        self.thruster_actuator_msgs = thruster_actuator_msgs

    def UpdateState(self, CurrentSimNanos):
        for i, thr_set in enumerate(self.thruster_sets):
            for j, actuator_msg in enumerate(self.thruster_actuator_msgs[i]):
                thr_output = thr_set.thrusterOutMsgs[j].read()
                thrust = max(0.0, thr_output.thrustForce)
                actuator_msg.write(messaging.SingleActuatorMsgPayload(input=thrust))
