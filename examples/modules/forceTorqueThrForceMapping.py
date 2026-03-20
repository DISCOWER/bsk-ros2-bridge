"""
Pure-Python replacement for the Basilisk C module forceTorqueThrForceMapping.
This module maps body-frame force and torque commands to individual non-negative
thruster force commands using an iterative minimum-norm pseudoinverse algorithm.
Drop-in compatible with the same Basilisk message interfaces.

Note: Due to SWIG limitations in accessing the THRArrayConfigMsgPayload.thrusters
struct array from Python, thruster geometry must be set directly via
setThrusterGeometry() instead of being read from thrConfigInMsg/vehConfigInMsg.
Those message inputs are kept as dummy attributes so existing subscribeTo() calls
do not need to be removed.
"""

import numpy as np
from Basilisk.architecture import sysModel, messaging


class forceTorqueThrForceMapping(sysModel.SysModel):
    """This module maps thruster forces for arbitrary forces and torques"""

    def __init__(self):
        super(forceTorqueThrForceMapping, self).__init__()

        # declare module public variables
        self.rThruster_B = None         # [m]     local copy of the thruster locations
        self.gtThruster_B = None        # []      local copy of the thruster force unit direction vectors

        # declare module private variables
        self.numThrusters = 0           # []      The number of thrusters available on vehicle
        self.CoM_B = np.zeros(3)        # [m]     CoM of the s/c
        self._geometrySet = False       # flag to check if geometry was provided

        # declare module IO interfaces
        self.cmdTorqueInMsg = messaging.CmdTorqueBodyMsgReader()        # (optional) vehicle control (Lr) input message
        self.cmdForceInMsg = messaging.CmdForceBodyMsgReader()          # (optional) vehicle control force input message
        self.thrForceCmdOutMsg = messaging.THRArrayCmdForceMsg()        # thruster force command output message

        # Dummy message inputs kept for subscribeTo() compatibility with existing scenarios.
        # Geometry is read via setThrusterGeometry() instead (SWIG cannot index thrusters array).
        self.thrConfigInMsg = messaging.THRArrayConfigMsgReader()
        self.vehConfigInMsg = messaging.VehicleConfigMsgReader()

    def setThrusterGeometry(self, rThruster_B, gtThruster_B, CoM_B=None):
        """
        Set thruster geometry directly, bypassing the THRArrayConfigMsg.

        Args:
            rThruster_B: list of [x,y,z] thruster positions in body frame [m]
            gtThruster_B: list of [x,y,z] thruster force unit direction vectors
            CoM_B: [x,y,z] center of mass in body frame [m], defaults to [0,0,0]
        """
        self.rThruster_B = np.array(rThruster_B, dtype=float)
        self.gtThruster_B = np.array(gtThruster_B, dtype=float)
        self.numThrusters = len(rThruster_B)
        if CoM_B is not None:
            self.CoM_B = np.array(CoM_B, dtype=float)
        self._geometrySet = True

    def setThrusterGeometryFromDefs(self, thruster_defs, CoM_B=None):
        """
        Set thruster geometry from a list of (position, direction) tuples.
        Convenience wrapper for the format used in scenario scripts.

        Args:
            thruster_defs: list of ([x,y,z], [dx,dy,dz]) tuples
            CoM_B: [x,y,z] center of mass in body frame [m], defaults to [0,0,0]
        """
        self.setThrusterGeometry(
            rThruster_B=[loc for loc, d in thruster_defs],
            gtThruster_B=[d for loc, d in thruster_defs],
            CoM_B=CoM_B,
        )

    def Reset(self, CurrentSimNanos):
        """This method performs a complete reset of the module. Local module variables that retain
        time varying states between function calls are reset to their default values.
        Check if required input messages are connected."""

        if not self._geometrySet:
            self.bskLogger.bskLog(sysModel.BSK_ERROR,
                        "Error: forceTorqueThrForceMapping: call setThrusterGeometry() before running the simulation.")
            return

        # zero the thruster force command output message
        thrForceCmdOutMsgBuffer = messaging.THRArrayCmdForceMsgPayload()
        self.thrForceCmdOutMsg.write(thrForceCmdOutMsgBuffer, CurrentSimNanos, self.moduleID)

    def UpdateState(self, CurrentSimNanos):
        """Map commanded body force/torque to individual thruster forces."""

        # always zero the output message buffers before assigning values
        thrForceCmdOutMsgBuffer = messaging.THRArrayCmdForceMsgPayload()

        # Check if torque message is linked and read, zero out if not
        if self.cmdTorqueInMsg.isLinked():
            cmdTorqueInMsgBuffer = self.cmdTorqueInMsg()
            torqueRequestBody = np.array([cmdTorqueInMsgBuffer.torqueRequestBody[j] for j in range(3)])
        else:
            torqueRequestBody = np.zeros(3)

        # Check if force message is linked and read, zero out if not
        if self.cmdForceInMsg.isLinked():
            cmdForceInMsgBuffer = self.cmdForceInMsg()
            forceRequestBody = np.array([cmdForceInMsgBuffer.forceRequestBody[j] for j in range(3)])
        else:
            forceRequestBody = np.zeros(3)

        # Initialize variables
        numThrusters = self.numThrusters
        force_B = np.zeros(numThrusters)

        # Create the torque and force vector
        forceTorque_B = np.concatenate([torqueRequestBody, forceRequestBody])  # (6,)

        # Compute thruster locations relative to COM
        rThrusterRelCOM_B = self.rThruster_B - self.CoM_B  # (N, 3)

        # Fill DG with thruster directions and moment arms
        DG = np.zeros((6, numThrusters))
        for i in range(numThrusters):
            # Compute moment arm and fill in
            rCrossGt = np.cross(rThrusterRelCOM_B[i], self.gtThruster_B[i])
            DG[:3, i] = rCrossGt

            # Fill in control axes
            DG[3:, i] = self.gtThruster_B[i]

        # Check DG for zero rows
        zeroRows = np.zeros(6, dtype=bool)
        numZeroes = 0
        for j in range(6):
            if np.allclose(DG[j, :], 0.0, atol=1e-7):
                zeroRows[j] = True
                numZeroes += 1

        # Create reduced force/torque vector (remove elements corresponding to zero rows)
        forceTorque_reduced = forceTorque_B[~zeroRows]

        # Iterative algorithm to handle negative forces by removing columns
        activeThrusterMask = np.ones(numThrusters, dtype=bool)
        numActiveThrusters = numThrusters

        for iteration in range(numThrusters):

            # Create reduced DG matrix with only active thrusters and non-zero rows
            DG_reduced = DG[np.ix_(~zeroRows, activeThrusterMask)]  # (m, numActive)

            # Compute pseudoinverse of reduced matrix and solve for active thruster forces
            activeForces = np.linalg.lstsq(DG_reduced, forceTorque_reduced, rcond=None)[0]

            # Map back to full thruster array
            force_B[:] = 0.0
            force_B[activeThrusterMask] = activeForces
            hasNegativeForces = np.any(force_B < -1e-10)

            # End iteration when solution has no negative forces
            if not hasNegativeForces:
                break

            # Deactivate the thruster with most negative force
            mostNegativeIndex = np.argmin(force_B)
            mostNegativeForce = force_B[mostNegativeIndex]

            if mostNegativeForce < -1e-10:
                activeThrusterMask[mostNegativeIndex] = False
                numActiveThrusters -= 1

                # If all thrusters are deactivated, set forces to zero, no solution found
                if numActiveThrusters <= 0:
                    force_B[:] = 0.0
                    break

        # Ensure non-negative thrust
        forceSubtracted_B = np.maximum(force_B, 0.0)

        # Write to the output messages
        thrForceCmdOutMsgBuffer.thrForce = forceSubtracted_B.tolist()
        self.thrForceCmdOutMsg.write(thrForceCmdOutMsgBuffer, CurrentSimNanos, self.moduleID)