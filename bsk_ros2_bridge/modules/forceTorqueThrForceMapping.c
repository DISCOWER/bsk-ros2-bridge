/*
 ISC License

 Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

*/

#include "fswAlgorithms/effectorInterfaces/forceTorqueThrForceMapping/forceTorqueThrForceMapping.h"
#include "architecture/utilities/linearAlgebra.h"
#include "string.h"

/*!
    This method initializes the output messages for this module.

 @param configData The configuration data associated with this module
 @param moduleID The module identifier
 */
void SelfInit_forceTorqueThrForceMapping(forceTorqueThrForceMappingConfig* configData, int64_t moduleID)
{
    THRArrayCmdForceMsg_C_init(&configData->thrForceCmdOutMsg);
}

/*! This method performs a complete reset of the module.  Local module variables that retain
    time varying states between function calls are reset to their default values.
    Check if required input messages are connected.

 @param configData The configuration data associated with the module
 @param callTime [ns] time the method is called
 @param moduleID The module identifier
*/
void Reset_forceTorqueThrForceMapping(forceTorqueThrForceMappingConfig *configData, uint64_t callTime, int64_t moduleID)
{
    if (!THRArrayConfigMsg_C_isLinked(&configData->thrConfigInMsg)) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: forceTorqueThrForceMapping.thrConfigInMsg was not connected.");
    }
    if (!VehicleConfigMsg_C_isLinked(&configData->vehConfigInMsg)) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: forceTorqueThrForceMapping.vehConfigInMsg was not connected.");
    }

    VehicleConfigMsgPayload vehConfigInMsgBuffer;       //!< local copy of message buffer
    THRArrayConfigMsgPayload thrConfigInMsgBuffer;      //!< local copy of message buffer
    THRArrayCmdForceMsgPayload thrForceCmdOutMsgBuffer; //!< local copy of message buffer

    //!< read the rest of the input messages
    thrConfigInMsgBuffer = THRArrayConfigMsg_C_read(&configData->thrConfigInMsg);
    vehConfigInMsgBuffer = VehicleConfigMsg_C_read(&configData->vehConfigInMsg);

    /*! - copy the thruster position and thruster force heading information into the module configuration data */
    configData->numThrusters = (uint32_t) thrConfigInMsgBuffer.numThrusters;
    v3Copy(vehConfigInMsgBuffer.CoM_B, configData->CoM_B);
    if (configData->numThrusters > MAX_EFF_CNT) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: forceTorqueThrForceMapping thruster configuration input message has a number of thrusters that is larger than MAX_EFF_CNT");
    }

    /*! - copy the thruster position and thruster force heading information into the module configuration data */
    for(uint32_t i = 0; i < configData->numThrusters; i++)
    {
        v3Copy(thrConfigInMsgBuffer.thrusters[i].rThrust_B, configData->rThruster_B[i]);
        v3Copy(thrConfigInMsgBuffer.thrusters[i].tHatThrust_B, configData->gtThruster_B[i]);
        if(thrConfigInMsgBuffer.thrusters[i].maxThrust <= 0.0){
            _bskLog(configData->bskLogger, BSK_ERROR, "Error: forceTorqueThrForceMapping: A configured thruster has a non-sensible saturation limit of <= 0 N!");
        }
    }

    /* zero the thruster force command output message */
    thrForceCmdOutMsgBuffer = THRArrayCmdForceMsg_C_zeroMsgPayload();
    THRArrayCmdForceMsg_C_write(&thrForceCmdOutMsgBuffer, &configData->thrForceCmdOutMsg, moduleID, callTime);
}

/*! Add a description of what this main Update() routine does for this module

 @param configData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The module identifier
*/
void Update_forceTorqueThrForceMapping(forceTorqueThrForceMappingConfig *configData, uint64_t callTime, int64_t moduleID)
{
    CmdTorqueBodyMsgPayload cmdTorqueInMsgBuffer;       //!< local copy of message buffer
    CmdForceBodyMsgPayload cmdForceInMsgBuffer;         //!< local copy of message buffer
    THRArrayCmdForceMsgPayload thrForceCmdOutMsgBuffer; //!< local copy of message buffer

    // always zero the output message buffers before assigning values
    thrForceCmdOutMsgBuffer = THRArrayCmdForceMsg_C_zeroMsgPayload();

    /* Check if torque message is linked and read, zero out if not*/
    if (CmdTorqueBodyMsg_C_isLinked(&configData->cmdTorqueInMsg)) {
        cmdTorqueInMsgBuffer = CmdTorqueBodyMsg_C_read(&configData->cmdTorqueInMsg);
    } else{
        cmdTorqueInMsgBuffer = CmdTorqueBodyMsg_C_zeroMsgPayload();
    }

    /* Check if force message is linked and read, zero out if not*/
    if (CmdForceBodyMsg_C_isLinked(&configData->cmdForceInMsg)) {
        cmdForceInMsgBuffer = CmdForceBodyMsg_C_read(&configData->cmdForceInMsg);
    } else{
        cmdForceInMsgBuffer = CmdForceBodyMsg_C_zeroMsgPayload();
    }

    /* Initialize variables */
    double DG[6][MAX_EFF_CNT];
    double rThrusterRelCOM_B[MAX_EFF_CNT][3];
    double rCrossGt[3];
    double zeroVector[MAX_EFF_CNT];
    uint32_t zeroRows[6];
    uint32_t numZeroes;
    double force_B[MAX_EFF_CNT];
    double forceTorque_B[6];
    double forceSubtracted_B[MAX_EFF_CNT];
    vSetZero(force_B, (size_t)MAX_EFF_CNT);

    for (uint32_t i = 0; i < 6; i++) {
        for (uint32_t j = 0; j < MAX_EFF_CNT; j++) {
            DG[i][j] = 0.0;
        }
    }

    /* Create the torque and force vector */
    for (uint32_t i = 0; i < 3; i++) {
        forceTorque_B[i] = cmdTorqueInMsgBuffer.torqueRequestBody[i];
        forceTorque_B[i+3] = cmdForceInMsgBuffer.forceRequestBody[i];
    }

    /* - compute thruster locations relative to COM */
    for (uint32_t i = 0; i<configData->numThrusters; i++) {
        v3Subtract(configData->rThruster_B[i], configData->CoM_B, rThrusterRelCOM_B[i]);
    }

    /* Fill DG with thruster directions and moment arms */
    for (uint32_t i = 0; i < configData->numThrusters; i++) {
        /* Compute moment arm and fill in */
        v3Cross(rThrusterRelCOM_B[i], configData->gtThruster_B[i], rCrossGt);
        for(uint32_t j = 0; j < 3; j++) {
            DG[j][i] = rCrossGt[j];
        }

        /* Fill in control axes */
        for(uint32_t j = 0; j < 3; j++) {
            DG[j+3][i] = configData->gtThruster_B[i][j];
        }
    }

    /* Check DG for zero rows */
    vSetZero(zeroVector, configData->numThrusters);
    numZeroes = 0;
    for(uint32_t j = 0; j < 6; j++) {
        if (vIsEqual(zeroVector, configData->numThrusters, DG[j], 0.0000001)) {
            zeroRows[j] = 1;
            numZeroes += 1;
        } else {
            zeroRows[j] = 0;
        }
    }

    /* Create reduced force/torque vector (remove elements corresponding to zero rows) */
    double forceTorque_reduced[6];
    uint32_t targetElement = 0;
    for(uint32_t i = 0; i < 6; i++) {
        if (!zeroRows[i]) {
            forceTorque_reduced[targetElement] = forceTorque_B[i];
            targetElement++;
        }
    }

    /* Iterative algorithm to handle negative forces by removing columns */
    uint32_t activeThrusterMask[MAX_EFF_CNT];
    uint32_t numActiveThrusters = configData->numThrusters;
    double mostNegativeForce;
    uint32_t mostNegativeIndex;

    /* Declare arrays with fixed size for MSVC compatibility */
    double DG_reduced[6 * MAX_EFF_CNT];
    double DG_reduced_inv[MAX_EFF_CNT * 6];
    double activeForces[MAX_EFF_CNT];

    /* Initialize all thrusters as active */
    for (uint32_t i = 0; i < configData->numThrusters; i++) {
        activeThrusterMask[i] = 1;
    }

    for (uint32_t iteration = 0; iteration < configData->numThrusters; iteration++) {

        /* Create reduced DG matrix with only active thrusters and non-zero rows */
        vSetZero(DG_reduced, (size_t)(6 - numZeroes) * numActiveThrusters);

        uint32_t activeColIndex = 0;
        for (uint32_t j = 0; j < configData->numThrusters; j++) {
            if (activeThrusterMask[j]) {
                uint32_t reducedRowIndex = 0;
                for (uint32_t i = 0; i < 6; i++) {
                    if (!zeroRows[i]) {
                        DG_reduced[MXINDEX(numActiveThrusters, reducedRowIndex, activeColIndex)] = DG[i][j];
                        reducedRowIndex++;
                    }
                }
                activeColIndex++;
            }
        }

        /* Compute pseudoinverse of reduced matrix */
        mMinimumNormInverse(DG_reduced, (size_t)(6 - numZeroes), (size_t)numActiveThrusters, DG_reduced_inv);

        /* Solve for active thruster forces */
        mMultV(DG_reduced_inv, (size_t)numActiveThrusters, (size_t)(6 - numZeroes), forceTorque_reduced, activeForces);

        /* Map back to full thruster array */
        activeColIndex = 0;
        uint32_t hasNegativeForces = 0;
        for (uint32_t j = 0; j < configData->numThrusters; j++) {
            if (activeThrusterMask[j]) {
                force_B[j] = activeForces[activeColIndex];
                if (force_B[j] < -1e-10) {
                    hasNegativeForces = 1;
                }
                activeColIndex++;
            } else {
                force_B[j] = 0.0;
            }
        }

        /* End iteration when solution has no negative forces */
        if (!hasNegativeForces) {
            break;
        }

        /* Deactivate the thruster with most negative force */
        mostNegativeForce = 0.0;
        mostNegativeIndex = 0;
        for (uint32_t j = 0; j < configData->numThrusters; j++) {
            if (activeThrusterMask[j] && force_B[j] < mostNegativeForce) {
                mostNegativeForce = force_B[j];
                mostNegativeIndex = j;
            }
        }

        if (mostNegativeForce < -1e-10) {
            activeThrusterMask[mostNegativeIndex] = 0;
            numActiveThrusters--;

            /* If all thrusters are deactivated, set forces to zero, no solution found */
            if (numActiveThrusters <= 0) {
                vSetZero(force_B, configData->numThrusters);
                break;
            }
        }
    }

    /* Ensure non-negative thrust */
    for (uint32_t i = 0; i < configData->numThrusters; i++) {
        forceSubtracted_B[i] = (force_B[i] > 0.0) ? force_B[i] : 0.0;
    }

    /* Write to the output messages */
    vCopy(forceSubtracted_B, configData->numThrusters, thrForceCmdOutMsgBuffer.thrForce);
    THRArrayCmdForceMsg_C_write(&thrForceCmdOutMsgBuffer, &configData->thrForceCmdOutMsg, moduleID, callTime);
}
