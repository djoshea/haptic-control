#include <stdlib.h>
#include "chai3d.h"
#include "utils.h"
#include "network.h"
#include "haptic.h"
#include "environment.h"

#include "stateMachine.h"

//////////// DECLARED VARIABLES ////

extern ChaiData chai;
extern HapticState haptic;

#define MAX_HAPTIC_COMMANDS 5
HapticCommand hapticCommandBuffer[MAX_HAPTIC_COMMANDS];
cThread* stateMachineThread;
StateMachineState state;
bool stateMachineUp = false;

unsigned stateMachineMsg = -1; // handle to opengl message label that we create and can update

bool stateMachineIsUp() {
    return stateMachineUp;
}

void stateMachineStart() {
    if(!stateMachineUp) {
        stateMachineUp = true;
        stateMachineThread = new cThread();
        stateMachineThread->set(stateMachineUpdate, CHAI_THREAD_PRIORITY_HAPTICS);
    }
}

void stateMachineUpdate(void) {

    printf("StateMachine: waiting\n");
    if(!waitForGraphicsReady()) {
        printf("StateMachine: aborting\n");
        stateMachineUp = false;
        return;
    }
    if(!waitForHapticsReady()) {
        printf("StateMachine: aborting\n");
        stateMachineUp = false;
        return;
    }

    printf("StateMachine: starting\n");

    stateMachineMsg = addMessageLabel();
    setMessageColor(stateMachineMsg, cColorf(0.4, 1.0, 0.4));

    int bytesRead;
    int bufferSize = MAX_PACKET_LENGTH;
    uint8_t rawPacket[MAX_PACKET_LENGTH];
    double currentTime, timeIntoPerturbationMs;
    int nCommandsReceived = 0;
    HapticCommand hapticCommand;

    char strMessage[200];
    bool displayStatus = false;

    chai.simClockStateMachine.reset();
    chai.simClockStateMachine.start(true);

    // retract handle to start
    state.isRetracted = true;
    hapticRetractHandle();
    snprintf(strMessage, 200, "StateMachine: Initialized with retracted handle");
    //hapticResume();

    // build a test target

    /*
    hapticCommand.targetPoints.clear();
    cVector3d pt;
    pt.z = 0;
    pt.x = -25; pt.y = 50;
    hapticCommand.targetPoints.push_back(pt);
    pt.x = 25; pt.y = 49;
    hapticCommand.targetPoints.push_back(pt);
    hapticCreateTarget(true, true, hapticCommand.targetPoints);
    */

    // build a test obstacle
    /*
    hapticCommand.obstaclePoints.clear();
    hapticCommand.obstacleCollisionPermitted = false;
    cVector3d pt;
    pt.z = 0;
    pt.x = -25; pt.y = -120;
    hapticCommand.obstaclePoints.push_back(pt);
    pt.x = -25; pt.y = -75;
    hapticCommand.obstaclePoints.push_back(pt);
    pt.x = -75; pt.y = 65;
    hapticCommand.obstaclePoints.push_back(pt);
    pt.x = -150; pt.y = 65;
    hapticCommand.obstaclePoints.push_back(pt);
    pt.x = -150; pt.y = -150;
    hapticCommand.obstaclePoints.push_back(pt);
    pt.x = 150; pt.y = -150;
    hapticCommand.obstaclePoints.push_back(pt);
    pt.x = 150; pt.y = 65;
    hapticCommand.obstaclePoints.push_back(pt);
    pt.x = -25; pt.y = 65;
    hapticCommand.obstaclePoints.push_back(pt);
    pt.x = 25; pt.y = -75;
    hapticCommand.obstaclePoints.push_back(pt);
    pt.x = 25; pt.y = -120;
    hapticCommand.obstaclePoints.push_back(pt);

    hapticCreateObstacle(hapticCommand.obstaclePoints,
        hapticCommand.obstacleCollisionPermitted);
    snprintf(strMessage, 200, "StateMachine: Initialized with test obstacle");
    */

    printf("%s\n", strMessage);
    updateMessage(stateMachineMsg, strMessage);

    while(!chai.simulationFinished) {
        currentTime = chai.simClockNetwork.getCurrentTimeSeconds();
        memset(rawPacket, 0, bufferSize);
        bytesRead = networkReceive(rawPacket, bufferSize);

        if(bytesRead > 0)
            nCommandsReceived = parseBinPacket(rawPacket, bytesRead, hapticCommandBuffer, MAX_HAPTIC_COMMANDS);
        else
            nCommandsReceived = 0;

        // handle linear constraint
        for(int iCommand = 0; iCommand < nCommandsReceived; iCommand++) {
            hapticCommand = hapticCommandBuffer[iCommand];

            if (state.isRetracted) {
                if(hapticCommand.commandId == HAPTIC_COMMAND_RESUME) {
                    snprintf(strMessage, 200, "StateMachine: Resuming, returning handle");
                    displayStatus = true;
                    hapticResume();
                    state.isRetracted = false;
                } else {
                    printf("StateMachine: Ignoring command received while retracted\n");
                }
            } else {

                //printf("processing command %d\n", hapticCommand.commandId);
                switch(hapticCommand.commandId) {
                    case HAPTIC_COMMAND_PULSEPERTURBATION:
                        hapticTriggerPerturbationPulse(hapticCommand.perturbDuration, hapticCommand.perturbDirection);
                        snprintf(strMessage, 200, "StateMachine: Perturbation pulse for %.0f ms towards (%.2f, %.2f, %.2f)",
                                hapticCommand.perturbDuration, hapticCommand.perturbDirection.x,
                                hapticCommand.perturbDirection.y, hapticCommand.perturbDirection.z);
                        displayStatus = true;
                        break;

                    case HAPTIC_COMMAND_ABORTPERTURBATION:
                        snprintf(strMessage, 200, "StateMachine: Abort perturbation");
                        hapticAbortPerturbation();
                        break;

            /*        case HAPTIC_COMMAND_CONSTRAIN_LINETOTARGET:
                        snprintf(strMessage, 200, "StateMachine: Constrain to target command - Target (%.0f, %.0f, %.0f)",
                            hapticCommand.linearConstraintEndRig.x,
                            hapticCommand.linearConstraintEndRig.y,
                            hapticCommand.linearConstraintEndRig.z);

                        displayStatus = true;

                        //hapticSetLinearConstraint(&hapticCommand);
                        break;

                    case HAPTIC_COMMAND_CONSTRAIN_RELEASE:
                        //hapticReleaseLinearConstraint(&hapticCommand);
                        snprintf(strMessage, 200, "StateMachine: Release linear constraint command");
                        break;
            */
                    case HAPTIC_COMMAND_MOVETOPOINT:
                        snprintf(strMessage, 200, "StateMachine: Received move to point command (%.0f, %.0f, %.0f)",
                            hapticCommand.moveEndpointRig.x, hapticCommand.moveEndpointRig.y, hapticCommand.moveEndpointRig.z);
                        displayStatus = true;
                        hapticMoveToPoint(hapticCommand.moveEndpointRig);
                        break;

                    case HAPTIC_COMMAND_MOVETOHOLDNOTSEEN:
                        snprintf(strMessage, 200, "StateMachine: Received hold not seen command");
                        displayStatus = true;
                        hapticMoveToHandNotSeen();
                        break;

                    case HAPTIC_COMMAND_MOVEABORT:
                        snprintf(strMessage, 200, "StateMachine: Abort move to point command");
                        hapticMoveAbort();
                        break;

                    case HAPTIC_COMMAND_CREATEOBSTACLE:
                        snprintf(strMessage, 200, "StateMachine: Create obstacle with %u points",
                                (unsigned int)hapticCommand.obstaclePoints.size());
                        displayStatus = true;
                        hapticCreateObstacle(hapticCommand.obstaclePoints,
                            hapticCommand.obstacleCollisionPermitted);
                        break;

                    case HAPTIC_COMMAND_CLEAROBSTACLES:
                        snprintf(strMessage, 200, "StateMachine: Clear obstacles");
                        hapticClearObstacles();
                        break;

                    case HAPTIC_COMMAND_CONSTRAINTOPOINT:
                        snprintf(strMessage, 200, "StateMachine: Ignoring constrain to current location");
                        //displayStatus = true;

                        //hapticConstrainToCurrentPoint();
                        break;

                    case HAPTIC_COMMAND_CONSTRAINABORT:
                        snprintf(strMessage, 200, "StateMachine: Aborting constrain to current location");
                        hapticConstrainAbort();
                        break;

                    case HAPTIC_COMMAND_SUPPORTATPOINT:
                        snprintf(strMessage, 200, "StateMachine: Support ledge at (%.0f, %.0f, %.0f)",
                            hapticCommand.supportPoint.x,
                            hapticCommand.supportPoint.y,
                            hapticCommand.supportPoint.z);
                        displayStatus = true;
                        hapticSupportAtPoint(hapticCommand.supportPoint);
                        break;

                    case HAPTIC_COMMAND_SUPPORTABORT:
                        snprintf(strMessage, 200, "StateMachine: Removing support ledge");
                        hapticAbortSupportAtPoint();
                        break;

                    case HAPTIC_COMMAND_RETRACT:
                        snprintf(strMessage, 200, "StateMachine: Retracting handle");
                        displayStatus = true;

                        state.isRetracted = true;
                        hapticRetractHandle();
                        break;

                    case HAPTIC_COMMAND_CREATEGAP:
                        snprintf(strMessage, 200, "StateMachine: Create gap with %u, %u points",
                                (unsigned int)hapticCommand.gapLeftPoints.size(),
                                (unsigned int)hapticCommand.gapRightPoints.size());
                        displayStatus = true;

                        hapticCreateGap(hapticCommand.hasGapLeft, hapticCommand.gapLeftPoints,
                                hapticCommand.hasGapRight, hapticCommand.gapRightPoints,
                                hapticCommand.gapCollisionPermitted);
                        break;

                    case HAPTIC_COMMAND_CREATETARGETS:
                        snprintf(strMessage, 200, "StateMachine: Create %d targets each with %u points",
                                (unsigned int)hapticCommand.nTargets,
                                (unsigned int)hapticCommand.targetPoints[0].size());
                        displayStatus = true;

                        // print out coordinates for debugging
                        /*
                        for(unsigned i = 0; i < hapticCommand.nTargets; i++)
                            printf("p1 (%g, %g), p2 (%g, %g)\n",
                                hapticCommand.targetPoints[i][0].x, hapticCommand.targetPoints[i][0].y,
                                hapticCommand.targetPoints[i][1].x, hapticCommand.targetPoints[i][1].y);
                        */

                        // double checking!
                        if(hapticCommand.nTargets > HAPTIC_MAX_TARGETS)
                            hapticCommand.nTargets = HAPTIC_MAX_TARGETS;

                        hapticClearTargets();
                        for(unsigned i = 0; i < hapticCommand.nTargets; i++)
                            hapticCreateTarget(hapticCommand.targetIsViscous, hapticCommand.targetIsPlanar,
                                hapticCommand.targetPoints[i]);
                        break;

                    case HAPTIC_COMMAND_CLEARTARGETS:
                        snprintf(strMessage, 200, "StateMachine: Clear targets");
                        hapticClearTargets();
                        break;

                    case HAPTIC_COMMAND_CREATECONSTRAINTS:
                        snprintf(strMessage, 200, "StateMachine: Create %d constraints each with %u points",
                                (unsigned int)hapticCommand.nConstraints,
                                (unsigned int)hapticCommand.constraintPoints[0].size());
                        displayStatus = true;

                        // double checking!
                        if(hapticCommand.nConstraints > HAPTIC_MAX_TARGETS)
                            hapticCommand.nConstraints = HAPTIC_MAX_TARGETS;

                        hapticClearConstraints();

                        for(unsigned i = 0; i < hapticCommand.nConstraints; i++)
                            hapticCreateConstraint(hapticCommand.constraintPoints[i]);

                        break;

                    case HAPTIC_COMMAND_CLEARCONSTRAINTS:
                        snprintf(strMessage, 200, "StateMachine: Clear constraints");
                        hapticClearConstraints();
                        break;
                }
            }

            printf("%s\n", strMessage);
            if(displayStatus)
                updateMessage(stateMachineMsg, strMessage);

        }

        //cSleep(1);
    }

    stateMachineUp = false;
}

// binary packet looks like:
// prefix: 2 x char "#h"
// version: uint16
// commandId: uint16
// <remainder depends on commandId>
//
// pulsePerturbation commandId == 2
//   directionX : double
//   directionY : double
//   directionZ : double
//   magnitude (N): double
//   duration (ms) : double

// parse binary packet received at haptic, and store the relevant info in
// command
int parseBinPacket(const uint8_t* rawPacket, int bytesRead, HapticCommand* commandBuffer, int maxCommands){
    const uint8_t* pBuf = rawPacket;
    uint16_t storeNameLength;
    uint16_t pVersion;
    uint8_t collisionPermitted = 0, targetMode = 0;
    uint16_t nPoints = 0;
    bool commandReceived = false;
    int nCommandsReceived = 0;
    HapticCommand* command = commandBuffer;

    if(bytesRead <= 0)
        return false;

    if (bytesRead < 6) {
        // ERROR
        printf("Packet too short at %d bytes!\n", bytesRead);
        return false;
    }

    // check packet header string matches (otherwise could be garbage packet)
    if(strncmp((char*)pBuf, HAPTIC_PACKET_PREFIX, strlen(HAPTIC_PACKET_PREFIX)) != 0)
    {
        // packet header does not match, discard
        printf("Packet prefix doesnt match\n");
        return false;
    }

    pBuf = pBuf + strlen(HAPTIC_PACKET_PREFIX);

    // store the packet version
    STORE_UINT16(pBuf, command->version);

    while(pBuf - rawPacket < bytesRead + 2 && nCommandsReceived < maxCommands) {
        //printf("processing command with id %d\n", command->commandId);
        // store in ith slot
        command = commandBuffer + nCommandsReceived;

        // store the command id
        STORE_UINT16(pBuf, command->commandId);

        commandReceived = false;
        if (command->commandId == 0)
            break;

        switch(command->commandId) {
            case HAPTIC_COMMAND_PULSEPERTURBATION:
                STORE_UINT16(pBuf, command->perturbType);
                STORE_DOUBLE(pBuf, command->perturbDuration);
                STORE_DOUBLE(pBuf, command->perturbDirection.x);
                STORE_DOUBLE(pBuf, command->perturbDirection.y);
                STORE_DOUBLE(pBuf, command->perturbDirection.z);
                commandReceived = true;

                break;

            case HAPTIC_COMMAND_ABORTPERTURBATION:
                commandReceived = true;
                break;

            case HAPTIC_COMMAND_CONSTRAIN_LINETOTARGET:
                break;
                STORE_DOUBLE(pBuf, command->linearConstraintEndRig.x);
                STORE_DOUBLE(pBuf, command->linearConstraintEndRig.y);
                command->linearConstraintEndRig.z = 0;

                command->linearConstraintActive = true;
                commandReceived = true;
                break;

            case HAPTIC_COMMAND_CONSTRAIN_RELEASE:
                break;
                command->linearConstraintActive = false;
                commandReceived = true;

                break;

            case HAPTIC_COMMAND_MOVETOPOINT:
                command->moveActive = true;
                STORE_DOUBLE(pBuf, command->moveEndpointRig.x);
                STORE_DOUBLE(pBuf, command->moveEndpointRig.y);
                command->moveEndpointRig.z = -1;
                commandReceived = true;

                break;

            case HAPTIC_COMMAND_MOVETOHOLDNOTSEEN:
                command->moveActive = true;
                STORE_DOUBLE(pBuf, command->moveEndpointRig.x);
                STORE_DOUBLE(pBuf, command->moveEndpointRig.y);
                command->moveEndpointRig.z = -1; // this may be overriden
                commandReceived = true;

                break;

            case HAPTIC_COMMAND_MOVEABORT:
                command->moveActive = false;
                commandReceived = true;
                break;

            case HAPTIC_COMMAND_CLEAROBSTACLES:
                commandReceived = true;
                break;

            case HAPTIC_COMMAND_CREATEOBSTACLE:
                // build an obstacle with the specified number of points
                command->obstaclePoints.clear();

                // flag indicating whether collisions are permitted
                STORE_UINT8(pBuf, collisionPermitted);
                command->obstacleCollisionPermitted = collisionPermitted > 0;

                STORE_UINT16(pBuf, nPoints);

                if(nPoints > MAX_COMMAND_POINTS) {
                    fprintf(stderr, "StateMachine: Too many points received in haptic command\n");
                    commandReceived = false;
                    break;
                }

                for(unsigned i = 0; i < nPoints; i++) {
                    cVector3d pt;
                    STORE_DOUBLE(pBuf, pt.x);
                    STORE_DOUBLE(pBuf, pt.y);
                    pt.z = 0;
                    command->obstaclePoints.push_back(pt);
                }

                commandReceived = true;

                break;

            case HAPTIC_COMMAND_CREATEGAP:
                // build an left and right wall with the specified number of points
                command->gapLeftPoints.clear();
                command->gapRightPoints.clear();

                // flag indicating whether collisions are permitted
                STORE_UINT8(pBuf, collisionPermitted);
                command->gapCollisionPermitted = collisionPermitted > 0;

                uint8_t hasGapLeft, hasGapRight;
                uint16_t nPointsLeft, nPointsRight;

                // store left points
                STORE_UINT8(pBuf, hasGapLeft);
                STORE_UINT16(pBuf, nPointsLeft);
                if(nPointsLeft > MAX_COMMAND_POINTS) {
                    fprintf(stderr, "StateMachine: Too many points received in haptic command\n");
                    commandReceived = false;
                    break;
                }
                for(unsigned i = 0; i < nPointsLeft; i++) {
                    cVector3d pt;
                    STORE_DOUBLE(pBuf, pt.x);
                    STORE_DOUBLE(pBuf, pt.y);
                    pt.z = 0;
                    command->gapLeftPoints.push_back(pt);
                }
                command->hasGapLeft = hasGapLeft > 0;

                // store right points
                STORE_UINT8(pBuf, hasGapRight);
                STORE_UINT16(pBuf, nPointsRight);
                if(nPointsRight > MAX_COMMAND_POINTS) {
                    fprintf(stderr, "StateMachine: Too many points received in haptic command\n");
                    commandReceived = false;
                    break;
                }
                for(unsigned i = 0; i < nPointsRight; i++) {
                    cVector3d pt;
                    STORE_DOUBLE(pBuf, pt.x);
                    STORE_DOUBLE(pBuf, pt.y);
                    pt.z = 0;
                    command->gapRightPoints.push_back(pt);
                }
                command->hasGapRight = hasGapRight > 0;

                commandReceived = true;
                break;

            case HAPTIC_COMMAND_CONSTRAINTOPOINT:
                // keep the haptic at the present location
                commandReceived = true;
                break;

            case HAPTIC_COMMAND_CONSTRAINABORT:
                commandReceived = true;
                break;

            case HAPTIC_COMMAND_SUPPORTATPOINT:
                STORE_DOUBLE(pBuf, command->supportPoint.x);
                STORE_DOUBLE(pBuf, command->supportPoint.y);
                command->supportPoint.z = 0;
                commandReceived = true;
                break;

            case HAPTIC_COMMAND_SUPPORTABORT:
                commandReceived = true;
                break;

            case HAPTIC_COMMAND_RETRACT:
                commandReceived = true;
                break;

            case HAPTIC_COMMAND_RESUME:
                commandReceived = true;
                break;

            case HAPTIC_COMMAND_CLEARTARGETS:
                commandReceived = true;
                break;

            case HAPTIC_COMMAND_CREATETARGETS:
                // build an target with the specified number of points

                // flag indicating whether target presents viscosity or acts like a backboard
                STORE_UINT8(pBuf, targetMode);
                command->targetIsViscous = targetMode == 1;
                command->targetIsPlanar = targetMode == 2;

                STORE_UINT16(pBuf, command->nTargets);
                if(command->nTargets > HAPTIC_MAX_TARGETS) {
                    fprintf(stderr, "StateMachine: Too many targets received in haptic command\n");
                    commandReceived = false;
                    break;
                }
                if(command->nTargets == 0) {
                    fprintf(stderr, "StateMachine: 0 targets received in haptic command\n");
                    commandReceived = false;
                    break;
                }

                STORE_UINT16(pBuf, nPoints);
                if(nPoints > MAX_COMMAND_POINTS) {
                    fprintf(stderr, "StateMachine: Too many points received in haptic command\n");
                    commandReceived = false;
                    break;
                }
                if(nPoints == 0) {
                    fprintf(stderr, "StateMachine: 0 points received in haptic command\n");
                    commandReceived = false;
                    break;
                }

                for(unsigned t = 0; t < command->nTargets; t++) {
                    command->targetPoints[t].clear();
                    for(unsigned i = 0; i < nPoints; i++) {
                        cVector3d pt;
                        STORE_DOUBLE(pBuf, pt.x);
                        STORE_DOUBLE(pBuf, pt.y);
                        pt.z = 0;
                        command->targetPoints[t].push_back(pt);
                    }
                }

                commandReceived = true;

                break;

            case HAPTIC_COMMAND_CREATECONSTRAINTS:
                // build multiple constraints with the specified number of points

                STORE_UINT16(pBuf, command->nConstraints);
                if(command->nConstraints > HAPTIC_MAX_CONSTRAINTS) {
                    fprintf(stderr, "StateMachine: Too many constraints received in haptic command\n");
                    commandReceived = false;
                    break;
                }
                if(command->nConstraints == 0) {
                    fprintf(stderr, "StateMachine: 0 constraints received in haptic command\n");
                    commandReceived = false;
                    break;
                }

                STORE_UINT16(pBuf, nPoints);
                if(nPoints > MAX_COMMAND_POINTS) {
                    fprintf(stderr, "StateMachine: Too many points received in haptic command\n");
                    commandReceived = false;
                    break;
                }
                if(nPoints == 0) {
                    fprintf(stderr, "StateMachine: 0 points received in haptic command\n");
                    commandReceived = false;
                    break;
                }

                for(unsigned t = 0; t < command->nConstraints; t++) {
                    command->constraintPoints[t].clear();
                    for(unsigned i = 0; i < nPoints; i++) {
                        cVector3d pt;
                        STORE_DOUBLE(pBuf, pt.x);
                        STORE_DOUBLE(pBuf, pt.y);
                        pt.z = 0;
                        command->constraintPoints[t].push_back(pt);
                    }
                }

                commandReceived = true;
                break;

            case HAPTIC_COMMAND_CLEARCONSTRAINTS:
                commandReceived = true;
                break;

            default:
                fprintf(stderr, "StateMachine: ERROR - unrecognized haptic command id %d\n", command->commandId);
                commandReceived = false;
        }

        if(commandReceived)
            nCommandsReceived++;
        else
            break;
    }

    return nCommandsReceived;
}


/*
void stateMachineGetPerturbForce(cVector3d* force) {
    double currentTime;
    currentTime = chai.simClockNetwork.getCurrentTimeSeconds();

    double timeIntoPerturbationMs = (currentTime - perturbStartTime) * 1000;
    if(timeIntoPerturbationMs <= hapticCommand.perturbDuration) {
//        printf("Applying forces!!! WATCH OUT !!!\n");

        // #TODO: limit forces to safe range
        force->x = hapticCommand.perturbDirection.x;
        force->y = hapticCommand.perturbDirection.y;
        force->z = hapticCommand.perturbDirection.z;
     //   printf(" perturbation direction: %f   %f   %f \n", force->x, force->y, force->z);
    } else {
        force->zero();
    }
}

*/
