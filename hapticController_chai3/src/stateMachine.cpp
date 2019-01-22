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
bool isStateMachineUp = false;

int MESSAGE_STATEMACHINE = -1; // handle to opengl message label that we create and can update

void stateMachineStart() {
    if(!isStateMachineUp) {
        printf("StateMachine: initializing\n");
        isStateMachineUp = true;

        MESSAGE_STATEMACHINE = addMessageLabel();
        setMessageColor(MESSAGE_STATEMACHINE, cColorf(0.4, 1.0, 0.4));

        stateMachineThread = new cThread();
        stateMachineThread->start(stateMachineUpdate, CTHREAD_PRIORITY_HAPTICS);
    }
}

void stateMachineStop() {
    if(isStateMachineUp) {
        printf("State machine: stopping\n");
        isStateMachineUp = false;
    }
}

void stateMachineUpdate(void) {
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
    updateMessage(MESSAGE_STATEMACHINE, strMessage);

    while(isStateMachineUp) {
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

                	case HAPTIC_COMMAND_ERRORCLAMPACTIVE:
                		hapticErrorClampActive(hapticCommand.wallPos1, hapticCommand.wallPos2, hapticCommand.wallOrientation);
                		snprintf(strMessage, 200, "StateMachine: Error clamp on, wall positions: (%.2f, %.2f), (%.2f, %.2f)\n wall direction: (%.2f, %.2f)\n",
                				hapticCommand.wallPos1.x(), hapticCommand.wallPos1.y(),
                				hapticCommand.wallPos2.x(), hapticCommand.wallPos2.y(),
                				hapticCommand.wallOrientation.x(),hapticCommand.wallOrientation.y());
                		displayStatus = true;
                		break;

                	case HAPTIC_COMMAND_ERRORCLAMPINACTIVE:
                		hapticErrorClampInactive();
                		snprintf(strMessage, 200, "StateMachine: Error clamp off");
                		break;

                	case HAPTIC_COMMAND_CURLFIELDACTIVE:
                		hapticSetCurlForceFieldActive(hapticCommand.XGainVector, hapticCommand.YGainVector);
                		snprintf(strMessage, 200, "StateMachine: Curl force field on, Vx gain vector (%.2f, %.2f), Vy gain vector (%.2f, %.2f)\n",
                				hapticCommand.XGainVector.x(), hapticCommand.XGainVector.y(),
                				hapticCommand.YGainVector.x(), hapticCommand.YGainVector.y());
                		displayStatus = true;
                		break;

                	case HAPTIC_COMMAND_CURLFIELDINACTIVE:
                		hapticSetCurlForceFieldInactive();
                		snprintf(strMessage, 200, "StateMachine: Curl force field off");
                		break;

                    case HAPTIC_COMMAND_PULSEPERTURBATION:
                        hapticTriggerPerturbationPulse(hapticCommand.perturbDuration, hapticCommand.perturbDirection);

                        hapticInformDirectionalMassOfPerturbation(hapticCommand.perturbDirection);

                        snprintf(strMessage, 200, "StateMachine: Perturbation pulse for %.0f ms towards (%.2f, %.2f, %.2f)",
                                hapticCommand.perturbDuration, hapticCommand.perturbDirection.x(),
                                hapticCommand.perturbDirection.y(), hapticCommand.perturbDirection.z());
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
                            hap/icCommand.linearConstraintEndRig.z);

                        displayStatus = true;

                        //hapticSetLinearConstraint(&hapticCommand);
                        break;

                    case HAPTIC_COMMAND_CONSTRAIN_RELEASE:
                        //hapticReleaseLinearConstraint(&hapticCommand);
                        snprintf(strMessage, 200, "StateMachine: Release linear constraint command");
                        break;
            */
                    case HAPTIC_COMMAND_MOVETOPOINT:
                        snprintf(strMessage, 200, "StateMachine: Received move to point command (%.0f, %.0f, %.0f) **************************************",
                            hapticCommand.moveEndpointRig.x(), hapticCommand.moveEndpointRig.y(), hapticCommand.moveEndpointRig.z());
                        displayStatus = true;
                        hapticAbortPerturbation();
                        hapticConstrainAbort();
                        hapticSetCurlForceFieldInactive();
                        hapticErrorClampInactive();
                        hapticMoveToPoint(hapticCommand.moveEndpointRig);
                        //XS commented it out for Vinnie training.20170404. XS uncommented it for new progress in Vinnie training 20170806
                        //hapticMoveAbort(); //XS added this for Vinnie training. XS commented this out for new progress in Vin training 20170806
                        break;

                    case HAPTIC_COMMAND_MOVETOHOLDNOTSEEN:
                        snprintf(strMessage, 200, "StateMachine: Received hold not seen command (%.0f, %.0f, %.0f)",
                            hapticCommand.moveEndpointRig.x(), hapticCommand.moveEndpointRig.y(), hapticCommand.moveEndpointRig.z());
                        displayStatus = true;
                        hapticMoveToPoint(hapticCommand.moveEndpointRig);

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
//                        snprintf(strMessage, 200, "StateMachine: Ignoring constrain to current location");
                        //displayStatus = true;
                    	snprintf(strMessage, 200, "StateMachine: Constrain to current location");

                        hapticConstrainToCurrentPoint();
                        break;

                    case HAPTIC_COMMAND_CONSTRAINABORT:
                        snprintf(strMessage, 200, "StateMachine: Aborting constrain to current location");
                        hapticConstrainAbort();
                        break;

                    case HAPTIC_COMMAND_SUPPORTATPOINT:
                        snprintf(strMessage, 200, "StateMachine: Support ledge at (%.0f, %.0f, %.0f)",
                            hapticCommand.supportPoint.x(),
                            hapticCommand.supportPoint.y(),
                            hapticCommand.supportPoint.z());
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
                        hapticSetSimulatedMass(0,0);
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

                    /*case HAPTIC_COMMAND_SETSIMULATEDMASS:
                    	// snprintf(strMessage,200, "StateMachine: Setting simulated [mass, drag] X: [%.4f, %.4f];   Y: [%.4f, %.4f]",
                   			hapticCommand.simulatedMassVector.x(), hapticCommand.simulatedDragVector.x(),
                    			hapticCommand.simulatedMassVector.y(), hapticCommand.simulatedDragVector.y());
                    	//hapticSetSimulatedMass(hapticCommand.simulatedMassVector, hapticCommand.simulatedDragVector);

                    	// zero out perturbation vector - directional mass needs to know about perturbation
                    	//hapticInformDirectionalMassOfPerturbation(cVector3d(0,0,0));
                    	//break;

                    //case HAPTIC_COMMAND_SETSIMULATEDMASSACTIVE:
                    	//hapticResumeSimulatedMass();
                    	//snprintf(strMessage,200, "StateMachine: Setting simulated mass active \n");
                    	//break;

                    //case HAPTIC_COMMAND_SETSIMULATEDMASSINACTIVE:
                    	//hapticPauseSimulatedMass();
                    	//snprintf(strMessage,200, "StateMachine: Setting simulated mass inactive \n");
                    	//break;

                    //case HAPTIC_COMMAND_SETCURLGAIN:
                    	//snprintf(strMessage,200, "StateMachine: Setting curl gain *DOES NOTHING* %.4f\n", hapticCommand.curlGain);
                    	//does nothing right now
                    	//break;

                    //case HAPTIC_COMMAND_SETDRAGACTIVE:
                    	//hapticSetDragActive();
                    	//snprintf(strMessage,200, "StateMachine: Setting drag =======================  active \n");
                    	//break;

                    //case HAPTIC_COMMAND_SETDRAGINACTIVE:
                    	//hapticSetDragInactive();
                    	//snprintf(strMessage,200, "StateMachine: Setting drag ======================= NOT active \n");
                    	//break;

                    //case HAPTIC_COMMAND_SETCONSTANTFORCEFIELD:
                    	//snprintf(strMessage,200, "StateMachine: Setting constant force field Mag: %.2F   Dir: %.2f",
                    			//hapticCommand.constantForceFieldMagnitude, hapticCommand.constantForceFieldDirection);
                    	//hapticSetConstantForceField(hapticCommand.constantForceFieldMagnitude, hapticCommand.constantForceFieldDirection);
                    	//break;

                    //case HAPTIC_COMMAND_SETCONSTANTFORCEFIELDACTIVE:
                    	//snprintf(strMessage,200, "StateMachine: Setting constant force field active \n");
                    	//hapticSetConstantForceFieldActive();
                    	//break;

                   //case HAPTIC_COMMAND_SETCONSTANTFORCEFIELDINACTIVE:
                    	//snprintf(strMessage,200, "StateMachine: Setting constant force field inactive \n");
                    	//hapticSetConstantForceFieldInactive();
                        break;
                  */

                }
            }

            printf("%s\n", strMessage);
            if(displayStatus)
                updateMessage(MESSAGE_STATEMACHINE, strMessage);

        }

    }
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
    double d = 0;
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

        	case HAPTIC_COMMAND_ERRORCLAMPACTIVE:
        		STORE_DOUBLE(pBuf, d);
        		command->wallPos1.x(d);
        		STORE_DOUBLE(pBuf, d);
        		command->wallPos1.y(d);
        		STORE_DOUBLE(pBuf, d);
        		command->wallPos2.x(d);
        		STORE_DOUBLE(pBuf, d);
        		command->wallPos2.y(d);
        		STORE_DOUBLE(pBuf, d);
        		command->wallOrientation.x(d);
        		STORE_DOUBLE(pBuf, d);
        		command->wallOrientation.y(d);
        		command->wallPos1.z(0);
        		command->wallPos2.z(0);
        		command->wallOrientation.z(0);
        		commandReceived = true;
        		break;


        	case HAPTIC_COMMAND_ERRORCLAMPINACTIVE:
        		commandReceived = true;
        		break;


        	case HAPTIC_COMMAND_CURLFIELDACTIVE:
        		STORE_DOUBLE(pBuf, d);
        		command->XGainVector.x(d);
        		STORE_DOUBLE(pBuf, d);
        		command->XGainVector.y(d);
        		STORE_DOUBLE(pBuf, d);
        		command->YGainVector.x(d);
        		STORE_DOUBLE(pBuf, d);
        		command->YGainVector.y(d);
        		command->XGainVector.z(0);
        		command->YGainVector.z(0);
        		commandReceived = true;
        		break;

        	case HAPTIC_COMMAND_CURLFIELDINACTIVE:
        	    commandReceived = true;
        	    break;

            case HAPTIC_COMMAND_PULSEPERTURBATION:
                STORE_UINT16(pBuf, command->perturbType);
                STORE_DOUBLE(pBuf, command->perturbDuration);
                STORE_DOUBLE(pBuf, d);
                command->perturbDirection.x(d);
                STORE_DOUBLE(pBuf, d);
                command->perturbDirection.y(d);
                STORE_DOUBLE(pBuf, d);
                command->perturbDirection.z(d);
                commandReceived = true;

                break;

            case HAPTIC_COMMAND_ABORTPERTURBATION:
                commandReceived = true;
                break;

            case HAPTIC_COMMAND_CONSTRAIN_LINETOTARGET:
                break;
                STORE_DOUBLE(pBuf, d);
                command->linearConstraintEndRig.x(d);
                STORE_DOUBLE(pBuf, d);
                command->linearConstraintEndRig.y(d);
                command->linearConstraintEndRig.z(0);

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
                STORE_DOUBLE(pBuf, d);
                command->moveEndpointRig.x(d);
                STORE_DOUBLE(pBuf, d);
                command->moveEndpointRig.y(d);
                command->moveEndpointRig.z(-2);
                commandReceived = true;

                break;

            case HAPTIC_COMMAND_MOVETOHOLDNOTSEEN:
                command->moveActive = true;
                STORE_DOUBLE(pBuf, d);
                command->moveEndpointRig.x(d);
                STORE_DOUBLE(pBuf, d);
                command->moveEndpointRig.y(d);
                command->moveEndpointRig.z(-1);
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
                    STORE_DOUBLE(pBuf, d);
                    pt.x(d);
                    STORE_DOUBLE(pBuf, d);
                    pt.y(d);
                    pt.z(0);
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
                    STORE_DOUBLE(pBuf, d);
                    pt.x(d);
                    STORE_DOUBLE(pBuf, d);
                    pt.y(d);
                    pt.z(0);
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
                    STORE_DOUBLE(pBuf, d);
                    pt.x(d);
                    STORE_DOUBLE(pBuf, d);
                    pt.y(d);
                    pt.z(0);
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
                STORE_DOUBLE(pBuf, d);
                command->supportPoint.x(d);
                STORE_DOUBLE(pBuf, d);
                command->supportPoint.y(d);
                command->supportPoint.z(0);
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
//                command->targetNoForces = targetMode == 3;  // EMT added 2016-11-04 for constant force reaching

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
                        STORE_DOUBLE(pBuf, d);
                        pt.x(d);
                        STORE_DOUBLE(pBuf, d);
                        pt.y(d);
                        pt.z(0);
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
                        STORE_DOUBLE(pBuf, d);
                        pt.x(d);
                        STORE_DOUBLE(pBuf, d);
                        pt.y(d);
                        pt.z(0);
                        command->constraintPoints[t].push_back(pt);
                    }
                }

                commandReceived = true;
                break;

            case HAPTIC_COMMAND_CLEARCONSTRAINTS:
                commandReceived = true;
                break;

            /*
            case HAPTIC_COMMAND_SETSIMULATEDMASS:
            	double tmp;

            	STORE_DOUBLE(pBuf, tmp);
            	command->simulatedMassVector.x(tmp);
            	STORE_DOUBLE(pBuf, tmp);
            	command->simulatedDragVector.x(tmp);

            	STORE_DOUBLE(pBuf, tmp);
				command->simulatedMassVector.y(tmp);
				STORE_DOUBLE(pBuf, tmp);
				command->simulatedDragVector.y(tmp);


//            	STORE_DOUBLE(pBuf, command->simulatedMassVector.x());
//            	STORE_DOUBLE(pBuf, command->simulatedDragVector.x());
//
//            	STORE_DOUBLE(pBuf, command->simulatedMassVector.y());
//            	STORE_DOUBLE(pBuf, command->simulatedDragVector.y());

            	command->simulatedMassVector.z(0);
            	command->simulatedDragVector.z(0);

            	commandReceived = true;
            	break;

            case HAPTIC_COMMAND_SETSIMULATEDMASSACTIVE:
            	commandReceived = true;
            	break;

            case HAPTIC_COMMAND_SETSIMULATEDMASSINACTIVE:
            	commandReceived = true;
            	break;

            case HAPTIC_COMMAND_SETCURLGAIN:
            	double tmp2;
            	STORE_DOUBLE(pBuf, tmp2);
            	command->curlGain = tmp2;
            	commandReceived = true;
            	break;

            case HAPTIC_COMMAND_SETDRAGACTIVE:
//            	command->dragActive = true;
            	commandReceived = true;
            	break;

            case HAPTIC_COMMAND_SETDRAGINACTIVE:
//            	command->dragActive = false;
            	commandReceived = true;
            	break;

            case HAPTIC_COMMAND_SETCONSTANTFORCEFIELD:
            	commandReceived = true;
            	double tmpMag;
            	double tmpDir;
            	STORE_DOUBLE(pBuf, tmpMag);
            	command->constantForceFieldMagnitude = tmpMag;
            	STORE_DOUBLE(pBuf, tmpDir);
            	command->constantForceFieldDirection = tmpDir;

            	commandReceived = true;
            	break;

            case HAPTIC_COMMAND_SETCONSTANTFORCEFIELDACTIVE:
            	commandReceived = true;
            	break;

            case HAPTIC_COMMAND_SETCONSTANTFORCEFIELDINACTIVE:
            	commandReceived = true;
            	break;
            */

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
