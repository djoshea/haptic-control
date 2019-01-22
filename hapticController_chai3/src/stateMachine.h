#ifndef _STATEMACHINE_H_
#define _STATEMACHINE_H_

#include <stdlib.h>
#include "haptic.h"

// define state machine states
#define idleState 1
#define perturbState 2

const int HAPTIC_COMMAND_PULSEPERTURBATION = 2;
const int HAPTIC_COMMAND_CONSTRAIN_LINETOTARGET= 3;
const int HAPTIC_COMMAND_CONSTRAIN_RELEASE = 4;
const int HAPTIC_COMMAND_MOVETOPOINT = 5;
const int HAPTIC_COMMAND_MOVEABORT = 6;
const int HAPTIC_COMMAND_CREATEOBSTACLE = 7;
const int HAPTIC_COMMAND_CLEAROBSTACLES = 8;
const int HAPTIC_COMMAND_CONSTRAINTOPOINT = 9;
const int HAPTIC_COMMAND_CONSTRAINABORT = 10;
const int HAPTIC_COMMAND_SUPPORTATPOINT = 11;
const int HAPTIC_COMMAND_SUPPORTABORT = 12;
const int HAPTIC_COMMAND_ABORTPERTURBATION = 13;
const int HAPTIC_COMMAND_CREATEGAP = 14;
const int HAPTIC_COMMAND_RETRACT = 15;
const int HAPTIC_COMMAND_RESUME = 16;
const int HAPTIC_COMMAND_CREATETARGETS = 17;
const int HAPTIC_COMMAND_CLEARTARGETS = 18;
const int HAPTIC_COMMAND_MOVETOHOLDNOTSEEN = 19;
const int HAPTIC_COMMAND_CREATECONSTRAINTS = 20;
const int HAPTIC_COMMAND_CLEARCONSTRAINTS = 21;
const int HAPTIC_COMMAND_CURLFIELDACTIVE = 22;
const int HAPTIC_COMMAND_CURLFIELDINACTIVE = 23;
const int HAPTIC_COMMAND_ERRORCLAMPACTIVE = 24;
const int HAPTIC_COMMAND_ERRORCLAMPINACTIVE = 25;
//const int HAPTIC_COMMAND_SETSIMULATEDMASS = 22;
//const int HAPTIC_COMMAND_SETSIMULATEDMASSACTIVE = 23;
//const int HAPTIC_COMMAND_SETSIMULATEDMASSINACTIVE = 24;
//const int HAPTIC_COMMAND_SETCURLGAIN = 25;
//const int HAPTIC_COMMAND_SETDRAGACTIVE = 26;
//const int HAPTIC_COMMAND_SETDRAGINACTIVE = 27;
//const int HAPTIC_COMMAND_SETCONSTANTFORCEFIELD = 28;
//const int HAPTIC_COMMAND_SETCONSTANTFORCEFIELDACTIVE = 29;
//const int HAPTIC_COMMAND_SETCONSTANTFORCEFIELDINACTIVE = 30;

#define MAX_COMMAND_POINTS 30

struct StateMachineState {
    bool isRetracted;
};

struct HapticCommand {
    int commandId;
    int version;

    uint16_t perturbType;
    double perturbationDelay;
    double perturbMagnitude;
    double perturbDuration;
    cVector3d perturbDirection;
    cVector3d XGainVector;
    cVector3d YGainVector;
    cVector3d wallPos1;
    cVector3d wallPos2;
    cVector3d wallOrientation;

    // when implementing a linear constraint
    bool linearConstraintActive;
    cVector3d linearConstraintEndRig;

    // when pulling the handle to a specific position
    bool moveActive;
    cVector3d moveEndpointRig;

    bool obstacleCollisionPermitted;
    std::vector<cVector3d> obstaclePoints;

    bool targetNoForces;
    bool targetIsViscous;
    bool targetIsPlanar;
    // matrix of targetPoints, each vector corresponds to one target
    std::vector<cVector3d> targetPoints[HAPTIC_MAX_TARGETS];
    unsigned nTargets;

    // matrix of constraint points, each vector corresponds to one constraint
    std::vector<cVector3d> constraintPoints[HAPTIC_MAX_CONSTRAINTS];
    unsigned nConstraints;

    // where the support will be located
    cVector3d supportPoint;

    // gap points
    bool gapCollisionPermitted;
    bool hasGapLeft;
    bool hasGapRight;
    std::vector<cVector3d> gapLeftPoints;
    std::vector<cVector3d> gapRightPoints;

    cVector3d simulatedMassVector;
    cVector3d simulatedDragVector;



    double constantForceFieldMagnitude;
    double constantForceFieldDirection;

};

void stateMachineStart();
void stateMachineStop();
void stateMachineUpdate(void);
int parseBinPacket(const uint8_t*, int, HapticCommand*, int);
void stateMachineGetPerturbForce( cVector3d*);

#endif
