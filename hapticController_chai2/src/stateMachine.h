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

    // when implementing a linear constraint
    bool linearConstraintActive;
    cVector3d linearConstraintEndRig;

    // when pulling the handle to a specific position
    bool moveActive;
    cVector3d moveEndpointRig;

    bool obstacleCollisionPermitted;
    vector<cVector3d> obstaclePoints;

    bool targetIsViscous;
    bool targetIsPlanar;
    // matrix of targetPoints, each vector corresponds to one target
    vector<cVector3d> targetPoints[HAPTIC_MAX_TARGETS];
    unsigned nTargets;

    // matrix of constraint points, each vector corresponds to one constraint
    vector<cVector3d> constraintPoints[HAPTIC_MAX_CONSTRAINTS];
    unsigned nConstraints;

    // where the support will be located
    cVector3d supportPoint;

    // gap points
    bool gapCollisionPermitted;
    bool hasGapLeft;
    bool hasGapRight;
    vector<cVector3d> gapLeftPoints;
    vector<cVector3d> gapRightPoints;
};

bool stateMachineIsUp();
void stateMachineStart();
void stateMachineUpdate(void);
int parseBinPacket(const uint8_t*, int, HapticCommand*, int);
void stateMachineGetPerturbForce( cVector3d*);

#endif
