#ifndef _HAPTIC_H_INCLUDED_
#define _HAPTIC_H_INCLUDED_

#include "math/CMatrix3d.h"
#include "geometry.h"
#include "cMoveToPoint.h"
#include "cConstrainToPoint.h"
#include "cBoundingCircle.h"
#include "cPlanarObstacle.h"
#include "cPerturbationPulse.h"
#include "cSupportLedge.h"
#include "cSlowDownDragField.h"

//-----------------------------------------------------------------------
// ENVIRONMENT CONSTANTS
//-----------------------------------------------------------------------

// this is the physical orientation of the device relative to its mount
#define HAPTIC_DEVICE_ANGLE_DEG 45.0 // device is angled 45 forward from upright
#define GAIN_HAPTIC_TO_RIG 1000.0 // haptic in meters, rig in mm

// this is the angle with respect to upright of the vertical workspace
#define HAPTIC_WORKSPACE_ANGLE_DEG 0.00

// this must be set correctly for gravity compensation to work correctly
//#define HAPTIC_EFFECTOR_MASS 0.395
#define HAPTIC_EFFECTOR_MASS 0.5 // changed from this after removing plastic 20181130

// device workspace plane is angled towards screen,relative to screen plane
// workspace size in mm
#define WORKSPACE_RADIUS 125

#define HAPTIC_MAX_TARGETS 3
#define HAPTIC_MAX_CONSTRAINTS 2

bool hapticsIsUp();
bool waitForHapticsReady();
void hapticsStart();

struct HapticState {
    // position and velocity in task coordinates
    double timeLastUpdate;
    cVector3d posRig;
    cVector3d velRig;
    cVector3d totalForce;
    bool hitObstacle;
    bool hitTarget;
    bool atWorkspaceEdge;
    bool onScreenPlane;
    bool hitConstraint;
};

struct HapticWorkspace {
    // haptic environment variables
    cBoundingCircle* screenPlane;
    cMoveToPoint *moveToPoint; // used for moving the haptic to a specific point
    cPlanarObstacle *obstacle; // contains a single planar obstacle
    cPlanarObstacle *gapLeftObstacle; // contains the left gap obstacle
    cPlanarObstacle *gapRightObstacle;

    cPlanarObstacle* targets[HAPTIC_MAX_TARGETS]; // green viscous trapping target at reach endpoint
    unsigned nTargets;

    cPlanarObstacle* constraints[HAPTIC_MAX_CONSTRAINTS]; // green viscous trapping target at reach endpoint
    unsigned nConstraints;

    cConstrainToPoint *constrainToPoint; // keeps the haptic at a particular point
    cSupportLedge *holdSupport; // a wall that supports the haptic from below at a particular point
    cPerturbationPulse *perturbationPulse; // a pulse of force as perturbation
    cShapeVector *forceVector; // show the force in X/Y

    cSlowDownDragField *slowDownDragField; // speed-gated drag field mainly used during training
};

bool hapticIsCollidingWith(const cGenericObject*);
void hapticMoveToPoint(cVector3d);
void hapticMoveToHandNotSeen();
void hapticEnableScreenPlane();
void hapticDisableScreenPlane();
void hapticMoveAbort();
void hapticCreateObstacle(vector<cVector3d>, bool);
void hapticCreateGap(bool, vector<cVector3d>, bool, vector<cVector3d>, bool);
void hapticClearObstacles();
void hapticCreateTarget(bool, bool, vector<cVector3d>);
void hapticClearTargets();
void hapticCreateConstraint(vector<cVector3d>);
void hapticClearConstraints();
void hapticConstrainToCurrentPoint();
void hapticConstrainAbort();
void hapticSupportAtPoint(cVector3d);
void hapticAbortSupportAtPoint();
void hapticTriggerPerturbationPulse(double, cVector3d);
void hapticAbortPerturbation();
void hapticResume();
void hapticRetractHandle();
void hapticReleaseForTesting();

#endif // ifndef _HAPTIC_H_INCLUDED_


