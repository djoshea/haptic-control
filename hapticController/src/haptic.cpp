#include <stdio.h>
#include <math.h>
#include <pthread.h>
#include "math/CVector3d.h"
#include "math/CMaths.h"
#include "math/CMatrix3d.h"
#include "math/CConstants.h"

#include "cMoveToPoint.h"
#include "cConstrainToPoint.h"
#include "cBoundingCircle.h"
#include "cPlanarObstacle.h"
#include "cShapeVector.h"
#include "cSlowDownDragField.h"

#include "dhdc.h"
#include "drdc.h"
#include "geometry.h"
#include "environment.h"
#include "haptic.h"
#include "utils.h"
#include "hapticController.h"

extern ChaiData chai; // declared in environment.cpp
double deviceAngleDeg = 0;
HapticState haptic; // most recent positions / state
HapticWorkspace workspace;

const double obstacleFrictionStatic = 0.5;
const double obstacleFrictionDynamic = 0.4;
const double OBSTACLE_DEPTH = 80; // in z off the plane
const double SUPPORT_WIDTH = 10; // was 2 for pierre

pthread_t hapticsThread;
bool hapticsUp = false;
bool hapticsReady = false;

int hapticInitialize();
void initHapticTool();
void hapticConfigureDevice();
void updateHaptics(void);

void initHapticWorkspace();
void hapticUpdateState();
void hapticInitLabels();

bool waitForHapticsReady() {
    while(true) {
        if(chai.simulationFinished)
            return false;
        else if(hapticsUp && hapticsReady)
            return true;
        else
            cSleepMs(10);
    }
}

bool hapticsIsUp() {
    return hapticsUp;
}

void hapticsStart() {
    if(!hapticsUp) {
        // create a thread which starts the main haptics rendering loop
        //cThread* hapticsThread = new cThread();
        //hapticsThread->set(updateHaptics, CHAI_THREAD_PRIORITY_HAPTICS);
        //
        pthread_create(&hapticsThread, 0, (void* (*)(void*))updateHaptics, 0);
        struct sched_param sp;
        memset(&sp, 0, sizeof(struct sched_param));
        sp.sched_priority = sched_get_priority_max(SCHED_RR);
        pthread_setschedparam(hapticsThread, SCHED_RR, &sp);
    }
}


int hapticInitialize() {
    if(drdOpen() < 0) {
        ERROR("Cannot open device (%s)\n", dhdErrorGetLastStr());
        dhdSleep(2.0);
        return -1;
    }

    if(!drdIsSupported()) {
        ERROR("Unsupported device\n");
        dhdSleep(2.0);
        dhdClose();
        return -1;
    }

    LOG("Haptic: Device detected: %s\n", dhdGetSystemName());

    strncpy(chai.deviceName, dhdGetSystemName(), MAX_DEVICE_NAME_LENGTH);
    chai.deviceName[MAX_DEVICE_NAME_LENGTH-1] = '\0';

    if(strcasecmp(chai.deviceName, "delta.3") == 0) {
        deviceAngleDeg = HAPTIC_DEVICE_ANGLE_DEG;

        LOG("Haptic: Setting device orientation at %.1f deg\n", deviceAngleDeg);
        if(dhdSetDeviceAngleDeg(deviceAngleDeg) < 0) {
            ERROR("Could not set device angle (%s)\n", dhdErrorGetLastStr());
            dhdClose();
            return -1;
        }
    }

    // enable force
    dhdEnableForce(DHD_ON);

    // initialize if necessary
    LOG("Haptic: Checking device initialization\n");
    if (!drdIsInitialized()) {
        LOG("Haptic: Auto initializing device\n");
        if(drdAutoInit() < 0) {
            dhdSleep (2.0);
            dhdClose();
            printf("Haptics: Initialization failed (%s)\n", dhdErrorGetLastStr ());
            // no need to error out here
        }
    }

    double effectorMass = 0;
    if(dhdGetEffectorMass(&effectorMass) != 0) {
        ERROR("Could not query effector mass (%s)\n", dhdErrorGetLastStr());
        //dhdClose();
        //return -1;
    }
    LOG("Haptic: Effector mass is %.4f\n", effectorMass);

    int initPauseTime = 0;
    if(initPauseTime > 0) {
        LOG("Sleeping for ");
        for(int i = initPauseTime; i > 0; i--) {
            LOG("%d..", i);
            fflush(stdout);
            dhdSleep(1);
        }
    }

    drdClose();

    return 0;
}

void initHapticTool() {
    //-----------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //-----------------------------------------------------------------------

    // create a haptic device handler
    chai.handler = new cHapticDeviceHandler();

    chai.handler->getDevice(chai.hapticDevice, 0);

    // retrieve information about the current haptic device
    if (chai.hapticDevice)
    {
        chai.deviceInfo = chai.hapticDevice->getSpecifications();
    }
    // create a 3D tool and add it to the world
    chai.tool = new cGeneric3dofPointer(chai.world);

    // Real-world: X/Y is screen plane, X points right, Y points towards ceiling, Z points normal away from screen
    // Haptic: X is away from baseplate, Z is up when baseplate vertical, Y is right looking at baseplate
    // Transform is:
    //      Permute X --> Z, Z --> Y, Y --> X
    //      Convert meters to mm

    // rotate the tool so that Z is out from the screen, X is right, Y is up
    cMatrix3d mPermute = cMatrix3d();
    mPermute.set(0,1,0, 0,0,1, 1,0,0);
    chai.tool->setRot(mPermute);
    chai.world->addChild(chai.tool);

    // set the device to convert m to mm
    double workspaceScaleFactor, forceScaleFactor;
    //printf("Name is %s\n", chai.deviceName);
    if (strcasecmp(chai.deviceName, "delta.3") == 0) {
        // meters to mm
        workspaceScaleFactor = 1000;
        forceScaleFactor = 1000;
    } else if (strcasecmp(chai.deviceName, "falcon") == 0) {
        // compensate for the falcon's limited workspace
        workspaceScaleFactor = 3000;
        forceScaleFactor = 1000;
    } else {
        workspaceScaleFactor = 1000;
        forceScaleFactor = 1000;
    }

    printf("Haptic: Setting scale factor to %.0f\n", workspaceScaleFactor);

    chai.tool->setWorkspaceScaleFactor(workspaceScaleFactor);
    chai.workspaceScaleFactor = workspaceScaleFactor;

    // connect the haptic device to the tool
    chai.tool->setHapticDevice(chai.hapticDevice);

    // initialize tool by connecting to haptic device
    chai.tool->start();
    chai.tool->setForcesON();

    // call configure device now that we've started the chai thread
    hapticConfigureDevice();

    // define a radius for the tool (both graphical and haptic proxy)
    chai.proxyRadius = 0.5;
    chai.tool->m_proxyPointForceModel->setProxyRadius(chai.proxyRadius);
    chai.tool->m_proxyPointForceModel->m_collisionSettings.m_checkBothSidesOfTriangles = false;
    chai.tool->m_proxyPointForceModel->m_useDynamicProxy = true;

    // hide the device sphere. only show proxy.
    chai.tool->m_deviceSphere->setShowEnabled(false);
    chai.tool->m_proxySphere->m_material.m_ambient = CHAI_COLOR_WHITE;
    chai.tool->m_proxySphere->m_material.m_diffuse.set(1.0, 1.0, 0.0);
    chai.tool->m_proxySphere->m_material.m_specular.set(1.0, 1.0, 1.0);
    chai.tool->m_proxySphere->setShowEnabled(true);
    chai.tool->m_proxySphere->setRadius(chai.proxyRadius);

    printf("Haptic: Proxy point force model radius = %f.1 mm\n", chai.proxyRadius);

    // enable if objects in the scene are going to rotate of translate
    // or possibly collide against the tool. If the environment
    // is entirely static, you can set this parameter to "false"
    chai.tool->m_proxyPointForceModel->m_useDynamicProxy = true;

    // define a maximum stiffness that can be handled by the current
    // haptic device. The value is scaled to take into account the
    // workspace scale factor
    double stiffnessMax = (double)chai.deviceInfo.m_maxForceStiffness / forceScaleFactor;
    printf("Haptic: Device Stiffness Max: %f\n", stiffnessMax);
    chai.scaledStiffnessMax = stiffnessMax;

    double linearDamping = (double)chai.deviceInfo.m_maxLinearDamping / forceScaleFactor;
    printf("Haptic: Linear damping max: %.2f\n", linearDamping);
    chai.scaledLinearDampingMax = linearDamping;

    double maxForce = (double)chai.deviceInfo.m_maxForce;// / forceScaleFactor;
    chai.scaledForceMax = maxForce;
    printf("Haptic: Force max: %.2f\n", chai.scaledForceMax);

}


// CALLED IN INITHAPTICTOOL IN ENVIRONMENT.CPP, NOT FROM ABOVE
void hapticConfigureDevice() {

    if(strcasecmp(chai.deviceName,"delta.3") == 0) {
        double deviceAngleDeg = HAPTIC_DEVICE_ANGLE_DEG;

        LOG("Haptic: Setting device orientation at %.1f deg\n", deviceAngleDeg);
        if(dhdSetDeviceAngleDeg(deviceAngleDeg) < 0) {
            ERROR("Could not set device angle (%s)\n", dhdErrorGetLastStr());
        }

        LOG("Haptic: Setting effector mass to %.4f kg\n", HAPTIC_EFFECTOR_MASS);
        if(dhdSetEffectorMass(HAPTIC_EFFECTOR_MASS) != 0) {
            ERROR("Could not set effector mass (%s)\n", dhdErrorGetLastStr());
        }

    } else if (strcasecmp(chai.deviceName, "falcon") == 0) {
        // falcon initialization here?
        double falconEffectorMass = 0.4;
        LOG("Haptic: Setting effector mass to %.4f kg\n", falconEffectorMass);
        if(dhdSetEffectorMass(falconEffectorMass) != 0) {
            ERROR("Could not set effector mass (%s)\n", dhdErrorGetLastStr());
        }
    }
}

void initHapticWorkspace() {
    //-----------------------------------------------------------------------
    // COMPOSE THE VIRTUAL SCENE
    //-----------------------------------------------------------------------
    // bound haptic to screen plane
    workspace.screenPlane = new cBoundingCircle(chai.world, WORKSPACE_RADIUS);
    workspace.screenPlane->setPos(0.0, 0.0, 0.0);
    // stiffness when on plane (high stiffness to keep on plane)
    workspace.screenPlane->setStiffness(chai.scaledStiffnessMax * 0.9);
    workspace.screenPlane->setViscosity(chai.scaledLinearDampingMax * 0.4);
    // stiffness when returning to plane from far away (reduced to avoid fast movements)
    workspace.screenPlane->setStiffnessReturning(chai.scaledStiffnessMax * 0.10); // was 0.3, changed from 0.15 20181130
    workspace.screenPlane->setViscosityReturning(chai.scaledLinearDampingMax * 0.8); // changed from 0.8 20181130
    // parameters for leaving and rejoining plane
    workspace.screenPlane->setDistanceLeavePlane(6.0);
    workspace.screenPlane->setDistanceReturnPlane(1.5); // was 3.0
    // force saturates at this distance off plane
    workspace.screenPlane->setDistanceForceSaturation(10.0); // was 20.0
    workspace.screenPlane->setShowEnabled(true, true);
    chai.world->addChild(workspace.screenPlane);

    // support the haptic at the touch point with a narrow ledge
    //printf("Scaled force max = %.3f\n", chai.scaledForceMax);
    workspace.holdSupport = new cSupportLedge(chai.world, OBSTACLE_DEPTH, SUPPORT_WIDTH,
            chai.scaledForceMax); // 3.0 is max force, used to be chai.scaledForceMax
    workspace.holdSupport->m_material.m_ambient.set(0.5f, 0.5f, 0.5f);
    workspace.holdSupport->m_material.m_diffuse.set(1.0f, 1.0f, 1.0f);
    workspace.holdSupport->m_material.m_specular.set(1.0f, 1.0f, 1.0f);
    workspace.holdSupport->setStiffness(0.4*chai.scaledStiffnessMax); // was 0.8
    workspace.holdSupport->setViscosity(chai.scaledLinearDampingMax * 0.2);
    workspace.holdSupport->setShowEnabled(false, true);
    workspace.holdSupport->setHapticEnabled(false, true);
    chai.world->addChild(workspace.holdSupport);

    // moves the haptic back to the touch point
    workspace.moveToPoint = new cMoveToPoint(chai.world);
    workspace.moveToPoint->setStiffness(0.5*chai.scaledStiffnessMax);
    workspace.moveToPoint->setViscosity(0.5*chai.scaledLinearDampingMax);
    workspace.moveToPoint->setAdvanceForce(2.5); // was 5 with P
    workspace.moveToPoint->setUseZForce(false); // screen plane takes care of this
    chai.world->addChild(workspace.moveToPoint);

    // obstacles in the plane
    workspace.obstacle = new cPlanarObstacle(chai.world, OBSTACLE_DEPTH);
    workspace.obstacle->setStiffness(0.2*chai.scaledStiffnessMax);
    workspace.obstacle->setFriction(obstacleFrictionStatic, obstacleFrictionDynamic);
    chai.world->addChild(workspace.obstacle);

    workspace.gapLeftObstacle = new cPlanarObstacle(chai.world, OBSTACLE_DEPTH);
    workspace.gapLeftObstacle->setStiffness(0.2*chai.scaledStiffnessMax);
    workspace.gapLeftObstacle->setFriction(obstacleFrictionStatic, obstacleFrictionDynamic);
    chai.world->addChild(workspace.gapLeftObstacle);

    workspace.gapRightObstacle = new cPlanarObstacle(chai.world, OBSTACLE_DEPTH);
    workspace.gapRightObstacle->setStiffness(0.2*chai.scaledStiffnessMax);
    workspace.gapRightObstacle->setFriction(obstacleFrictionStatic, obstacleFrictionDynamic);
    chai.world->addChild(workspace.gapRightObstacle);

    // viscous target in the plane
    for(unsigned int i = 0; i < HAPTIC_MAX_TARGETS; i++) {
        workspace.targets[i] = new cPlanarObstacle(chai.world, OBSTACLE_DEPTH);
        workspace.targets[i]->setTrapViscosity(chai.scaledLinearDampingMax * 0.70);
        workspace.targets[i]->setTrapViscosityRampUpTime(10);
        workspace.targets[i]->setTrapStiffness(0.3 * chai.scaledStiffnessMax);
        workspace.targets[i]->setStiffness(0.0);
        workspace.targets[i]->setViscosity(0);
        workspace.targets[i]->m_material.m_ambient.set(0.3f, 1.0f, 0.3f);
        workspace.targets[i]->m_material.m_diffuse.set(1.0f, 1.0f, 1.0f);
        workspace.targets[i]->m_material.m_specular.set(1.0f, 1.0f, 1.0f);
        workspace.targets[i]->setTransparencyLevel(0.6, true, true);
        // debugging
        //workspace.targets[i]->setTrapStiffness(0.01 * chai.scaledStiffnessMax);
        //workspace.targets[i]->setTrapViscosity(chai.scaledLinearDampingMax * 0.070);
        chai.world->addChild(workspace.targets[i]);
    }
    workspace.nTargets = 0;

    // planar constraints (like obstacles only closer to start)
    for(unsigned int i = 0; i < HAPTIC_MAX_CONSTRAINTS; i++) {
        workspace.constraints[i] = new cPlanarObstacle(chai.world, OBSTACLE_DEPTH);
        workspace.constraints[i]->setStiffness(0.2*chai.scaledStiffnessMax);
        workspace.constraints[i]->setFriction(obstacleFrictionStatic, obstacleFrictionDynamic);
        workspace.constraints[i]->setTransparencyLevel(0.6, true, true);
        chai.world->addChild(workspace.constraints[i]);
    }
    workspace.nConstraints = 0;

    // keeps the haptic at a particular point
    workspace.constrainToPoint = new cConstrainToPoint(chai.world);
    workspace.constrainToPoint->setStiffness(1*chai.scaledStiffnessMax);
    workspace.constrainToPoint->setUseZForce(false);
    chai.world->addChild(workspace.constrainToPoint);

    // for showing active haptic forces
    workspace.forceVector = new cShapeVector(cVector3d(0,0,0), cVector3d(0,0,0), 5, 5);
    workspace.forceVector->m_ColorPointA.set(0.3, 1.0, 0.3, 1.0);
    workspace.forceVector->m_ColorPointB.set(0.3, 1.0, 0.3, 1.0);
    workspace.forceVector->setHapticEnabled(false, true);
    chai.world->addChild(workspace.forceVector);

    // initialize a pulse perturbation, which will be configured/triggered
    // over the network
    workspace.perturbationPulse = new cPerturbationPulse(chai.world, chai.scaledForceMax / 2);
    chai.world->addChild(workspace.perturbationPulse);

    workspace.slowDownDragField = new cSlowDownDragField(chai.world);
    workspace.slowDownDragField->setViscosity(chai.scaledLinearDampingMax * 4.0);
    workspace.slowDownDragField->setActivateSpeed(800);
    workspace.slowDownDragField->setMaxForce(0.8*chai.scaledForceMax);
    workspace.slowDownDragField->setUseZForce(false);
    //workspace.slowDownDragField->setFieldActive();
    workspace.slowDownDragField->setFieldInactive();
    chai.world->addChild(workspace.slowDownDragField);

    // draw both sides of mesh triangles
    chai.world->setUseCulling(false, true);

    chai.maxHapticForceXY = 12.0;
}

void updateHaptics(void) {
    hapticsUp = true;
    LOG("Haptics: starting\n");

    if(hapticInitialize() != 0) {
        LOG("Haptics: error during initialization\n");
        //hapticsUp = false;
        // TODO uncomment these
        //shutdown();
        //return;
   }

    initHapticTool(); // in haptic.cpp
    initHapticWorkspace(); // in haptic.cpp

    // start the haptic tool here
    chai.tool->start();

    bool forceStarted = false;
    // reset clock
    chai.simClock.reset();

    // start monotonic clock and never reset
    chai.simClockHapticMonotonic.start();

    double timeInterval = 0;

    // allow other threads waiting on haptic to proceed
    hapticsReady = true;

    // main haptic simulation loop
    while(!chai.simulationFinished)
    {
        // compute global reference frames for each object
        chai.world->computeGlobalPositions(true);

        // update position and orientation of tool
        chai.tool->updatePose();

        // compute interaction forces
        if(forceStarted)
            chai.tool->computeInteractionForces();
        else
            forceStarted = true;

        chai.simClock.start();

        // set maximum force for haptic in the XY plane
        cVector3d totalForceXY = chai.tool->m_lastComputedGlobalForce;
        totalForceXY.z = 0;
        if (totalForceXY.lengthsq() > pow(chai.maxHapticForceXY, 2.0)) {
            totalForceXY = cMul(chai.maxHapticForceXY, cNormalize(totalForceXY));
            chai.tool->m_lastComputedGlobalForce.x = totalForceXY.x;
            chai.tool->m_lastComputedGlobalForce.y = totalForceXY.y;
        }

        chai.tool->applyForces();

        chai.simClock.stop();
        timeInterval = chai.simClock.getCurrentTimeSeconds();
        chai.simClock.reset();

        hapticUpdateState();
    }

    // exit haptics thread
    printf("Haptics: closing\n");
    chai.tool->stop();
    cVector3d noForce = cVector3d(0,0,0);
    chai.hapticDevice->setForce(noForce);
    hapticsUp = false;
}

void hapticUpdateState() {
    // query the position and velocity and tranform into rig coords
    haptic.timeLastUpdate = chai.simClockHapticMonotonic.getCurrentTimeSeconds();
    haptic.posRig = chai.tool->m_deviceGlobalPos;
    haptic.velRig = chai.tool->m_deviceGlobalVel;

    // query the applied forces
    haptic.totalForce = chai.tool->m_lastComputedGlobalForce;

    // visually display the x,y force vector, above the screen plane
    double gain = 10;
    workspace.forceVector->m_pointA = haptic.posRig;
    workspace.forceVector->m_pointB = cAdd(haptic.posRig, haptic.totalForce * gain);
    workspace.forceVector->m_pointA.z = 0.9;
    workspace.forceVector->m_pointB.z = 0.9;

    // am i touching the edge of the workspace?
    haptic.atWorkspaceEdge = workspace.screenPlane->getIsToolAtEdge();
    haptic.onScreenPlane = workspace.screenPlane->getIsToolOnPlane();

    // am i touching the obstacle?
    bool hitObstacle = false;
    if(hapticIsCollidingWith(workspace.obstacle)) {
        hitObstacle = true;
        workspace.obstacle->setTransparencyLevel(1, true, true);
    }
    if(hapticIsCollidingWith(workspace.gapLeftObstacle)) {
        hitObstacle = true;
        workspace.gapLeftObstacle->setTransparencyLevel(1, true, true);
    }
    if(hapticIsCollidingWith(workspace.gapRightObstacle)) {
        hitObstacle = true;
        workspace.gapRightObstacle->setTransparencyLevel(1, true, true);
    }

    if(hitObstacle) {
        // hitting one of the obstacles
        haptic.hitObstacle = true;

    } else {
        workspace.obstacle->setTransparencyLevel(0.8, true, true);
        workspace.gapLeftObstacle->setTransparencyLevel(0.8, true, true);
        workspace.gapRightObstacle->setTransparencyLevel(0.8, true, true);
        haptic.hitObstacle = false;
   }

    // am i touching any of the constraints?
    cPlanarObstacle *constraint;
    bool hitConstraint = false;
    for(unsigned i = 0; i < workspace.nConstraints; i++) {
        if(i >= HAPTIC_MAX_CONSTRAINTS)
            break;

        constraint = workspace.constraints[i];

        if(!hitConstraint && hapticIsCollidingWith(constraint)) {
            hitConstraint= true;
            // make opaque
            constraint->setTransparencyLevel(1, true, true);

            // and trigger perturbation ramp down
            // @djoshea 20180625 - was originally 100 ms, changing to be 1 ms
            workspace.perturbationPulse->rampDown(0);

            // lock the hand position there!
            hapticConstrainToCurrentPoint();
            break;
        }
    }
    haptic.hitConstraint = hitConstraint;

    // am i touching the target? Here we trust the viscous trap
    cPlanarObstacle *target;
    for(unsigned i = 0; i < workspace.nTargets; i++) {
        if(i >= HAPTIC_MAX_TARGETS)
            break;

        target = workspace.targets[i];
        if(target->isTrapped()) {
            if(!haptic.hitTarget) {
                //printf("Haptic: Hit target %d!\n", i+1);
                // make opaque
                haptic.hitTarget = true;
                target->setTransparencyLevel(1, true, true);

                // and trigger perturbation ramp down
                workspace.perturbationPulse->rampDown(100);
            }
        } else {
            if (haptic.hitTarget) {
                //printf("Haptic: Released target %d\n", i+1);
                haptic.hitTarget = false;
                target->setTransparencyLevel(0.6, true, true);
            }
        }
    }
}

bool hapticIsCollidingWith(const cGenericObject* obj) {
    cProxyPointForceAlgo * proxy = chai.tool->m_proxyPointForceModel;
    int numContacts = proxy->getNumContacts();
    bool isColliding = false;

    if (obj == NULL)
        return false;

    if(numContacts > 0 && proxy->m_contactPoint0->m_object == obj)
        return true;
    if(numContacts > 1 && proxy->m_contactPoint1->m_object == obj)
        return true;
    if(numContacts > 2 && proxy->m_contactPoint1->m_object == obj)
        return true;

    return false;
}

void hapticMoveToPoint(cVector3d dest) {
    hapticAbortPerturbation();
    hapticConstrainAbort();

    // omit z forces
    dest.z = 0;
    // was 0.4, 0.5, 5
    workspace.moveToPoint->setStiffness(0.3*chai.scaledStiffnessMax);
    workspace.moveToPoint->setViscosity(0.4*chai.scaledLinearDampingMax);
    workspace.moveToPoint->setAdvanceForce(5);
    workspace.moveToPoint->setUseZForce(false);
    workspace.moveToPoint->moveToPoint(dest);

    hapticEnableScreenPlane();
}

void hapticMoveToHandNotSeen() {
    hapticAbortPerturbation();
    hapticConstrainAbort();

    // move to negative z position
    cVector3d dest = cVector3d(0,20,-1);
    // was 0.4, 0.5, 5
    workspace.moveToPoint->setStiffness(0.3*chai.scaledStiffnessMax);
    workspace.moveToPoint->setViscosity(0.4*chai.scaledLinearDampingMax);
    workspace.moveToPoint->setAdvanceForce(4);
    workspace.moveToPoint->setUseZForce(false);
    workspace.moveToPoint->moveToPoint(dest);

    //hapticDisableScreenPlane();
}

void hapticMoveAbort() {
    workspace.moveToPoint->abort();
    //hapticEnableScreenPlane();
}

void hapticCreateObstacle(vector<cVector3d> obstaclePoints, bool collisionPermitted) {
    workspace.obstacle->setPoints(obstaclePoints);
    workspace.obstacle->setCollisionSticky(!collisionPermitted);
    workspace.obstacle->setShowEnabled(true, true);
    workspace.obstacle->setHapticEnabled(true, true);
}

void hapticCreateGap(bool hasGapLeft, vector<cVector3d> gapLeftPoints, bool hasGapRight, vector<cVector3d> gapRightPoints, bool collisionPermitted) {
    if (hasGapLeft) {
        workspace.gapLeftObstacle->setPoints(gapLeftPoints);
        workspace.gapLeftObstacle->setCollisionSticky(!collisionPermitted);
        workspace.gapLeftObstacle->setShowEnabled(true, true);
        workspace.gapLeftObstacle->setHapticEnabled(true, true);
    } else {
        workspace.gapLeftObstacle->setShowEnabled(false, true);
        workspace.gapLeftObstacle->setHapticEnabled(false, true);
   }

    if (hasGapRight) {
        workspace.gapRightObstacle->setPoints(gapRightPoints);
        workspace.gapRightObstacle->setCollisionSticky(!collisionPermitted);
        workspace.gapRightObstacle->setShowEnabled(true, true);
        workspace.gapRightObstacle->setHapticEnabled(true, true);
    } else {
        workspace.gapRightObstacle->setShowEnabled(false, true);
        workspace.gapRightObstacle->setHapticEnabled(false, true);
    }

}

void hapticClearObstacles() {
    workspace.obstacle->setShowEnabled(false, true);
    workspace.obstacle->setHapticEnabled(false, true);

    workspace.gapLeftObstacle->setShowEnabled(false, true);
    workspace.gapLeftObstacle->setHapticEnabled(false, true);
    workspace.gapRightObstacle->setShowEnabled(false, true);
    workspace.gapRightObstacle->setHapticEnabled(false, true);
}

void hapticCreateTarget(bool targetIsViscous, bool targetIsPlanar, vector<cVector3d> targetPoints) {

    if(workspace.nTargets == HAPTIC_MAX_TARGETS) {
        fprintf(stderr, "Creating haptic target would exceed maximum of %u targets\n", HAPTIC_MAX_TARGETS);
        return;
    }

    cPlanarObstacle* target = workspace.targets[workspace.nTargets];
    workspace.nTargets++;

    target->setPoints(targetPoints);

    unsigned nPoints = targetPoints.size();
    target->setClosed(true);

    target->setShowEnabled(true, true);

    if(targetIsViscous || targetIsPlanar)
        target->setHapticEnabled(true, true);
    else
        target->setHapticEnabled(false, true);

    target->releaseTrap();
    if(targetIsViscous) {
        target->setHapticEnabled(true, true);
        target->armTrap();

    } else if (targetIsPlanar) {
        target->setHapticEnabled(true, true);
        target->armTrap();
    } else {
        target->setHapticEnabled(true, true);
        target->setStiffness(0.0);
        target->setViscosity(0.0);
        target->releaseTrap();
    }

    // temporarily disable for debugging
    //target->setHapticEnabled(false, true);
}

void hapticClearTargets() {
    cPlanarObstacle* target;
    for(unsigned i = 0; i < HAPTIC_MAX_TARGETS; i++)
    {
        target = workspace.targets[i];
        target->releaseTrap();
        target->setShowEnabled(false, true);
        target->setHapticEnabled(false, true);
    }
    workspace.nTargets = 0;
}

void hapticCreateConstraint(vector<cVector3d> points) {

    if(workspace.nConstraints == HAPTIC_MAX_CONSTRAINTS) {
        fprintf(stderr, "Creating haptic constraint would exceed maximum of %u constraints\n", HAPTIC_MAX_CONSTRAINTS);
        return;
    }

    cPlanarObstacle* constraint = workspace.constraints[workspace.nConstraints];
    workspace.nConstraints++;

    constraint->setPoints(points);

    constraint->setClosed(true);
    constraint->setShowEnabled(true, true);
    constraint->setHapticEnabled(true, true);
}

void hapticClearConstraints() {
    cPlanarObstacle* constraint;
    for(unsigned i = 0; i < HAPTIC_MAX_CONSTRAINTS; i++)
    {
        constraint = workspace.constraints[i];
        constraint->setShowEnabled(false, true);
        constraint->setHapticEnabled(false, true);
    }
    workspace.nConstraints = 0;
}

void hapticConstrainToCurrentPoint() {
    hapticAbortPerturbation();
    for(unsigned i = 0; i < HAPTIC_MAX_TARGETS; i++)
        workspace.targets[i]->setHapticEnabled(false, true);
    for(unsigned i = 0; i < HAPTIC_MAX_CONSTRAINTS; i++)
        workspace.constraints[i]->setHapticEnabled(false, true);

    workspace.gapLeftObstacle->setHapticEnabled(false, true);
    workspace.gapRightObstacle->setHapticEnabled(false, true);

    workspace.constrainToPoint->constrainToPoint(haptic.posRig);
}

void hapticConstrainAbort() {
    workspace.constrainToPoint->abort();
}

void hapticSupportAtPoint(cVector3d point) {
    workspace.holdSupport->setPos(point);
    workspace.holdSupport->setShowEnabled(true, true);
    workspace.holdSupport->setHapticEnabled(true, true);
}

void hapticAbortSupportAtPoint() {
    workspace.holdSupport->setShowEnabled(false, true);
    workspace.holdSupport->setHapticEnabled(false, true);
}

void hapticTriggerPerturbationPulse(double durationMs, cVector3d force) {
    workspace.perturbationPulse->m_pulseDurationMs = cClamp(durationMs, 0.0, 50000.0);
    force.z = 0;
    workspace.perturbationPulse->m_pulseForce = force;
    workspace.perturbationPulse->startPulse();

    // don't do this anymore
    //workspace.holdSupport->setShowEnabled(false, true);
    //workspace.holdSupport->setHapticEnabled(false, true);
}

void hapticAbortPerturbation() {
    workspace.perturbationPulse->abort();
}

void hapticDisableScreenPlane() {
    workspace.screenPlane->setHapticEnabled(false, true);
    workspace.screenPlane->setShowEnabled(false, true);
}

void hapticEnableScreenPlane() {
    // reenable the screen plane
    workspace.screenPlane->setHapticEnabled(true, true);
    workspace.screenPlane->setShowEnabled(true, true);
}

void hapticRetractHandle() {
    hapticAbortPerturbation();
    hapticConstrainAbort();

    // move the handle to a up and back retracted point (units in mm)
    cVector3d retractPoint = cVector3d(0, 100, -220);

    // set move to point properties
    workspace.moveToPoint->setStiffness(0.1*chai.scaledStiffnessMax);
    workspace.moveToPoint->setViscosity(0.3*chai.scaledLinearDampingMax);
    workspace.moveToPoint->setAdvanceForce(3);
    workspace.moveToPoint->setUseZForce(true);
    workspace.moveToPoint->moveToPoint(retractPoint);

    // while disabling the screen plane
    hapticDisableScreenPlane();

    // and hide the force vector
    workspace.forceVector->setShowEnabled(false, true);

    workspace.slowDownDragField->setFieldInactive();

    // and hide the obstacles
    hapticClearObstacles();
    hapticAbortSupportAtPoint();
    hapticClearTargets();

}

void hapticResume() {
    hapticAbortPerturbation();
    hapticMoveAbort();
    hapticMoveToHandNotSeen();

    //workspace.slowDownDragField->setFieldActive();
    workspace.slowDownDragField->setFieldInactive();

    hapticEnableScreenPlane();

    // show the force vector
    workspace.forceVector->setShowEnabled(true, true);
}

void hapticReleaseForTesting() {
	// demo mode simply for testing
	hapticAbortPerturbation();
	hapticMoveAbort();
	hapticEnableScreenPlane();
}
