#include <stdio.h>
#include <math.h>
#include "math/CVector3d.h"
#include "math/CMaths.h"
#include "math/CMatrix3d.h"
#include "math/CConstants.h"

#include "cMoveToPoint.h"
#include "cConstrainToPoint.h"
#include "cBoundingPlane.h"
#include "cPlanarObstacle.h"
#include "cShapeVector.h"
#include "cDirectionalViscosity.h"
#include "cDirectionalMass.h"
#include "cCircularBuffer.hpp"
#include "cCurlForceField.h"
#include "cErrorClamp.h"
#include "cConstantForceField.h"

#include "dhdc.h"
#include "drdc.h"
#include "geometry.h"
#include "environment.h"
#include "haptic.h"
#include "utils.h"


extern ChaiData chai; // declared in environment.cpp
double deviceAngleDeg = 0;
HapticState haptic; // most recent positions / state
HapticWorkspace workspace;

//double lastHapicStateUpdateTime;
//double lastPrintTime;
//const double PRINT_INTERVAL = 2;

const double obstacleFrictionStatic = 0.5;
const double obstacleFrictionDynamic = 0.4;
const double OBSTACLE_DEPTH = 80; // in z off the plane
const double SUPPORT_WIDTH = 8; //was 10 as of 2016-06-08
const cVector3d DEFAULT_DIRECTIONAL_DRAG_COEFF = cVector3d(0,0,0);
const cVector3d DEFAULT_DIRECTIONAL_MASS_COEFF = cVector3d(0,0,0);


//const double STATE_UPDATE_INTERVAL = .00025; //4kHz state updates

void hapticConditionWedge() {
  // build sample targets
  hapticClearTargets();
  hapticClearObstacles();

  cVector3d supportPoint;
  supportPoint.set(0, -75, 0);
  hapticSupportAtPoint(supportPoint);

  std::vector<cVector3d> targetPoints;
  targetPoints.clear();
  targetPoints.push_back(cVector3d(-8,75,0));
  targetPoints.push_back(cVector3d( 8,75,0));
  hapticCreateTarget(true, true, targetPoints);
  targetPoints.clear();
  targetPoints.push_back(cVector3d(-110,75,0));
  targetPoints.push_back(cVector3d( -65,75,0));
  hapticCreateTarget(true, true, targetPoints);
  targetPoints.clear();
  targetPoints.push_back(cVector3d( 65,75,0));
  targetPoints.push_back(cVector3d(110,75,0));
  hapticCreateTarget(true, true, targetPoints);

  std::vector<cVector3d> obstaclePointsLeft;
  std::vector<cVector3d> obstaclePointsRight;
  obstaclePointsLeft.push_back(cVector3d(-35,25,0));
  obstaclePointsLeft.push_back(cVector3d(-55,70,0));
  obstaclePointsLeft.push_back(cVector3d(-33,70,0));
  obstaclePointsLeft.push_back(cVector3d(-33,25,0));
  obstaclePointsRight.push_back(cVector3d(35,25,0));
  obstaclePointsRight.push_back(cVector3d(55,70,0));
  obstaclePointsRight.push_back(cVector3d(33,70,0));
  obstaclePointsRight.push_back(cVector3d(33,25,0));
  hapticCreateGap(true, obstaclePointsLeft, true, obstaclePointsRight, true);
}

void hapticConditionDebug() {
  // build sample targets
  hapticClearTargets();
  hapticClearObstacles();

//  cVector3d supportPoint;
//  supportPoint.set(0, -75, 0);
//  hapticSupportAtPoint(supportPoint);

//  std::vector<cVector3d> targetPoints;
//  targetPoints.clear();
//  targetPoints.push_back(cVector3d(-8,75,0));
//  targetPoints.push_back(cVector3d( 8,75,0));
//  hapticCreateTarget(true, true, targetPoints);
//  targetPoints.clear();
//  targetPoints.push_back(cVector3d(-110,75,0));
//  targetPoints.push_back(cVector3d( -65,75,0));
//  hapticCreateTarget(true, true, targetPoints);
//  targetPoints.clear();
//  targetPoints.push_back(cVector3d( 65,75,0));
//  targetPoints.push_back(cVector3d(110,75,0));
//  hapticCreateTarget(true, true, targetPoints);

  std::vector<cVector3d> obstaclePointsLeft;
  std::vector<cVector3d> obstaclePointsRight;
  obstaclePointsLeft.push_back(cVector3d(-35,25,0));
  obstaclePointsLeft.push_back(cVector3d(-55,70,0));
  obstaclePointsLeft.push_back(cVector3d(-33,70,0));
  obstaclePointsLeft.push_back(cVector3d(-33,25,0));
  obstaclePointsRight.push_back(cVector3d(35,25,0));
  obstaclePointsRight.push_back(cVector3d(55,70,0));
  obstaclePointsRight.push_back(cVector3d(33,70,0));
  obstaclePointsRight.push_back(cVector3d(33,25,0));
  hapticCreateGap(true, obstaclePointsLeft, true, obstaclePointsRight, true);
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
            ERROR("Initialization failed (%s)\n", dhdErrorGetLastStr ());
            return -1;
        }
    }

    double effectorMass = 0;
    if(dhdGetEffectorMass(&effectorMass) != 0) {
        ERROR("Could not query effector mass (%s)\n", dhdErrorGetLastStr());
        dhdClose();
        return -1;
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

    // initialize haptic state

    //haptic.posBufferFilled = false;
    //haptic.posBufferInit = false;
    //haptic.posRigBufferX = cCircularBuffer(haptic.accelFiltCoefficients.size());
    //haptic.posRigBufferX.zero();
    //haptic.posRigBufferY = cCircularBuffer(haptic.accelFiltCoefficients.size());
	//haptic.posRigBufferY.zero();
	//haptic.posRigBufferZ = cCircularBuffer(haptic.accelFiltCoefficients.size());
	//haptic.posRigBufferZ.zero();

    return 0;
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
    workspace.screenPlane = new cBoundingPlane(chai.world, WORKSPACE_WIDTH, WORKSPACE_HEIGHT);
    workspace.screenPlane->setLocalPos(0.0, 0.0, 0.0);
    // stiffness when on plane (high stiffness to keep on plane)
    workspace.screenPlane->setStiffness(chai.scaledStiffnessMax * 1.7);  // was 1.2 as of 2016-06-08
    workspace.screenPlane->setViscosity(chai.scaledLinearDampingMax * 0.6);

    if(strcasecmp(chai.deviceName,"delta.3") == 0) {
      // stiffness when returning to plane from far away (reduced to avoid fast movements)
      workspace.screenPlane->setStiffnessReturning(chai.scaledStiffnessMax * 0.15); // DJO had 0.15, was set to 0.12 (not sure if EMT did this or when), then back on 2016-06-08
      workspace.screenPlane->setViscosityReturning(chai.scaledLinearDampingMax * 0.8);
      // parameters for leaving and rejoining plane
      workspace.screenPlane->setDistanceLeavePlane(7.0); // default to 6 (was on 2016-06-08)
      workspace.screenPlane->setDistanceReturnPlane(3.0); //default to 3
    } else {
      // stiffness when returning to plane from far away (reduced to avoid fast movements)
      workspace.screenPlane->setStiffnessReturning(chai.scaledStiffnessMax * 0.45);
      workspace.screenPlane->setViscosityReturning(chai.scaledLinearDampingMax * 0.03);
      // parameters for leaving and rejoining plane
      workspace.screenPlane->setDistanceLeavePlane(20.0);
      workspace.screenPlane->setDistanceReturnPlane(10.0);
    }
    // force saturates at this distance off plane
    workspace.screenPlane->setDistanceForceSaturation(20.0);
    workspace.screenPlane->setShowEnabled(true, true);
    chai.world->addChild(workspace.screenPlane);

    // support the haptic at the touch point with a narrow ledge
    //printf("Scaled force max = %.3f\n", chai.scaledForceMax);
    workspace.holdSupport = new cSupportLedge(chai.world, OBSTACLE_DEPTH, SUPPORT_WIDTH,
            chai.scaledForceMax);
    workspace.holdSupport->m_material->m_ambient.set(0.5f, 0.5f, 0.5f);
    workspace.holdSupport->m_material->m_diffuse.set(1.0f, 1.0f, 1.0f);
    workspace.holdSupport->m_material->m_specular.set(1.0f, 1.0f, 1.0f);
    workspace.holdSupport->setStiffness(chai.scaledStiffnessMax); // 2016-06-08 was at 0.8* max
    workspace.holdSupport->setViscosity(chai.scaledLinearDampingMax/5);
    workspace.holdSupport->setShowEnabled(false, true);
    workspace.holdSupport->setHapticEnabled(false, true);
    chai.world->addChild(workspace.holdSupport);

    // moves the haptic back to the touch point
    workspace.moveToPoint = new cMoveToPoint(chai.world);
    workspace.moveToPoint->setStiffness(0.7*chai.scaledStiffnessMax); // DJO had 0.5, 2016-06-08 was .6
    workspace.moveToPoint->setViscosity(0.8*chai.scaledLinearDampingMax);  // 2016-06-08, was .5*
    workspace.moveToPoint->setAdvanceForce(6);  //2016-06-08 was 5
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
        workspace.targets[i]->setTrapViscosity(chai.scaledLinearDampingMax * 0);   // changed from 0.7 to 0.9 on 2016-11-24 by EMT; changed from 0.9 to 0 by XS for vinnie training. will change back.
        workspace.targets[i]->setTrapViscosityRampUpTime(10);
        workspace.targets[i]->setTrapStiffness(0 * chai.scaledStiffnessMax);// changed from 0.3 to 0 by XS for vinnie training. will change back.
        workspace.targets[i]->setStiffness(0.0);
        workspace.targets[i]->setViscosity(0);
        workspace.targets[i]->m_material->m_ambient.set(0.3f, 1.0f, 0.3f);
        workspace.targets[i]->m_material->m_diffuse.set(1.0f, 1.0f, 1.0f);
        workspace.targets[i]->m_material->m_specular.set(1.0f, 1.0f, 1.0f);
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
    workspace.constrainToPoint->setStiffness(0);  // was 1*chai.scaledStiffnessMax.XS
    workspace.constrainToPoint->setUseZForce(false);
    chai.world->addChild(workspace.constrainToPoint);

    // for showing active haptic forces
    workspace.forceVector = new cShapeLine(cVector3d(0,0,0), cVector3d(0,0,0));
    workspace.forceVector->setLineWidth(5);
    workspace.forceVector->m_colorPointA.setGreenSea();
    workspace.forceVector->m_colorPointB.setGreenSea();
    //workspace.forceVector->m_colorPointA.set(0.3, 1.0, 0.3, 1.0);
    ///workspace.forceVector->m_colorPointB.set(0.3, 1.0, 0.3, 1.0);
    workspace.forceVector->setHapticEnabled(false, true);
    chai.world->addChild(workspace.forceVector);

    // initialize a pulse perturbation, which will be configured/triggered
    // over the network
    workspace.perturbationPulse = new cPerturbationPulse(chai.world, chai.scaledForceMax / 2);
    chai.world->addChild(workspace.perturbationPulse);

    // draw both sides of mesh triangles
    chai.world->setUseCulling(false, true);

    workspace.failDrag = new cDirectionalViscosity(chai.world, DRAG_COEFF);
    chai.world->addChild(workspace.failDrag);

	workspace.directionalMass = new cDirectionalMass(chai.world, DEFAULT_DIRECTIONAL_MASS_COEFF, DEFAULT_DIRECTIONAL_DRAG_COEFF, chai.scaledForceMax / 2);
	chai.world->addChild(workspace.directionalMass);

	workspace.constantForceField = new cConstantForceField(chai.world, 0, 0);
	chai.world->addChild(workspace.constantForceField);

    // initialize a curl force field, which will be configured/triggered
    // over the network
    workspace.curlForceField = new cCurlForceField(chai.world, chai.scaledForceMax);
    //workspace.curlForceField->setHapticEnabled(false, true);
    chai.world->addChild(workspace.curlForceField);


    // initialize an error clamp, which will be configured/triggered
    // over the network
    workspace.errorClamp = new cErrorClamp(chai.world);
    chai.world->addChild(workspace.errorClamp);

//    hapticConditionDebug();
}



void hapticUpdateState() {


	// deal with update timing measurement
//	double stateUpdateDt = chai.simClockHapticStateUpdate.getCurrentTimeSeconds() - lastHapicStateUpdateTime;

//	lastHapicStateUpdateTime = chai.simClockHapticStateUpdate.getCurrentTimeSeconds();

//	if (chai.simClockHapticStateUpdate.getCurrentTimeSeconds() - lastPrintTime > PRINT_INTERVAL) {
//		printf("\n\n state updated delta T: %.8f \n\n", stateUpdateDt);
//		lastPrintTime = chai.simClockHapticStateUpdate.getCurrentTimeSeconds();
//	}


	// query the position and velocity and transform into rig coords
	haptic.posRig = chai.tool->getDeviceGlobalPos();
	haptic.velRig = chai.tool->getDeviceGlobalLinVel();

	if (haptic.posBufferInit) {
		haptic.posRigBufferX.write(haptic.posRig.x());
		haptic.posRigBufferY.write(haptic.posRig.y());
		haptic.posRigBufferZ.write(haptic.posRig.z());
	}


	// update the acceleration filter

	if (haptic.posRigBufferX.bufferFilled()) {
		haptic.posBufferFilled = true;
	}
	else {
		haptic.posBufferFilled = false; // would prefer to just init this to false and flip it once.
	}


	// only run acceleration filter if position buffer is filled
	//if (haptic.posBufferFilled) {
		//double accelXfiltered;
		//double accelYfiltered;
		//double accelZfiltered;

		//cVector3d accelRig_tmp;
		//accelRig_tmp.zero();

		//haptic.posRigBufferX.filterSamples(accelXfiltered, haptic.accelFiltCoefficients);
		//accelRig_tmp.x(accelXfiltered);

		//haptic.posRigBufferY.filterSamples(accelYfiltered, haptic.accelFiltCoefficients);
		//accelRig_tmp.y(accelYfiltered);

		//haptic.posRigBufferZ.filterSamples(accelZfiltered, haptic.accelFiltCoefficients);
		//accelRig_tmp.z(accelZfiltered);


		//haptic.accelRig = accelRig_tmp;
//
	//}
	// set filtered acceleration to zero as position buffer is initializing/filling.  This should take a fraction of a second.
	//else {
		//cVector3d accelRig_tmp;
		//accelRig_tmp.zero();
		//haptic.accelRig = accelRig_tmp;
	//}

	// if haptic is driving to a point, dropped simulated mass.
	//if (workspace.moveToPoint->m_active)
	//{
		//workspace.directionalMass->setMassInactive();
//		printf("setting mass Inactive \n");
	//} else {
	//	workspace.directionalMass->setMassActive();
	//	printf("setting mass Active +++ \n");
	//}

	// query the applied forces
	haptic.totalForce = chai.tool->getDeviceGlobalForce();

	// visually display the x,y force vector, above the screen plane
	double gain = 10;
	workspace.forceVector->m_pointA = haptic.posRig;
	workspace.forceVector->m_pointB = cAdd(haptic.posRig, haptic.totalForce * gain);
	workspace.forceVector->m_pointA.z(0.9);
	workspace.forceVector->m_pointB.z(0.9);

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
			workspace.perturbationPulse->rampDown(100);

			// lock the hand position there!
			//hapticConstrainToCurrentPoint();
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
    if (obj == NULL)
      return false;

    cAlgorithmFingerProxy* proxy = chai.tool->m_hapticPoint->m_algorithmFingerProxy;
    int numContacts = proxy->getNumCollisionEvents();
    bool isColliding = false;

    if(numContacts > 0 && proxy->m_collisionEvents[0]->m_object == obj)
        return true;
    if(numContacts > 1 && proxy->m_collisionEvents[1]->m_object == obj)
        return true;
    if(numContacts > 2 && proxy->m_collisionEvents[2]->m_object == obj)
        return true;

    return false;
}

void hapticMoveToPoint(cVector3d dest) {
    hapticAbortPerturbation();
    hapticConstrainAbort();
    hapticSetCurlForceFieldInactive();
    hapticErrorClampInactive();
    //hapticPauseSimulatedMass();

    // omit z forces
    dest.z(0);

    printf("viscosity: %.5f\n", chai.scaledLinearDampingMax);

    // stiffness: .4*max and viscosity .5*max as of 2016-04-26,
    // changing to reduce ringing and increase return force.
    workspace.moveToPoint->setStiffness(0.5*chai.scaledStiffnessMax);
    workspace.moveToPoint->setViscosity(chai.scaledLinearDampingMax);
    workspace.moveToPoint->setAdvanceForce(6.5);
    workspace.moveToPoint->setUseZForce(false);  //comment this out when training Vinnie, add this back for human test. 2017.4.14 XS. XS uncommented it for new progress in Vinnie training 20170806.
    workspace.moveToPoint->moveToPoint(dest);
}

void hapticMoveAbort() {
    workspace.moveToPoint->abort();
}

void hapticCreateObstacle(std::vector<cVector3d> obstaclePoints, bool collisionPermitted) {
    workspace.obstacle->setPoints(obstaclePoints);
    workspace.obstacle->setCollisionSticky(!collisionPermitted);
    workspace.obstacle->setShowEnabled(true, true);
    workspace.obstacle->setHapticEnabled(true, true);
}

void hapticCreateGap(bool hasGapLeft, std::vector<cVector3d> gapLeftPoints, bool hasGapRight, std::vector<cVector3d> gapRightPoints, bool collisionPermitted) {
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

void hapticCreateTarget(bool targetIsViscous, bool targetIsPlanar, std::vector<cVector3d> targetPoints) {

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
        target->setHapticEnabled(false, false);
//        target->setStiffness(0.0);
//        target->setViscosity(0.0);
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

void hapticCreateConstraint(std::vector<cVector3d> points) {

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
    //hapticConstrainAbort(); //added by xs 20170404.
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
    hapticSetCurlForceFieldInactive();
    //hapticConstrainAbort(); //added by xs 20170404.
    hapticErrorClampInactive();

    for(unsigned i = 0; i < HAPTIC_MAX_TARGETS; i++)
        workspace.targets[i]->setHapticEnabled(false, true);
    for(unsigned i = 0; i < HAPTIC_MAX_CONSTRAINTS; i++)
        workspace.constraints[i]->setHapticEnabled(false, true);

    workspace.gapLeftObstacle->setHapticEnabled(false, true);
    workspace.gapRightObstacle->setHapticEnabled(false, true);

    workspace.constrainToPoint->constrainToPoint(haptic.posRig); //all above lines for constraint commented out by xs. 20170404
}

void hapticConstrainAbort() {
    workspace.constrainToPoint->abort();
}

void hapticSupportAtPoint(cVector3d point) {
    workspace.holdSupport->setLocalPos(point);
    workspace.holdSupport->setShowEnabled(true, true);
    workspace.holdSupport->setHapticEnabled(true, true);
}

void hapticAbortSupportAtPoint() {
    workspace.holdSupport->setShowEnabled(false, true);
    workspace.holdSupport->setHapticEnabled(false, true);
}

void hapticTriggerPerturbationPulse(double durationMs, cVector3d force) {
    workspace.perturbationPulse->m_pulseDurationMs = cClamp(durationMs, 0.0, 50000.0);
    force.z(0);
    workspace.perturbationPulse->m_pulseForce = force;
    workspace.perturbationPulse->startPulse();

    // don't do this anymore
    //workspace.holdSupport->setShowEnabled(false, true);
    //workspace.holdSupport->setHapticEnabled(false, true);
}

void hapticInformDirectionalMassOfPerturbation(cVector3d force) {
	workspace.directionalMass->m_pulseForce = force;
}

void hapticAbortPerturbation() {
    workspace.perturbationPulse->abort();
}

void hapticRetractHandle() {
    hapticAbortPerturbation();
    hapticConstrainAbort();
    hapticSetCurlForceFieldInactive();
    hapticErrorClampInactive();
    //hapticPauseSimulatedMass();

    // move the handle to a up and back retracted point (units in mm)
    cVector3d retractPoint = cVector3d(0, 100, -220);

    // set move to point properties
    workspace.moveToPoint->setStiffness(0.1*chai.scaledStiffnessMax);
    workspace.moveToPoint->setViscosity(0.2*chai.scaledLinearDampingMax);
    workspace.moveToPoint->setAdvanceForce(3);
    workspace.moveToPoint->setUseZForce(true);
    workspace.moveToPoint->moveToPoint(retractPoint);

    // while disabling the screen plane
    workspace.screenPlane->setHapticEnabled(false, true);
    workspace.screenPlane->setShowEnabled(false, true);

    // and hide the force vector
    workspace.forceVector->setShowEnabled(false, true);

    // and hide the obstacles
    hapticClearObstacles();
    hapticAbortSupportAtPoint();
    hapticClearTargets();

}

void hapticResume() {

    hapticAbortPerturbation();
    hapticMoveAbort();
    hapticPauseSimulatedMass();

    // reenable the screen plane
    workspace.screenPlane->setHapticEnabled(true, true);
    workspace.screenPlane->setShowEnabled(true, true);

    // show the force vector
    workspace.forceVector->setShowEnabled(true, true);
    //hapticResumeSimulatedMass();
}

void hapticSetSimulatedMass(cVector3d simulatedMassVector, cVector3d simulatedDragVector) {
	workspace.directionalMass->setDirectionalMassCoeff(simulatedMassVector);
	workspace.directionalMass->setDirectionalMassDragCoeff(simulatedDragVector);
}

void hapticPauseSimulatedMass() {
	workspace.directionalMass->setMassInactive();
}

void hapticResumeSimulatedMass() {
	//workspace.directionalMass->setMassActive();
}

void hapticSetDragActive() {
	//workspace.failDrag->setDragActive();
}

void hapticSetDragInactive() {
	workspace.failDrag->setDragInactive();
}

void hapticSetConstantForceField(double forceFieldMagnitude, double forceFieldDirection) {
	//workspace.constantForceField->setForce(forceFieldMagnitude, forceFieldDirection);
}

void hapticSetConstantForceFieldActive() {
	//workspace.constantForceField->setForceFieldActive();
}

void hapticSetConstantForceFieldInactive() {
	workspace.constantForceField->setForceFieldInactive();
}

void hapticSetCurlForceFieldActive(cVector3d curlForceFieldGainX, cVector3d curlForceFieldGainY) {
	workspace.curlForceField->setXGainVector(curlForceFieldGainX);
	workspace.curlForceField->setYGainVector(curlForceFieldGainY);
	workspace.curlForceField->setFieldActive();
}

void hapticSetCurlForceFieldInactive(){
  workspace.curlForceField->setFieldInactive();
}

void hapticErrorClampActive(cVector3d wallPosition1, cVector3d wallPosition2, cVector3d wallOrient) {
    workspace.errorClamp->setWallPositions(wallPosition1, wallPosition2);
    workspace.errorClamp->setWallOrientation(wallOrient);
    workspace.errorClamp->setClampActive();
}  //later should add wall positions and wall directions as input for this method;
//and should declare+define methods for set wallPos and set wallDirection in the errorClamp.cpp + errorClamp.h files

void hapticErrorClampInactive()
{
    workspace.errorClamp->setClampInactive();
}
