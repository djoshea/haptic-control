/*
 * cDirectionalMass.cpp
 *
 *  Created on: Apr 3, 2016
 *      Author: shenoylab
 */


#include "math/CConstants.h"
#include "world/CMesh.h"
#include "math/CVector3d.h"
#include "math/CMaths.h"
#include "world/CGenericObject.h"

#include "geometry.h"
#include "environment.h"
#include "cDirectionalMass.h"
#include "haptic.h"

extern ChaiData chai;
extern HapticState haptic;

cDirectionalMass::cDirectionalMass(cWorld* a_world, cVector3d a_massCoeffVector, cVector3d a_dragCoeffVector, double a_maxForce) : m_maxForce(a_maxForce)
{
	m_parentWorld = a_world;
	m_directionalMassVector = a_massCoeffVector;
	m_directionalDragVector = a_dragCoeffVector;

//	m_directionalMassCoeffX = a_massCoeffX;
//	m_directionalMassDragCoeffX = a_dragCoeffX;
//	m_directionalMassCoeffX = a_massCoeffY;
//	m_directionalMassDragCoeffX = a_dragCoeffY;

	m_pulseForce = cVector3d(0,0,0);
	m_massActive = true;

	// for visualizing simulated mass
	m_vector = new cShapeVector(cVector3d(0,0,0), cVector3d(0,0,0), 5, 5);
	m_vector->m_colorPointA.set(1.0, 0.3, 0.3, 1.0);
	m_vector->m_colorPointB.set(1.0, 0.3, 0.3, 1.0);
	m_vector->setHapticEnabled(false, true);

	cEffectDirectionalMass *effect = new cEffectDirectionalMass(this);
	addEffect(effect);
}

void cDirectionalMass::render(cRenderOptions &a_options)
{
//    m_vector->renderSceneGraph(a_options);
}

void cDirectionalMass::setDirectionalMassCoeff(cVector3d a_massCoeffVector)
{
	m_directionalMassVector = a_massCoeffVector;
}

cVector3d cDirectionalMass::getDirectionalMassCoeff()
{
	return m_directionalMassVector;

}

void cDirectionalMass::setDirectionalMassDragCoeff(cVector3d a_dragCoeffVector)
{
	m_directionalDragVector = a_dragCoeffVector;
}

cVector3d cDirectionalMass::getDirectionalMassDragCoeff()
{
	return m_directionalDragVector;

}

void cDirectionalMass::setPerturbationForce(cVector3d pulseForce)
{
	m_pulseForce = pulseForce;
}

void cDirectionalMass::setMassActive()
{
	m_massActive = true;
//	printf("setting mass Active \n");
}

void cDirectionalMass::setMassInactive()
{
	m_massActive = false;
//	printf("setting mass Inactive \n");
}

///////////////////////////////////////////////////////////////////////////////////
// cEffectDirectionalMass: handles force computations for the cEffectDirectionalMass class
///////////////////////////////////////////////////////////////////////////////////

cEffectDirectionalMass::cEffectDirectionalMass(cDirectionalMass* a_parent):cGenericEffect(a_parent)
{
}

bool cEffectDirectionalMass::computeForce(const cVector3d& a_toolPos,
        const cVector3d& a_toolVel,
        const unsigned int& a_toolID,
        cVector3d& a_reactionForce)
{
	cDirectionalMass *parent = (cDirectionalMass*)m_parent;

	if (parent->m_massActive == true)
	{
		cVector3d accelForce;
	//	accelForce = haptic.accelRig;
	//	accelForce.mul(-parent->m_directionalMassVector);
	//	accelForce = cMul(haptic.accelRig, -parent->m_directionalMassVector);
		accelForce.x(haptic.accelRig.x() * -parent->m_directionalMassVector.x());
		accelForce.y(haptic.accelRig.y() * -parent->m_directionalMassVector.y());
		accelForce.z(0);
		if (accelForce.length() > parent->m_maxForce) {
			accelForce.normalize();
			accelForce.mul(parent->m_maxForce);
		}

		cVector3d dragForce;
	//	dragForce = cMul(-parent->m_directionalDragVector, a_toolVel);
		dragForce.x(a_toolVel.x() * -parent->m_directionalDragVector.x());
		dragForce.y(a_toolVel.y() * -parent->m_directionalDragVector.y());
		dragForce.z(0);

		a_reactionForce = accelForce+dragForce;

		// adjust for perturbation
		// want to make directional mass and drag act ONLY in the opposite direction of the perturbation

		if (parent->m_pulseForce.x() > 0) {
			if (a_reactionForce.x() < 0) {
				a_reactionForce.x(0);
			}
		} else if (parent->m_pulseForce.x() < 0) {
			if (a_reactionForce.x() > 0){
				a_reactionForce.x(0);
			}
		}

	// zero out force if mass is inactive
	} else {

		a_reactionForce.x(0);
		a_reactionForce.y(0);
		a_reactionForce.z(0);
	}

//	a_reactionForce.(0);
//	a_reactionForce.y(0);
//	a_reactionForce.z(0);
	return true;
}

