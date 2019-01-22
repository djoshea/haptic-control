#include <stdlib.h>
#include "math/CConstants.h"
#include "world/CMesh.h"
#include "math/CVector3d.h"
#include "world/CGenericObject.h"

#include "geometry.h"
#include "cConstantForceField.h"


// NOTE: there is nothing directional about this yet, this is just standin code for now
cConstantForceField::cConstantForceField(cWorld* a_world, double a_forceMagnitude, double a_forceDirection) {
	m_parentWorld = a_world;
	m_forceMagnitude = a_forceMagnitude;
	m_forceDirection = a_forceDirection;

	m_forceX = m_forceMagnitude * cCosRad(m_forceDirection);
	m_forceY = m_forceMagnitude * cSinRad(m_forceDirection);

	cEffectConstantForceField *effect = new cEffectConstantForceField(this);
	addEffect(effect);

	m_active = true;
}

void cConstantForceField::setForce(double forceMagnitude, double forceDirection) {
	m_forceMagnitude = forceMagnitude;
	m_forceDirection = forceDirection;

	m_forceX = m_forceMagnitude * cCosRad(m_forceDirection);
	m_forceY = m_forceMagnitude * cSinRad(m_forceDirection);
}

double cConstantForceField::getForceMagnitude() {
	return m_forceMagnitude;
}

double cConstantForceField::getForceDirection() {
	return m_forceDirection;
}

double cConstantForceField::getForceX() {
	return m_forceX;
}

double cConstantForceField::getForceY() {
	return m_forceY;
}

void cConstantForceField::setForceFieldActive() {
	m_active = true;
}

void cConstantForceField::setForceFieldInactive() {
	m_active = false;
}


///////////////////////////////////////////////////////////////////////////////////
// cEffectConstantForceField: handles force computations for the cPerturbationPulse class
///////////////////////////////////////////////////////////////////////////////////

cEffectConstantForceField::cEffectConstantForceField(cConstantForceField* a_parent):cGenericEffect(a_parent) {
}

bool cEffectConstantForceField::computeForce(const cVector3d& a_toolPos,
        const cVector3d& a_toolVel,
        const unsigned int& a_toolID,
        cVector3d& a_reactionForce)
{
	cConstantForceField *parent = (cConstantForceField*)m_parent;

	if ((parent->m_forceMagnitude > 0) & (parent->m_active == true)) {
		a_reactionForce.x(parent->getForceX());
		a_reactionForce.y(parent->getForceY());

	} else {
		a_reactionForce.zero();
	}

	return true;
}
