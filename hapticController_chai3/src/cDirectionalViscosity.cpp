#include "math/CConstants.h"
#include "world/CMesh.h"
#include "math/CVector3d.h"
#include "world/CGenericObject.h"

#include "geometry.h"
#include "cDirectionalViscosity.h"


// NOTE: there is nothing directional about this yet, this is just standin code for now
cDirectionalViscosity::cDirectionalViscosity(cWorld* a_world, double a_dragCoeffient)
{
	m_parentWorld = a_world;
	m_directionalViscosityCoeff = a_dragCoeffient;

	cEffectDirectionalViscosity *effect = new cEffectDirectionalViscosity(this);
	addEffect(effect);

	m_active = false;
}

void cDirectionalViscosity::setDirectionalViscosityCoeff(double dragCoeff)
{
	m_directionalViscosityCoeff = dragCoeff;
}

double cDirectionalViscosity::getDirectionalViscosityCoeff()
{
	return m_directionalViscosityCoeff;

}

void cDirectionalViscosity::setDragActive()
{
	m_active = true;
}

void cDirectionalViscosity::setDragInactive()
{
	m_active = false;
}


///////////////////////////////////////////////////////////////////////////////////
// cEffectDirectionalViscosity: handles force computations for the cPerturbationPulse class
///////////////////////////////////////////////////////////////////////////////////

cEffectDirectionalViscosity::cEffectDirectionalViscosity(cDirectionalViscosity* a_parent):cGenericEffect(a_parent)
{
}

bool cEffectDirectionalViscosity::computeForce(const cVector3d& a_toolPos,
        const cVector3d& a_toolVel,
        const unsigned int& a_toolID,
        cVector3d& a_reactionForce)
{
	cDirectionalViscosity *parent = (cDirectionalViscosity*)m_parent;

	// for now make it non-directional until it's working;
	if (parent->m_active)
	{
		a_reactionForce = cMul(-parent->m_directionalViscosityCoeff, a_toolVel);
	} else
	{
		cVector3d a_reactionForce;
		a_reactionForce.zero();
	}

	return true;
}
