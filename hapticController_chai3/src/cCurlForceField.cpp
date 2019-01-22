
/*  cCurlForceField.cpp
 *
 *  Created on: June 15, 2016
 *      Author: Xulu Sun + Dan O'Shea
 */


#include "math/CConstants.h"
#include "world/CMesh.h"
#include "math/CVector3d.h"
#include "math/CMatrix3d.h"
#include "math/CMaths.h"
#include "world/CGenericObject.h"

#include "geometry.h"
#include "environment.h"
#include "cCurlForceField.h"
#include "haptic.h"

extern ChaiData chai;
extern HapticState haptic;

const double MAX_VEL_TO_ACTIVATE_FORCE=1; // turn off curl force for the stabilization of haptic device when speed is too high

cCurlForceField::cCurlForceField(cWorld* a_world, double a_maxForce):m_maxForce(a_maxForce)
{
    m_parentWorld = a_world;

    m_fieldActive = false;
    m_maxForce = a_maxForce;
    printf("Haptic: initializing curl force field, %d \n",m_fieldActive);
    // for visualizing simulated curl force field
    m_vector = new cShapeVector(cVector3d(0,0,0), cVector3d(0,0,0), 5, 5);
    m_vector->m_colorPointA.set(1.0, 0.3, 0.3, 1.0);
    m_vector->m_colorPointB.set(1.0, 0.3, 0.3, 1.0);
    m_vector->setHapticEnabled(false, true);
    m_vector->setShowEnabled(false, true);

    m_effect = new cEffectCurlForceField(this);
    addEffect(m_effect);
}

void cCurlForceField::render(cRenderOptions &a_options)
{
    //    m_vector->renderSceneGraph(a_options);
}


void cCurlForceField::setXGainVector(cVector3d a_xGainVector)
{
	m_xGainVector = a_xGainVector;
}

cVector3d cCurlForceField::getXGainVector()
{
	return m_xGainVector;

}

void cCurlForceField::setYGainVector(cVector3d a_yGainVector)
{
	m_yGainVector = a_yGainVector;
}

cVector3d cCurlForceField::getYGainVector()
{
	return m_yGainVector;

}

void cCurlForceField::setFieldActive()
{
    //m_effect->setEnabled(true);
    m_effect->m_waiting=false;
    m_effect->count=0;
    //printf("disable curl force until it's stable, %d \n", m_effect->m_waiting);
    m_fieldActive = true;
    //printf("Haptic: set curl force field active, %d \n",m_fieldActive);
    m_vector->setShowEnabled(true, true);
}

void cCurlForceField::setFieldInactive()
{
    //m_effect->setEnabled(false);
    m_fieldActive = false;
    //	printf("setting force field Inactive \n");
    m_vector->setShowEnabled(false, true);
}

///////////////////////////////////////////////////////////////////////////////////
// cEffectCurlForceField: handles force computations for the cEffectCurlForceField class
///////////////////////////////////////////////////////////////////////////////////

cEffectCurlForceField::cEffectCurlForceField(cCurlForceField* a_parent):cGenericEffect(a_parent)
{
}

//define the function computeForce for the cEffectCurlForceField class//
bool cEffectCurlForceField::computeForce(const cVector3d& a_toolPos,
                                          const cVector3d& a_toolVel,
                                          const unsigned int& a_toolID,
                                          cVector3d& a_curlForce)
{
	cCurlForceField *parent = (cCurlForceField*)m_parent;
	cVector3d localForce;

  double speed = a_toolVel.length ();
  //printf("speed is: %f \n",speed);

  if (m_waiting==true and speed < MAX_VEL_TO_ACTIVATE_FORCE)
  {
    count++;
  }

  /*
  if (count>=200)
  {
    m_waiting=false;
    //printf("enable curl force, %d \n", m_waiting);  //only need this when testing without the network.
  }
  */

	if (parent->m_fieldActive and m_waiting==false)
	{

		localForce.x(a_toolVel.x()* -parent->m_xGainVector.x()+a_toolVel.y()* -parent->m_yGainVector.x());
		localForce.y(a_toolVel.x()* -parent->m_xGainVector.y()+a_toolVel.y()* -parent->m_yGainVector.y());
		localForce.z(0);

	} else {
		//
		localForce.x(0);
		localForce.y(0);
		localForce.z(0);
	}
  a_curlForce=localForce;
	return true;
}
