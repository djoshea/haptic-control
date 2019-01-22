
/*  cErrorClamp.cpp
 *
 *  Created on: Dec 16, 2016
 *      Author: Xulu Sun
 */


#include "math/CConstants.h"
#include "world/CMesh.h"
#include "math/CVector3d.h"
#include "math/CMatrix3d.h"
#include "math/CMaths.h"
#include "world/CGenericObject.h"

#include "geometry.h"
#include "environment.h"
#include "cErrorClamp.h"


extern ChaiData chai;
//extern HapticState haptic;

const double MAX_VEL_TO_ACTIVATE_FORCE=1; // turn off the error clamp during initialization

cErrorClamp::cErrorClamp(cWorld* a_world)
{
    m_parentWorld = a_world;

    //m_pulseForce = cVector3d(0,0,0);
    m_clampActive = false;
    printf("Haptic: initializing error clamp, %d \n",m_clampActive);
    // for visualizing error clamp
    m_vector = new cShapeVector(cVector3d(0,0,0), cVector3d(0,0,0), 5, 5);
    m_vector->m_colorPointA.set(1.0, 0.3, 0.3, 1.0);
    m_vector->m_colorPointB.set(1.0, 0.3, 0.3, 1.0);
    m_vector->setHapticEnabled(false, true);

    m_effect = new cEffectErrorClamp(this);
    addEffect(m_effect);
}

void cErrorClamp::render(cRenderOptions &a_options)
{
    //    m_vector->renderSceneGraph(a_options);
}


void cErrorClamp::setDampingGain(double a_dampingGain)
{
	m_dampingGain = a_dampingGain;
}

double cErrorClamp::getDampingGain()
{
	return m_dampingGain;

}

void cErrorClamp::setSpringGain(double a_springGain)
{
	m_springGain = a_springGain;
}

double cErrorClamp::getSpringGain()
{
	return m_springGain;

}

void cErrorClamp::setClampActive()
{
    //m_effect->setEnabled(true);
    m_effect->m_waiting=false;
    m_effect->count=0;
    //printf("disable clamp until it moves to the correct location, %d \n", m_effect->m_waiting);
    m_clampActive = true;
    //printf("Haptic: set error clamp active, %d \n",m_clampActive);
}

void cErrorClamp::setClampInactive()
{
    //m_effect->setEnabled(false);
    m_clampActive = false;
    //	printf("setting error clamp Inactive \n");
}

void cErrorClamp::setWallPositions (cVector3d wallPosition1, cVector3d wallPosition2)
{
  m_wallPos1 = wallPosition1;
  m_wallPos2 = wallPosition2;
}

void cErrorClamp::setWallOrientation (cVector3d wallOrientation)
{
  m_wallOrientation = wallOrientation;
}

///////////////////////////////////////////////////////////////////////////////////
// cEffectErrorClamp: handles force computations for the cEffectErrorClamp class
///////////////////////////////////////////////////////////////////////////////////

cEffectErrorClamp::cEffectErrorClamp(cErrorClamp* a_parent):cGenericEffect(a_parent)
{
}

//define the function computeForce for the cEffectErrorClamp class//
bool cEffectErrorClamp::computeForce(const cVector3d& a_toolPos,
                                          const cVector3d& a_toolVel,
                                          const unsigned int& a_toolID,
                                          cVector3d& a_clampForce)
{
	cErrorClamp *parent = (cErrorClamp*)m_parent;

  cVector3d wallPos1;
  cVector3d wallPos2;
  cVector3d wallOrientation; // wall orientation is dependent on the reaching direction for that trial
  cVector3d distance1;
  cVector3d distance2;
  double distance;
	cVector3d localForce;
  cVector3d springForce;
  cVector3d dampingForce;

  double speed = a_toolVel.length ();
  //printf("speed is: %f \n",speed);

  if (m_waiting==true and speed < MAX_VEL_TO_ACTIVATE_FORCE)
  {
    count++;
  }

  if (m_waiting==true and count>=4000)
  {
    m_waiting=false;

    //printf("enable error clamp %d \n", m_waiting);
  }

  wallPos1 = parent->m_wallPos1;
  wallPos2 = parent->m_wallPos2;
  wallOrientation = parent->m_wallOrientation;

	if (parent->m_clampActive and m_waiting==false)
	{
    cVector3d toolPos = a_toolPos;
    cVector3d toolVel = a_toolVel;
    toolPos.z(0);
    toolVel.z(0);
    distance1 = toolPos-cProjectPointOnLine(toolPos, wallPos1, wallOrientation);
    distance2 = toolPos-cProjectPointOnLine(toolPos, wallPos2, wallOrientation);
    distance = cMax (distance1.length(), distance2.length());
    //cVector3d test = cProjectPointOnLine(toolPos, wallPos1, WallOrientation);
    //printf("%f %f %f %f\n", toolPos.y(),test.y(), test.x(), distance);

    if (distance >= WALL_WIDTH)
      {

        dampingForce = -WALL_VISCOSITY * (toolVel-cProject(toolVel, wallOrientation));;
          if (distance1.length() < distance2.length())
          {
            springForce = - WALL_STIFFNESS * distance1; //6 is the stiffness for the wall
          } else {
            springForce = - WALL_STIFFNESS * distance2;
          }

      } else{
        dampingForce = cVector3d(0,0,0);
        springForce = cVector3d(0,0,0);
      }

    //printf("%f, %f\n%f, %f\n", springForce.x(),springForce.y(), dampingForce.x(),dampingForce.y());
    localForce = cAdd(springForce,dampingForce);

	} else {
		localForce.x(0);
		localForce.y(0);
		localForce.z(0);
	}
  a_clampForce=localForce;
	return true;
}
