#include "math/CConstants.h"
#include "scenegraph/CMesh.h"
#include "math/CVector3d.h"
#include "scenegraph/CGenericObject.h"

#include "geometry.h"
#include "cConstrainToPoint.h"
#include <vector>
#include <algorithm>

cConstrainToPoint::cConstrainToPoint(cWorld* a_world)
{
    m_parentWorld = a_world;

    // initalize endpoint sphere
    m_targetSphere = new cShapeSphere(2);
    m_targetSphere->m_material.m_ambient = CHAI_COLOR_WHITE;
    m_targetSphere->m_material.m_diffuse.set(1.0, 1.0, 0.0);
    m_targetSphere->m_material.m_specular.set(1.0, 1.0, 1.0);
    m_targetSphere->setHapticEnabled(false, true);
    m_targetSphere->setShowEnabled(false, true);
    addChild(m_targetSphere);

    m_useZ = true;
    m_active = false;

    cEffectConstrainToPoint* effect = new cEffectConstrainToPoint(this);
    addEffect(effect);
}

void cConstrainToPoint::setUseZForce(bool a_useZ) {
    m_useZ = a_useZ;
}

bool cConstrainToPoint::getUseZForce() {
    return m_useZ;
}

void cConstrainToPoint::constrainToPoint(cVector3d a_point) {
    m_point = a_point;
    m_active = true;

    // show graphical indicators
    m_targetSphere->setPos(m_point);
    m_targetSphere->setShowEnabled(true, true);
}

void cConstrainToPoint::abort() {
    m_active = false;

    // hide graphical indicators
    m_targetSphere->setShowEnabled(false, true);
}

///////////////////////////////////////////////////////////////////
// cEffectMoveToPoint
///////////////////////////////////////////////////////////////////

cEffectConstrainToPoint::cEffectConstrainToPoint(cConstrainToPoint *a_parent) : cGenericEffect(a_parent) {}

bool cEffectConstrainToPoint::computeForce(const cVector3d& a_toolPos,
                      const cVector3d& a_toolVel,
                      const unsigned int& a_toolID,
                      cVector3d& a_reactionForce) {
    cConstrainToPoint* parent = (cConstrainToPoint*)m_parent;

    if(parent->m_active) {
        double stiffness = parent->m_material.getStiffness();

        // pull towards that point on the line segment as a spring
        a_reactionForce = cSub(parent->m_point, a_toolPos);
        a_reactionForce.mul(stiffness);

        // null out z forces if requested
        if(!parent->getUseZForce())
            a_reactionForce.z = 0;

        return true;

    } else {
        a_reactionForce.zero();
        return true;
    }
}



