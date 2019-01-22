#include "math/CConstants.h"
#include "math/CVector3d.h"
#include "math/CMatrix3d.h"
#include "math/CMaths.h"
#include "scenegraph/CGenericObject.h"

#include "geometry.h"
#include "environment.h"
#include "cSlowDownDragField.h"
#include "haptic.h"

extern ChaiData chai;
extern HapticState haptic;

cSlowDownDragField::cSlowDownDragField(cWorld* a_world)
{
    m_parentWorld = a_world;

    m_fieldActive = false;
    m_useZ = false;
    m_activateSpeed = 0;
    m_viscosity = 0;
    m_maxForce = 0;

    // for visualizing simulated curl force field
    m_vector = new cShapeVector(cVector3d(0,0,0), cVector3d(0,0,0), 5, 5);
    m_vector->m_ColorPointA.set(1.0, 0.3, 0.8, 1.0);
    m_vector->m_ColorPointB.set(1.0, 0.3, 0.8, 1.0);
    m_vector->setHapticEnabled(false, true);

    cEffectSlowDownDragField *effect = new cEffectSlowDownDragField(this);
    addEffect(effect);
}

void cSlowDownDragField::setViscosity(double a_viscosity) {
    m_viscosity = a_viscosity;
}

double cSlowDownDragField::getViscosity() {
    return m_viscosity;
}

void cSlowDownDragField::setMaxForce(double a_maxForce) {
    m_maxForce = a_maxForce;
}

double cSlowDownDragField::getMaxForce() {
    return m_maxForce;
}

void cSlowDownDragField::setActivateSpeed(double a_activateSpeed) {
    m_activateSpeed = a_activateSpeed;
}

double cSlowDownDragField::getActivateSpeed() {
    return m_activateSpeed;
}

void cSlowDownDragField::setUseZForce(bool a_useZ) {
    m_useZ = a_useZ;
}

bool cSlowDownDragField::getUseZForce() {
    return m_useZ;
}

void cSlowDownDragField::render(const int a_renderMode)
{
    m_vector->renderSceneGraph(a_renderMode);
}

void cSlowDownDragField::setFieldActive()
{
    m_fieldActive = true;
}

void cSlowDownDragField::setFieldInactive()
{
    m_fieldActive = false;
}

bool cSlowDownDragField::getFieldActive() {
    return m_fieldActive;
}
///////////////////////////////////////////////////////////////////////////////////
// cSlowDownDragField: handles force computations for the cSlowDownDragField class
///////////////////////////////////////////////////////////////////////////////////

cEffectSlowDownDragField::cEffectSlowDownDragField(cSlowDownDragField* a_parent):cGenericEffect(a_parent)
{
}

bool cEffectSlowDownDragField::computeForce(const cVector3d& a_toolPos,
                                          const cVector3d& a_toolVel,
                                          const unsigned int& a_toolID,
                                          cVector3d& a_reactionForce)
{
    cSlowDownDragField*parent = (cSlowDownDragField*)m_parent;

    if (parent->m_fieldActive == true)
    {
        // compute speed - activateSpeed
        double excessSpeed;
        excessSpeed = a_toolVel.length() - parent->m_activateSpeed;

        if(excessSpeed > 0) {
            a_reactionForce = -parent->m_viscosity * cNormalize(a_toolVel) * excessSpeed;

            if(a_reactionForce.length() > parent->m_maxForce)
                a_reactionForce = cNormalize(a_reactionForce) * parent->m_maxForce;

            if(!parent->m_useZ)
                a_reactionForce.z = 0;

            // zero out force if mass is inactive
            double gain = 10;
            parent->m_vector->m_pointA = a_toolPos;
            parent->m_vector->m_pointB = cAdd(a_toolPos, a_reactionForce * gain);

            /* move arrow above screen plane so visible */
            parent->m_vector->m_pointA.z = 1;
            parent->m_vector->m_pointB.z = 1;
            parent->m_vector->setShowEnabled(true, true);
        } else {
            a_reactionForce.zero();
            parent->m_vector->setShowEnabled(false, true);
        }
    } else {
    	a_reactionForce.zero();
        parent->m_vector->setShowEnabled(false, true);
    }


    return true;
}


