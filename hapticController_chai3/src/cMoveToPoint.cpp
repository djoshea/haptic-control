#include "math/CConstants.h"
#include "world/CMesh.h"
#include "math/CVector3d.h"
#include "world/CGenericObject.h"

#include "geometry.h"
#include "cMoveToPoint.h"
#include <vector>
#include <algorithm>

cMoveToPoint::cMoveToPoint(cWorld* a_world)
{
    m_parentWorld = a_world;

    // initalize endpoint sphere
    m_targetSphere = new cShapeSphere(2);
    m_targetSphere->m_material->m_ambient.set(1.0, 1.0, 1.0);
    m_targetSphere->m_material->m_diffuse.set(1.0, 1.0, 0.0);
    m_targetSphere->m_material->m_specular.set(1.0, 1.0, 1.0);
    m_targetSphere->setHapticEnabled(false, true);
    m_targetSphere->setShowEnabled(false, true);
    addChild(m_targetSphere);

    //initialize line segment indicator
    m_targetLine = new cShapeLine(cVector3d(0,0,0), cVector3d(0,0,0));
    m_targetLine->m_colorPointA.set(1, 1, 1);
    m_targetLine->m_colorPointB.set(1, 1, 1);
    m_targetLine->setHapticEnabled(false, true);
    m_targetLine->setShowEnabled(false, true);
    addChild(m_targetLine);

    m_useZ = true;
    m_active = false;
    m_hasStartPoint = false;
    m_advanceForce = 0;
    m_viscosity = 0;

    cEffectMoveToPoint* effect = new cEffectMoveToPoint(this);
    addEffect(effect);
}

void cMoveToPoint::setViscosity(double a_viscosity) {
    m_viscosity = a_viscosity;
}

double cMoveToPoint::getViscosity() {
    return m_viscosity;
}

void cMoveToPoint::setAdvanceForce(double a_force) {
    m_advanceForce = a_force;
}

double cMoveToPoint::getAdvanceForce() {
    return m_advanceForce;
}

void cMoveToPoint::setUseZForce(bool a_useZ) {
    m_useZ = a_useZ;
}

bool cMoveToPoint::getUseZForce() {
    return m_useZ;
}

void cMoveToPoint::moveToPoint(cVector3d a_point) {
    m_active = true;
    m_hasStartPoint = false;
    m_endPoint = a_point;

    // show graphical indicators
    m_targetLine->m_pointA.copyfrom(m_endPoint); // will be updated later
    m_targetLine->m_pointB.copyfrom(m_endPoint);
    m_targetSphere->setLocalPos(m_endPoint);
    m_targetLine->setShowEnabled(true, true);
    m_targetSphere->setShowEnabled(true, true);
}

void cMoveToPoint::abort() {
    m_active = false;
    m_hasStartPoint = false;

    // hide graphical indicators
    m_targetLine->setShowEnabled(false, true);
    m_targetSphere->setShowEnabled(false, true);
}

///////////////////////////////////////////////////////////////////
// cEffectMoveToPoint
///////////////////////////////////////////////////////////////////

cEffectMoveToPoint::cEffectMoveToPoint(cMoveToPoint *a_parent) : cGenericEffect(a_parent) {}

bool cEffectMoveToPoint::computeForce(const cVector3d& a_toolPos,
                      const cVector3d& a_toolVel,
                      const unsigned int& a_toolID,
                      cVector3d& a_reactionForce) {
    cMoveToPoint* parent = (cMoveToPoint*)m_parent;

    if(parent->m_active) {
        if(parent->m_hasStartPoint) {
            cVector3d m_startPoint = parent->m_startPoint;
            cVector3d m_endPoint = parent->m_endPoint;

            cVector3d deltaSegment; // force back to point on line segment
            cVector3d deltaEnd; // advancing force towards endpoint
            cVector3d drag; // viscous drag
            bool closeToEndpoint, applyAdvanceForce, applyDrag;


            // find the closest point on the segment spanning the start -> end move anchors
            cVector3d projectionOnSegment = cProjectPointOnSegment(a_toolPos, m_startPoint,
                 m_endPoint);

            // if we're close to the endpoint, just aim there with spring force,
            // don't bother advancing along the line segment
            if (cDistance(projectionOnSegment, m_endPoint) < 0.1) {
            	closeToEndpoint = true;
                applyAdvanceForce = false;
                applyDrag = true;
                // 2016-05-31, removed drag to reduce ringing at center
                //applyDrag = false;
                projectionOnSegment = m_endPoint;
            } else {
            	closeToEndpoint = false;
                applyAdvanceForce = true;
                applyDrag = true;
            }

            // pull towards closest point on the start->end line segment as a spring
            double stiffness = parent->m_material->getStiffness();
            deltaSegment = cSub(projectionOnSegment, a_toolPos);
            deltaSegment.mul(stiffness);

            // if we're not close to the endpoint, add an additional force towards the endpoint
            if (applyAdvanceForce) {
                deltaEnd = cSub(m_endPoint, projectionOnSegment);
                deltaEnd.mul(stiffness);

                double deltaEndForce = deltaEnd.length();
                double advanceForce = parent->getAdvanceForce();
                if (deltaEndForce <= C_SMALL) {
                    deltaEnd.zero();
                } else if (deltaEndForce >= advanceForce) {
                    deltaEnd.normalize();
                    deltaEnd.mul(advanceForce);
                }

            } else {
                deltaEnd.zero();
            }

            if (applyDrag) {
                // also apply viscous drag in this regime so that we proceed at constant
                // velocity
                double Kv = parent->getViscosity();  // N/m
                drag = a_toolVel;
                drag.mul(-Kv);
            } else {
                drag.zero();
            }

            // add these forces together
            a_reactionForce = cAdd(deltaSegment, deltaEnd, drag);

            // null out z forces if requested
            if(!parent->getUseZForce())
                a_reactionForce.z(0);

            // if we've advanced, update the start point to reflect
            // this progress and prevent backtracking
            if(cDistanceSq(projectionOnSegment, m_endPoint) <
               cDistanceSq(m_startPoint, m_endPoint) - C_SMALL) {
                parent->m_startPoint = projectionOnSegment;
            }

            // update the graphics
            parent->m_targetLine->m_pointA.copyfrom(a_toolPos);
            return true;

        } else {
            // no start point yet, set it to our current position
            // and pick up the force next iteration
            parent->m_startPoint.copyfrom(a_toolPos);
            parent->m_hasStartPoint = true;
            a_reactionForce.zero();
            return false;
        }

    } else {
        a_reactionForce.zero();
        return true;
    }
}
