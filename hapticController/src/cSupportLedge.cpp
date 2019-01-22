#include "math/CConstants.h"
#include "scenegraph/CMesh.h"
#include "math/CVector3d.h"
#include "scenegraph/CGenericObject.h"

#include "geometry.h"
#include "cSupportLedge.h"

cSupportLedge::cSupportLedge(cWorld* a_world, double a_height, double a_width, double a_maxForce) :
m_height(a_height),
m_width(a_width)
{
    m_parentWorld = a_world;
    m_maxForce = a_maxForce;

    // build a mesh object to visually represent the ledge
    m_planeMesh = new cMesh(a_world);

    // add triangles to my mesh object to visually represent plane
    cVector3d tl = cVector3d(-m_width/2.0, 0,  m_height/2.0);
    cVector3d tr = cVector3d( m_width/2.0, 0,  m_height/2.0);
    cVector3d bl = cVector3d(-m_width/2.0, 0, -m_height/2.0);
    cVector3d br = cVector3d( m_width/2.0, 0, -m_height/2.0);
    m_planeMesh->newTriangle(tl, tr, br);
    m_planeMesh->newTriangle(br, bl, tl);
    m_planeMesh->setHapticEnabled(false, true); // not haptic enabled
    m_planeMesh->m_material.m_ambient.set(0.3f, 0.3f, 0.3f);
    m_planeMesh->m_material.m_diffuse.set(0.5f, 0.5f, 0.5f);
    m_planeMesh->m_material.m_specular.set(1.0f, 1.0f, 1.0f);

    // draw both sides of the plane
    m_planeMesh->setUseCulling(false, true);

    // DO NOT ADD PLANE MESH AS CHILD
    // it will be rendered inside render, but we don't want it to become
    // haptic enabled

    cEffectSupportLedge *effect = new cEffectSupportLedge(this);
    addEffect(effect);
}

void cSupportLedge::setTransparencyLevel(double alpha)
{
    m_planeMesh->setTransparencyLevel(alpha, true, true);
}

void cSupportLedge::setViscosity(double Kv)
{
    m_viscosity = Kv;
}

double cSupportLedge::getViscosity()
{
    return m_viscosity;
}

void cSupportLedge::render(const int a_renderMode)
{
    m_planeMesh->renderSceneGraph(a_renderMode);
}

///////////////////////////////////////////////////////////////////////////////////
// cEffectSupportLedge: handles force computations for the cSupportLedge class
///////////////////////////////////////////////////////////////////////////////////

cEffectSupportLedge::cEffectSupportLedge(cSupportLedge* a_parent):cGenericEffect(a_parent)
{
}

bool cEffectSupportLedge::computeForce(const cVector3d& a_toolPos,
                                  const cVector3d& a_toolVel,
                                  const unsigned int& a_toolID,
                                  cVector3d& a_reactionForce)
{
    cSupportLedge *parent = (cSupportLedge*)m_parent;
    cVector3d goalPos = a_toolPos;
    if(abs(a_toolPos.x) <= parent->m_width / 2.0 && a_toolPos.y < 0
            && a_toolPos.y > -20 // within X mm below the shelf, keeps it from suddenly popping up
            && abs(a_toolPos.z) <= parent->m_height/2.0) {
        // pushing on plane
        goalPos.y = 0;
        // get closest point on the bounded plane
        cVector3d vRestoring = cSub(goalPos, a_toolPos);
        cVector3d localForce = cVector3d(vRestoring);

        // compute spring restoring force
        double stiffness = m_parent->m_material.getStiffness();
        localForce.mul(stiffness);

        // saturate restoring force
        if(localForce.length() > parent->m_maxForce) {
            localForce.normalize();
            localForce.mul(parent->m_maxForce);
        }

        // apply drag along restoring force
        double Kv = parent->getViscosity();  // N/m
        cVector3d velCopy, drag, perp;
        velCopy = a_toolVel;
        velCopy.decompose(vRestoring, drag, perp);
        drag.mul(-Kv);
        localForce.add(drag);

        a_reactionForce = localForce;
        return true;

    } else {
        // not pushing on plane
        a_reactionForce.zero();
        return false;
    }
}
