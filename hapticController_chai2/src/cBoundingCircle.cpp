#include "math/CConstants.h"
#include "scenegraph/CMesh.h"
#include "math/CVector3d.h"
#include "scenegraph/CGenericObject.h"

#include "geometry.h"
#include "cBoundingCircle.h"

cBoundingCircle::cBoundingCircle(cWorld* a_world, double radius)
{
    m_parentWorld = a_world;
    m_radius = radius;

    m_distanceLeavePlane = 5;
    m_distanceReturnPlane = 2;
    m_forceSaturationDistance = 20;

    m_state = RETURNING;
    m_toolAtEdge = false;

    // created the bounded plane that encapsulates the geometry calculations
    // the plane is centered at 0,0,0 and extends in X,Y with size (szX, szY)
    m_bplane = new cBoundedCircle3d(CHAI_VECTOR_X, CHAI_VECTOR_Y, CHAI_VECTOR_ZERO, m_radius);

    // build a mesh object to visually represent the plane
    m_planeMesh = new cMesh(a_world);
    // add triangles for plane to my mesh object
    m_bplane->addToMesh(m_planeMesh);
    m_planeMesh->setHapticEnabled(false, true);

    addChild(m_planeMesh);

    m_proxyPoint = new cShapeSphere(2);
    m_proxyPoint->m_material.m_ambient = CHAI_COLOR_WHITE;
    m_proxyPoint->m_material.m_diffuse.set(1.0, 1.0, 0.0);
    m_proxyPoint->m_material.m_specular.set(1.0, 1.0, 1.0);
    m_proxyPoint->setHapticEnabled(false, true);
    m_proxyPoint->setShowEnabled(false, true);
    addChild(m_proxyPoint);

    cEffectBoundingCircle* effect = new cEffectBoundingCircle(this);
    addEffect(effect);
}

cBoundingCircle::~cBoundingCircle() { }

void cBoundingCircle::setViscosity(double Kv)
{
    m_viscosity = Kv;
}

double cBoundingCircle::getViscosity()
{
    return m_viscosity;
}

void cBoundingCircle::setViscosityReturning(double Kv)
{
    m_viscosity_returning = Kv;
}

double cBoundingCircle::getViscosityReturning()
{
    return m_viscosity_returning;
}

void cBoundingCircle::setStiffnessReturning(double Kv)
{
    m_stiffness_returning = Kv;
}

double cBoundingCircle::getStiffnessReturning()
{
    return m_stiffness_returning;
}

void cBoundingCircle::setTransparencyLevel(double alpha)
{
    m_planeMesh->setTransparencyLevel(alpha, true, true);
}

bool cBoundingCircle::getIsToolAtEdge()
{
    return m_toolAtEdge;
}

bool cBoundingCircle::getIsToolOnPlane()
{
    return m_state == ONPLANE;
}

double cBoundingCircle::getSignedDistanceSquaredFromEdge() {
    return m_distFromEdge;
}

///////////////////////////////////////////////////////////////////////////////////
// cEffectBoundingCircle: handles force computations for the cBoundingCircle class
///////////////////////////////////////////////////////////////////////////////////

cEffectBoundingCircle::cEffectBoundingCircle(cBoundingCircle* a_parent):cGenericEffect(a_parent)
{
}

bool cEffectBoundingCircle::computeForce(const cVector3d& a_toolPos,
                                  const cVector3d& a_toolVel,
                                  const unsigned int& a_toolID,
                                  cVector3d& a_reactionForce)
{
    // get closest point on the bounded plane
    cBoundingCircle* parent = (cBoundingCircle*)m_parent;
    cBoundedCircle3d* bplane = parent->m_bplane;
    cVector3d closestPoint;

    bplane->boundedClosestPoint(a_toolPos, closestPoint);

    parent->m_proxyPoint->setPos(closestPoint);

    cVector3d vRestoring = cSub(closestPoint, a_toolPos);

    // test whether we're very near or beyond the workspace edge
    // debounce via schmidt trigger
    double signedDistSqFromEdge = bplane->signedDistanceSquaredFromEdge(a_toolPos);
    parent->m_distFromEdge = signedDistSqFromEdge;
    if(parent->m_toolAtEdge && signedDistSqFromEdge > 2)
        parent->m_toolAtEdge = false;
    else if(!parent->m_toolAtEdge && signedDistSqFromEdge < 0.5)
        parent->m_toolAtEdge = true;

    double restoringDistance = vRestoring.length();

    // update current state
    if(restoringDistance > parent->m_distanceLeavePlane)
        parent->m_state = RETURNING;
    else if(restoringDistance <= parent->m_distanceReturnPlane)
        parent->m_state = ONPLANE;

    // update color to indicate state
    if(parent->m_state == RETURNING) {
        // gray off plane
        parent->m_planeMesh->m_material.m_ambient.set(0.5f, 0.5f, 0.5f);
        parent->m_planeMesh->m_material.m_diffuse.set(0.5f, 0.5f, 0.5f);
        parent->m_planeMesh->m_material.m_specular.set(1.0f, 1.0f, 1.0f);
        parent->setTransparencyLevel(0.5);
    } else if(parent->m_toolAtEdge) {
        // red at edge
        parent->m_planeMesh->m_material.m_ambient.set(0.80f, 0.5f, 0.5f);
        parent->m_planeMesh->m_material.m_diffuse.set(0.80f, 0.5f, 0.5f);
        parent->m_planeMesh->m_material.m_specular.set(1.0f, 1.0f, 1.0f);
        parent->setTransparencyLevel(0.5);
    } else if(parent->m_state == ONPLANE) {
        // blue on plane
        parent->m_planeMesh->m_material.m_ambient.set(0.13f, 0.5f, 1.0f);
        parent->m_planeMesh->m_material.m_diffuse.set(0.13f, 0.5f, 1.0f);
        parent->m_planeMesh->m_material.m_specular.set(1.0f, 1.0f, 1.0f);
        parent->setTransparencyLevel(1.0);
    }

    cVector3d localForce = cVector3d(vRestoring);

    // saturate restoring force at a certain distance
    if(restoringDistance > parent->m_forceSaturationDistance) {
        localForce.normalize();
        localForce.mul(parent->m_forceSaturationDistance);
    }

    // get state dependent stiffness and viscosity
    double stiffness, Kv; // N/m, N*sec/m
    if(parent->m_state == ONPLANE) {
        stiffness = parent->m_material.getStiffness();
        Kv = parent->getViscosity();
    } else {
        stiffness = parent->m_stiffness_returning;
        Kv = parent->getViscosityReturning();
    }

    // compute spring restoring force
    localForce.mul(stiffness);

    // apply drag along restoring force
    cVector3d velCopy, drag, perp;
    velCopy = a_toolVel;
    velCopy.decompose(vRestoring, drag, perp);
    drag.mul(-Kv);
    localForce.add(drag);

    a_reactionForce = localForce;
    return true;
}

