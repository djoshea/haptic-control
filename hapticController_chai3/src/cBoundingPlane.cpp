#include "math/CConstants.h"
#include "world/CMesh.h"
#include "math/CVector3d.h"
#include "world/CGenericObject.h"

#include "geometry.h"
#include "cBoundingPlane.h"

cBoundingPlane::cBoundingPlane(cWorld* a_world, double szX, double szY)
{
    m_parentWorld = a_world;
    m_sizeX = szX;
    m_sizeY = szY;

    m_distanceLeavePlane = 5; //default was 5 from Dan
    m_distanceReturnPlane = 2;
    m_forceSaturationDistance = 20;

    m_state = RETURNING;
    m_toolAtEdge = false;

    // created the bounded plane that encapsulates the geometry calculations
    // the plane is centered at 0,0,0 and extends in X,Y with size (szX, szY)
    m_bplane = new cBoundedPlane3d(CHAI_VECTOR_X, CHAI_VECTOR_Y, CHAI_VECTOR_ZERO, m_sizeX/2.0, m_sizeY/2.0);

    // build a mesh object to visually represent the plane
    m_planeMesh = new cMesh();
    // add triangles for plane to my mesh object
    m_bplane->addToMesh(m_planeMesh);
    m_planeMesh->setHapticEnabled(false, true);

    addChild(m_planeMesh);

    m_proxyPoint = new cShapeSphere(2);
    m_proxyPoint->m_material->m_ambient.set(1.0, 1.0, 1.0);
    m_proxyPoint->m_material->m_diffuse.set(1.0, 1.0, 0.0);
    m_proxyPoint->m_material->m_specular.set(1.0, 1.0, 1.0);
    m_proxyPoint->setHapticEnabled(false, true);
    m_proxyPoint->setShowEnabled(false, true);
    addChild(m_proxyPoint);

    cEffectBoundingPlane* effect = new cEffectBoundingPlane(this);
    addEffect(effect);
}

cBoundingPlane::~cBoundingPlane() { }

void cBoundingPlane::setViscosity(double Kv)
{
    m_viscosity = Kv;
}

double cBoundingPlane::getViscosity()
{
    return m_viscosity;
}

void cBoundingPlane::setViscosityReturning(double Kv)
{
    m_viscosity_returning = Kv;
}

double cBoundingPlane::getViscosityReturning()
{
    return m_viscosity_returning;
}

void cBoundingPlane::setStiffnessReturning(double Kv)
{
    m_stiffness_returning = Kv;
}

double cBoundingPlane::getStiffnessReturning()
{
    return m_stiffness_returning;
}

void cBoundingPlane::setTransparencyLevel(double alpha)
{
    m_planeMesh->setTransparencyLevel(alpha, true, true);
}

bool cBoundingPlane::getIsToolAtEdge()
{
    return m_toolAtEdge;
}

bool cBoundingPlane::getIsToolOnPlane()
{
    return m_state == ONPLANE;
}

double cBoundingPlane::getSignedDistanceSquaredFromEdge() {
    return m_distFromEdge;
}

///////////////////////////////////////////////////////////////////////////////////
// cEffectBoundingPlane: handles force computations for the cBoundingPlane class
///////////////////////////////////////////////////////////////////////////////////

cEffectBoundingPlane::cEffectBoundingPlane(cBoundingPlane* a_parent):cGenericEffect(a_parent)
{
}

bool cEffectBoundingPlane::computeForce(const cVector3d& a_toolPos,
                                  const cVector3d& a_toolVel,
                                  const unsigned int& a_toolID,
                                  cVector3d& a_reactionForce)
{
    // get closest point on the bounded plane
    cBoundingPlane* parent = (cBoundingPlane*)m_parent;
    cBoundedPlane3d* bplane = parent->m_bplane;
    cVector3d closestPoint;

    bplane->boundedClosestPoint(a_toolPos, closestPoint);

    parent->m_proxyPoint->setLocalPos(closestPoint);

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
        parent->m_planeMesh->m_material->m_ambient.set(0.5f, 0.5f, 0.5f);
        parent->m_planeMesh->m_material->m_diffuse.set(0.5f, 0.5f, 0.5f);
        parent->m_planeMesh->m_material->m_specular.set(1.0f, 1.0f, 1.0f);
        parent->setTransparencyLevel(0.5);
    } else if(parent->m_toolAtEdge) {
        // red at edge
        parent->m_planeMesh->m_material->m_ambient.set(0.80f, 0.5f, 0.5f);
        parent->m_planeMesh->m_material->m_diffuse.set(0.80f, 0.5f, 0.5f);
        parent->m_planeMesh->m_material->m_specular.set(1.0f, 1.0f, 1.0f);
        parent->setTransparencyLevel(0.5);
    } else if(parent->m_state == ONPLANE) {
        // blue on plane
        parent->m_planeMesh->m_material->m_ambient.set(0.13f, 0.5f, 1.0f);
        parent->m_planeMesh->m_material->m_diffuse.set(0.13f, 0.5f, 1.0f);
        parent->m_planeMesh->m_material->m_specular.set(1.0f, 1.0f, 1.0f);
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
        stiffness = parent->m_material->getStiffness();
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
    decomposeVector3d(velCopy, vRestoring, drag, perp);
    drag.mul(-Kv);
    localForce.add(drag);

    a_reactionForce = localForce;
    return true;
}
