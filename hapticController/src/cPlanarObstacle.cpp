#include "math/CConstants.h"
#include "scenegraph/CMesh.h"
#include "math/CVector3d.h"
#include "scenegraph/CGenericObject.h"

#include "geometry.h"
#include "cPlanarObstacle.h"
#include <vector>
#include <algorithm>
#include <cmath>
#include "triangulatePolygon.h"
#include "environment.h"

extern ChaiData chai;
extern int MESSAGE_DEBUG;

cPlanarObstacle::cPlanarObstacle(cWorld* a_world, double a_height, const vector<cVector3d> a_points) :
cMesh(a_world),
m_height(a_height),
m_sticky(false),
m_closed(true)
{
    setAppearance();

    m_surface = new cEffectSurface(this);
    addEffect(m_surface);

    //m_material.setMagnetMaxDistance(2);
    //m_material.setMagnetMaxForce(chai.scaledForceMax / 2.0);
    //m_magnet = new cEffectMagnet(this);
    //addEffect(m_magnet);

    //m_vibration = new cEffectVibration(this);
    //addEffect(m_vibration);

    // add internal viscosity
    m_viscosity = new cEffectViscosity(this);
    addEffect(m_viscosity);

    // add viscous trap effect that takes hold upon collision
    m_collisionTrap = new cEffectPlanarViscousTrap(this);
    addEffect(m_collisionTrap);

    setHapticProperties();

    setPoints(a_points);

    // for visualizing trap direction vector
    m_trapVectorVis = new cShapeVector(cVector3d(0,0,0), cVector3d(0,0,0), 5, 5);
    m_trapVectorVis->m_ColorPointA.set(0.3, 1.0, 0.3, 1.0);
    m_trapVectorVis->m_ColorPointB.set(0.3, 1.0, 0.3, 1.0);
    m_trapVectorVis->setHapticEnabled(false, true);
    m_trapVectorVis->setShowEnabled(false, true);
    addChild(m_trapVectorVis);

    // for visualizing trap spring vector
    m_trapForceVectorVis = new cShapeVector(cVector3d(0,0,0), cVector3d(0,0,0), 5, 5);
    m_trapForceVectorVis->m_ColorPointA.set(1.0, 1.0, 0.3, 1.0);
    m_trapForceVectorVis->m_ColorPointB.set(1.0, 1.0, 0.3, 1.0);
    m_trapForceVectorVis->setHapticEnabled(false, true);
    m_trapForceVectorVis->setShowEnabled(false, true);
    addChild(m_trapForceVectorVis);
}


cPlanarObstacle::~cPlanarObstacle() { }

void cPlanarObstacle::setClosed(bool a_closed) {
    m_closed = a_closed;
    rebuildMesh();
}

void cPlanarObstacle::setViscosity(double a_viscosity) {
    m_material.setViscosity(a_viscosity);
}

void cPlanarObstacle::setTrapViscosity(double a_viscosity) {
    m_collisionTrap->setTrapViscosity(a_viscosity);
}

void cPlanarObstacle::setTrapViscosityRampUpTime(double durationMs) {
    m_collisionTrap->setTrapViscosityRampUpTime(durationMs);
}

void cPlanarObstacle::setTrapStiffness(double a_stiffness) {
    m_collisionTrap->setTrapStiffness(a_stiffness);
}

void cPlanarObstacle::setCollisionSticky(bool a_sticky) {
    m_sticky = a_sticky;
    setHapticProperties();
    setAppearance();
}

void cPlanarObstacle::setHapticProperties() {
    m_surface->enable(true);
    //m_magnet->enable(false);
    //m_vibration->enable(false);
    m_viscosity->enable(false);
    m_collisionTrap->enable(true);
}

bool cPlanarObstacle::isTrapped() {
    return m_collisionTrap->isTrapped();
}

void cPlanarObstacle::armTrap() {
    return m_collisionTrap->armTrap();
}

void cPlanarObstacle::releaseTrap() {
    return m_collisionTrap->releaseTrap();
}

void cPlanarObstacle::setAppearance() {
    if(m_sticky) {
        // red
        m_material.m_ambient.set(1.0f, 0.2f, 0.2f);
        m_material.m_diffuse.set(0.5f, 0.5f, 0.5f);
        m_material.m_specular.set(1.0f, 1.0f, 1.0f);
    } else {
        // yellow
        m_material.m_ambient.set(1.0f, 0.9f, 0.4f);
        m_material.m_diffuse.set(0.5f, 0.5f, 0.5f);
        m_material.m_specular.set(1.0f, 1.0f, 1.0f);
    }
}

void cPlanarObstacle::setPoints(vector<cVector3d> a_points) {
    m_points = vector<cVector3d>(a_points);

    if (m_points.size() < 2) {
        clear();
        return;
    }

    unsigned nPoints = m_points.size();
    if(nPoints > 2) {
        // first, check whehter the polygon points are in clockwise order
        // see http://stackoverflow.com/questions/1165647/how-to-determine-if-a-list-of-polygon-points-are-in-clockwise-order/1165943#1165943
        unsigned c, l;
        float accum = 0;
        for(unsigned i = 0; i <= nPoints; i++) {
            c = i % nPoints;
            l = (i-1) % nPoints;
            accum += (m_points[c].x - m_points[l].x)*(m_points[c].y + m_points[l].y);
        }

        if(accum < 0) {
            // points run counterclockwise, reverse it!
            //printf("Reversing points order to make clockwise!");
            std::reverse(m_points.begin(), m_points.end());
        }
    }

    // compute inward pointing trapping vector, for a 2 point obstacle, this
    // should point "into" the trapping region. Take the vector from point B to point A
    // and rotate 90 degrees clockwise
    if(nPoints == 2) {
        cVector3d firstWall = cSub(m_points[1], m_points[0]);
        m_trapInwardVec.x = -firstWall.y;
        m_trapInwardVec.y = firstWall.x;
        m_trapInwardVec.z = 0;
        m_trapInwardVec.normalize();

        m_trapLateralA = m_points[0];
        m_trapLateralB = m_points[1];

        // start the vector at the origin
        m_trapOrigin = cAdd(m_trapLateralA, cMul(0.5, cSub(m_trapLateralB, m_trapLateralA)));

        // point the visualizing vector in this direction
        m_trapVectorVis->m_pointA = m_trapOrigin;
        m_trapVectorVis->m_pointA.z = 5;

        m_trapVectorVis->m_pointB = cAdd(m_trapOrigin, cMul(20, m_trapInwardVec));
        m_trapVectorVis->m_pointB.z = 5;
        m_trapVectorVis->setShowEnabled(true, true);
    } else {
        m_trapInwardVec.zero();
        m_trapOrigin.zero();
        m_trapLateralA.zero();
        m_trapLateralB.zero();
        m_trapVectorVis->setShowEnabled(false, true);
    }

    rebuildMesh();
}

void cPlanarObstacle::rebuildMesh() {
    // clear existing points and mesh
    clear();

    if (m_points.size() < 2) {
        return;
    }

    int lastVertices[2], currentVertices[2];
    cVector3d corners[4];
    cVector3d topVertexCoords, bottomVertexCoords;
    cVector3d currentPoint, lastPoint;
    unsigned nPoints = m_points.size();

    m_topVertices.clear();
    m_bottomVertices.clear();

    // build for first point
    currentPoint = m_points[0];
    topVertexCoords.set(currentPoint.x, currentPoint.y, m_height/2.0);
    bottomVertexCoords.set(currentPoint.x, currentPoint.y, -m_height/2.0);
    currentVertices[0] = newVertex(topVertexCoords);
    currentVertices[1] = newVertex(bottomVertexCoords);
    m_topVertices.push_back(currentVertices[0]);
    m_bottomVertices.push_back(currentVertices[1]);

    for(unsigned i = 1; i <= nPoints; i++) {
        lastPoint = currentPoint;
        if(i==nPoints) {
            if(!m_closed)
                break;
            currentPoint = m_points[0];
        } else {
            currentPoint = m_points[i];
        }

        lastVertices[0] = currentVertices[0];
        lastVertices[1] = currentVertices[1];

        // build new vertices and add to mesh
        topVertexCoords = currentPoint;
        topVertexCoords.z = m_height / 2.0;
        bottomVertexCoords = currentPoint;
        bottomVertexCoords.z = -m_height / 2.0;
        if(i < nPoints) {
            currentVertices[0] = newVertex(topVertexCoords);
            currentVertices[1] = newVertex(bottomVertexCoords);
            m_topVertices.push_back(currentVertices[0]);
            m_bottomVertices.push_back(currentVertices[1]);
        } else {
            currentVertices[0] = m_topVertices[0];
            currentVertices[1] = m_bottomVertices[0];
        }

        if(i < nPoints) {
        }

        // then add triangles
        newTriangle(lastVertices[0], currentVertices[0], currentVertices[1]);
        newTriangle(lastVertices[0], currentVertices[1], lastVertices[1]);
    }

    // done with walls

    if(!m_closed)
        return;

    // add caps at top and bottom via triangulation
    Vector2dVector ptVec;

    // add points to Vector2dVector
    for(unsigned i = 0; i < nPoints; i++) {
        ptVec.push_back(Vector2d(m_points[i].x, m_points[i].y));
    }

    // allocate an STL vector to hold the answer.
    Vector2dVector result;

    //  Invoke the triangulator to triangulate this polygon.
    Triangulate::Process(ptVec, result);

    // print out the results.
    int tcount = result.size()/3;

    int pv1 = -1, pv2 = -1, pv3 = -1; // polygon vertices matching points
    int tv1, tv2, tv3, bv1, bv2, bv3; // top and bottom vertices in mesh matching point

    for (int i=0; i<tcount; i++)
    {
        const Vector2d &p1 = result[i*3+0];
        const Vector2d &p2 = result[i*3+1];
        const Vector2d &p3 = result[i*3+2];

        cVector3d pt1, pt2, pt3;
        pt1.set(p1.GetX(), p1.GetY(), m_height/2.0);
        pt2.set(p2.GetX(), p2.GetY(), m_height/2.0);
        pt3.set(p3.GetX(), p3.GetY(), m_height/2.0);

        cVector3d pb1, pb2, pb3;
        pb1.set(p1.GetX(), p1.GetY(), -m_height/2.0);
        pb2.set(p2.GetX(), p2.GetY(), -m_height/2.0);
        pb3.set(p3.GetX(), p3.GetY(), -m_height/2.0);

        bool isClockwise = isTriangleClockwise(p1, p2, p3);
        if(isClockwise) {
            newTriangle(pt1, pt2, pt3);
            newTriangle(pb3, pb2, pb1);
        } else {
            newTriangle(pt3, pt2, pt1);
            newTriangle(pb1, pb2, pb3);
        }
    }

    // draw both sides of every triangle
    setUseCulling(false, true);
}

// if pt matches a polygon vertex, return the index of that point in m_points
// otherwise return -1
int cPlanarObstacle::findPolygonPointMatchingVector2d(const Vector2d &pt) {
    unsigned nPoints = m_points.size();
    for(unsigned i = 0; i < nPoints; i++) {
        if(abs(pt.GetX() - m_points[i].x) < 0.001 &&
           abs(pt.GetY() - m_points[i].y) < 0.001)
            return i;
    }

    return -1;
}

bool cPlanarObstacle::isTriangleClockwise(const Vector2d &p1, const Vector2d &p2, const Vector2d &p3) {
    float accum;
    accum = (p1.GetX() - p2.GetX())*(p1.GetY() + p2.GetY());
    accum = (p2.GetX() - p3.GetX())*(p2.GetY() + p3.GetY());
    accum = (p3.GetX() - p1.GetX())*(p3.GetY() + p1.GetY());

    return accum > 0;
}

///////////////////////////////////////////////////////////////////////////////////
// cEffectPlanarViscousTrap: handles forces that trap cursor upon collision with planar obstacle
///////////////////////////////////////////////////////////////////////////////////

cEffectPlanarViscousTrap::cEffectPlanarViscousTrap(cGenericObject* a_parent):cGenericEffect(a_parent),
m_trapArmed(false),
m_trapTriggered(false),
m_viscosity(0),
m_viscosityRampUpTimeMs(0),
m_stiffness(0)
{
}

void cEffectPlanarViscousTrap::setTrapViscosity(double a_viscosity) {
    m_viscosity = a_viscosity;
}

bool cEffectPlanarViscousTrap::computeForce(const cVector3d& a_toolPos,
                                  const cVector3d& a_toolVel,
                                  const unsigned int& a_toolID,
                                  cVector3d& a_reactionForce)
{
    cPlanarObstacle *parent = (cPlanarObstacle*)m_parent;

    if (parent->m_interactionInside)
    {
        // supposedly we've collided with the obstacle, but just to check,
        // determine whether we are actually located "inside" the obstacle
        cVector3d deltaOrigin = cSub(cProjectPointOnLine(a_toolPos, parent->m_trapOrigin,
            parent->m_trapInwardVec), parent->m_trapOrigin);
        double dot = deltaOrigin.dot(parent->m_trapInwardVec);
        // if dot > 0, then we are on the back face of the obstacle, i.e. actually colliding with it

        cVector3d faceDir = cSub(parent->m_trapLateralB, parent->m_trapLateralA);
        cVector3d projOnFace = cProjectPointOnLine(a_toolPos, parent->m_trapLateralA, faceDir);
        // 0 is left edge, 1 is right edge, outside this bound means off the face
        double normCoordAlongFace = faceDir.dot(cSub(projOnFace, parent->m_trapLateralA)) /
            faceDir.dot(faceDir);

        // we've ACTUALLY collided with the object, the trap is armed, but not yet triggered
        if (dot >= 0 && normCoordAlongFace >= 0 && normCoordAlongFace <= 1 && m_trapArmed && !m_trapTriggered)
        {
            // so trigger the trap
            m_trapTriggered = true;
            //printf("Environment: Planar obstacle triggered!\n");
            // log the triggering time
            m_timeTriggered = chai.simClockNetwork.getCurrentTimeSeconds();

            // store the current position as where we are along the inward trapping vector
            m_maxTrapVectorPosition = parent->m_trapOrigin;

            // store the current position as our point of first contact
            m_trapFirstContact = projOnFace;
        }
    }

    if (m_trapTriggered) {
        double kv = m_viscosity;

        // compute ramp up of viscosity
        if(m_viscosityRampUpTimeMs > 0) {
            double currentTime = chai.simClockNetwork.getCurrentTimeSeconds();
            double deltaMs = (currentTime - m_timeTriggered) * 1000.0;

            double rampFraction = deltaMs / m_viscosityRampUpTimeMs;
            rampFraction = cClamp01(rampFraction);
            //printf("Ramp Fraction = %g\n", rampFraction);
            kv *= rampFraction;
        }

        // apply viscous drag in x, y
        a_reactionForce = cMul(-kv, a_toolVel);
        a_reactionForce.z = 0;

        // compute where we are along the inward trapping vector, which
        // extends along my parent object's m_trapInwardVec, beginning at the maximum
        // position the tool has been along this vector. This creates a rachet and
        // spring effect, where the spring origin follows the tool along progressing forward,
        // but locks when the tool moves back in the other direction and resists this motion

        // this will be a vector along m_trapInwardVec with 0 being m_maxTrapVectorPosition
        cVector3d deltaMax = cSub(cProjectPointOnLine(a_toolPos, m_maxTrapVectorPosition,
            parent->m_trapInwardVec), m_maxTrapVectorPosition);

        // apply spring force (replacing viscosity) in direction of trapInwardVector
        // if now moved below maxTrapVectorPosition
        // or register the new max y if still moving upwards
        double dot = deltaMax.dot(parent->m_trapInwardVec);
        if(dot <= 0) {
            // we've regressed along the trapping vector, compute spring force
            a_reactionForce = -m_stiffness * deltaMax;

        } else {
            // we've advanced along the trapping vector, advance spring origin
            m_maxTrapVectorPosition = cAdd(m_maxTrapVectorPosition, deltaMax);

            parent->m_trapForceVectorVis->setShowEnabled(false, true);
        }

        // apply lateral spring force towards point of initial contact (only parallel to the face of the obstacle)
        cVector3d vRestoring = cSub(a_toolPos, m_trapFirstContact);
        // isolate vector parallel with the lateral extent
        cVector3d vRestoringLateral, perp;
        vRestoring.decompose( cSub(parent->m_trapLateralB, parent->m_trapLateralA), vRestoringLateral, perp);

        // and add in this spring force
        a_reactionForce.add(cMul(-m_stiffness, vRestoringLateral));

        // show a vector from the point of first contact
        // to the point along towards which we are restoring the tool
        // i.e. the furthest point we have reached along the vector parallel to trap inward extending from trapFirstContact
        cVector3d deltaOrigin = cSub(cProjectPointOnLine(m_maxTrapVectorPosition, parent->m_trapOrigin,
            parent->m_trapInwardVec), parent->m_trapOrigin);
        parent->m_trapForceVectorVis->m_pointA = m_trapFirstContact;
        parent->m_trapForceVectorVis->m_pointA.z = 5;
        parent->m_trapForceVectorVis->m_pointB = cAdd(m_trapFirstContact, deltaOrigin);
        parent->m_trapForceVectorVis->m_pointB.z = 5;
        parent->m_trapForceVectorVis->setShowEnabled(true, true);

        return (true);
    }
    else
    {
        // the trap hasn't been triggered
        parent->m_trapForceVectorVis->setShowEnabled(false, true);
        a_reactionForce.zero();
        return (false);
    }
}

void cEffectPlanarViscousTrap::setTrapStiffness(double a_stiffness) {
    m_stiffness = a_stiffness;
}

void cEffectPlanarViscousTrap::setTrapViscosityRampUpTime(double durationMs) {
    m_viscosityRampUpTimeMs = durationMs;
}

void cEffectPlanarViscousTrap::armTrap()
{
    //printf("Haptic: Trap armed!\n");
    m_trapArmed = true;
}

void cEffectPlanarViscousTrap::releaseTrap()
{
    //printf("Haptic: Trap released!\n");
    m_trapArmed = false;
    m_trapTriggered = false;
}

bool cEffectPlanarViscousTrap::isTrapped()
{
    return m_trapTriggered;
}

