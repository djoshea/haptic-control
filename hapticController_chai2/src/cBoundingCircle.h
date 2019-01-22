#ifndef _C_BOUNDING_CIRCLE_H_
#define _C_BOUNDING_CIRCLE_H_

#include "scenegraph/CMesh.h"
#include "scenegraph/CWorld.h"
#include "scenegraph/CShapeSphere.h"
#include "collisions/CCollisionBasics.h"
#include "forces/CInteractionBasics.h"
#include "scenegraph/CGenericObject.h"
#include "effects/CGenericEffect.h"
#include "geometry.h"

#ifndef _DEFINED_ONPLANE_ENUM_
enum onPlaneStateType { ONPLANE, RETURNING };
#define _DEFINED_ONPLANE_ENUM_
#endif

// utility class for performing geometry
class cBoundedCircle3d : public cPlane3d
{
    public:
    double radius;

    cBoundedCircle3d() : cPlane3d(), radius(0) {}

    cBoundedCircle3d(const cVector3d &_vec1, const cVector3d &_vec2, const cVector3d &_thru,
        double _radius) :
        cPlane3d(_vec1, _vec2, _thru), radius(_radius) {}

    void copyfrom(const cBoundedCircle3d &src) {
        cPlane3d::copyfrom(src);
        radius = src.radius;
    }

    inline void boundedClosestPoint(const cVector3d &pt, cVector3d &closest) const {
        cVector3d coords;
        this->projectIntoCoords(pt, coords);

        // project point down into plane
        coords.z = 0;
        double current_length = coords.length();

        // constrain to lie in circle with radius r
        if (current_length > radius)
        {
        	coords.normalize();
        	coords.mul(radius);
    	}

        // and lie on plane
        coords.z = 0;

        this->pointFromCoords(coords, closest);
    }

    // returns the distance squared to the workspace edge. If inside workspace bounds,
    // distance is positive, if outside, distance is negative
    inline double signedDistanceSquaredFromEdge(const cVector3d &pt) const {
        cVector3d coords;
        this->projectIntoCoords(pt, coords);

        coords.z = 0;
        double signedDistance = radius - coords.length();
        return signedDistance * signedDistance;
    }

    // get a ray pointing from a point on this bounded plane to the point pt
    // also returns bounded distance signed as below
    inline double boundedRayToPoint(const cVector3d &pt, cRay3d &ray) const {
        cVector3d closest;
        this->boundedClosestPoint(pt, closest);

        ray.m_origin = closest;
        ray.m_direction = pt - closest;

        double boundedDistance = ray.m_direction.length();
        double signedDistance = this->distanceToPointSigned(pt);

        if(signedDistance < 0)
            boundedDistance = -boundedDistance;

        return boundedDistance;
    }

    // return the distance from this bounded plane to the point, negative if outside
    inline double boundedDistanceToPointSigned(const cVector3d &pt) const {
        cVector3d closest, vec;
        this->boundedClosestPoint(pt, closest);
        vec = pt - closest;
        double boundedDistance = vec.length();
        double signedDistance = this->distanceToPointSigned(pt);

        if(signedDistance < 0)
            boundedDistance = -boundedDistance;

        return boundedDistance;
    }

    void addToMesh(cMesh* mesh) const {
        //printf("Adding plane to mesh:");
        //this->printCorners();

    	const int Nvertices = 36;
        int vertices[Nvertices];
        cVector3d vertexCoords[Nvertices];

        int centerVertex;
        cVector3d centerCoords;

        cVector3d temp;
        temp.set(0, 0, 0);

        // create center vertex
        this->pointFromCoords(temp, centerCoords);
        centerVertex = mesh->newVertex(centerCoords);

        // generate coordinates for vertices
        double x;
        double y;
        double angle;
        for(int i = 0; i < Nvertices; i++) {
        	angle = 2.0*CHAI_PI * (double)i / (double)Nvertices;
        	x = radius * cCosRad(angle);
        	y = radius * cSinRad(angle);
        	temp.set(x, y, 0);
        	this->pointFromCoords(temp, vertexCoords[i]);
        	vertices[i] = mesh->newVertex(vertexCoords[i]);
        }

        for(int i = 0; i < Nvertices; i++) {
        	// then add triangles, center -> vertex -> next vertex
        	mesh->newTriangle(centerVertex, vertices[i], vertices[(i+1) % Nvertices]);
        }
    }

};


// haptic object which renders forces that restore the tool
// to a circularly bounded plane (i.e. a circular surface)
class cBoundingCircle : public cGenericObject
{
    public:
        cBoundingCircle(cWorld* a_world, double radius);
        virtual ~cBoundingCircle();

        void setViscosity(double Kv);
        double getViscosity();

        void setViscosityReturning(double Kv);
        double getViscosityReturning();

        void setStiffnessReturning(double Kv);
        double getStiffnessReturning();

        void setDistanceLeavePlane(double v) {
            m_distanceLeavePlane = v;
        }

        void setDistanceReturnPlane(double v) {
            m_distanceReturnPlane = v;
        }

        void setDistanceForceSaturation(double v) {
            m_forceSaturationDistance = v;
        }

        void setTransparencyLevel(double alpha);

        bool getIsToolAtEdge();

        bool getIsToolOnPlane();

        double getSignedDistanceSquaredFromEdge();

        friend class cEffectBoundingCircle;

    protected:
        cBoundedCircle3d *m_bplane;
        cShapeSphere *m_proxyPoint;
        cMesh *m_planeMesh;
        onPlaneStateType m_state;

        cWorld* m_parentWorld;
        double m_radius;
        double m_viscosity;
        double m_viscosity_returning;
        double m_stiffness_returning;

        // parameters for shmidt trigger
        // low stiffness when far away from plane all the way back to lower threshold distance
        // but then high stiffness out from there to higher threshold distance
        double m_distanceLeavePlane;
        double m_distanceReturnPlane;
        double m_forceSaturationDistance;

        bool m_toolAtEdge;
        double m_distFromEdge;
};

// computes forces for cBoundingCircle objects
class cEffectBoundingCircle : public cGenericEffect
{
  public:

	cEffectBoundingCircle(cBoundingCircle* a_parent);
    virtual ~cEffectBoundingCircle() {};

    bool computeForce(const cVector3d& a_toolPos,
                      const cVector3d& a_toolVel,
                      const unsigned int& a_toolID,
                      cVector3d& a_reactionForce);
};

#endif
