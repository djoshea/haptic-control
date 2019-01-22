#ifndef _C_BOUNDING_PLANE_H_
#define _C_BOUNDING_PLANE_H_

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

// haptic project which renders forces that restore the tool
// to a bounded plane (i.e. a rectangle surface)
class cBoundingPlane : public cGenericObject
{
    public:
        cBoundingPlane(cWorld* a_world, double szX, double szY);
        virtual ~cBoundingPlane();

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

        friend class cEffectBoundingPlane;

    protected:
        cBoundedPlane3d *m_bplane;
        cShapeSphere *m_proxyPoint;
        cMesh *m_planeMesh;
        onPlaneStateType m_state;

        cWorld* m_parentWorld;
        double m_sizeX;
        double m_sizeY;
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

// computes forces for cBoundingPlane objects
class cEffectBoundingPlane : public cGenericEffect
{
  public:

    cEffectBoundingPlane(cBoundingPlane* a_parent);
    virtual ~cEffectBoundingPlane() {};

    bool computeForce(const cVector3d& a_toolPos,
                      const cVector3d& a_toolVel,
                      const unsigned int& a_toolID,
                      cVector3d& a_reactionForce);
};

#endif
