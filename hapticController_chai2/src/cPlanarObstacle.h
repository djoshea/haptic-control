#ifndef _C_PLANAR_OBSTACLE_H
#define _C_PLANAR_OBSTACLE_H

#include <vector>
#include "scenegraph/CMesh.h"
#include "scenegraph/CWorld.h"
#include "effects/CEffectMagnet.h"
#include "effects/CEffectViscosity.h"
#include "effects/CEffectVibration.h"
#include "effects/CEffectSurface.h"
#include "cShapeVector.h"
#include "triangulatePolygon.h"

#include "collisions/CCollisionBasics.h"
#include "forces/CInteractionBasics.h"
#include "scenegraph/CGenericObject.h"
#include "effects/CGenericEffect.h"

class cEffectPlanarViscousTrap;

// haptic project which renders forces that restore the tool
// to a bounded plane (i.e. a rectangle surface)
class cPlanarObstacle : public cMesh
{
    public:
        cPlanarObstacle(cWorld* a_world, double a_height,
                const vector<cVector3d> a_points = vector<cVector3d>());
        virtual ~cPlanarObstacle();

        void setPoints(vector<cVector3d> a_points);

        void setCollisionSticky(bool);

        void setClosed(bool);

        // set the internal viscosity of the mesh
        void setViscosity(double);

        // has the trap been triggered by collision with this object
        bool isTrapped();

        // arm the viscous trap effect upon collision with object
        void armTrap();

        // release and disarm the trap effect
        void releaseTrap();

        // set viscosity of the trap effect
        void setTrapViscosity(double a_viscosity);

        // set ramp up time of the viscosity of the trap effect
        void setTrapViscosityRampUpTime(double durationsMs);

        // set viscosity of the trap effect
        void setTrapStiffness(double a_stiffness);

        // both of these are set by setPoints() and used by computeForces() below
        // the vector pointing "into the obstacle" for the viscous trap
        // this is only used for 2 point obstacles (that define a linear absorbing pad)
        cVector3d m_trapInwardVec;
        // this is the same as m_points[0]
        cVector3d m_trapOrigin;

        // these again are the same as m_points[0] and [1],
        // but are used by the trapping effect to laterally constrain the tool within the
        // extent of the plane so it doesn't "slip off" laterally
        cVector3d m_trapLateralA;
        cVector3d m_trapLateralB;

        cShapeVector *m_trapVectorVis;
        cShapeVector *m_trapForceVectorVis;

    protected:
        void rebuildMesh();

        int findPolygonPointMatchingVector2d(const Vector2d &pt);

        bool isTriangleClockwise(const Vector2d &p1, const Vector2d &p2, const Vector2d &p3);

        void setAppearance();

        void setHapticProperties();

        cEffectSurface *m_surface;
        cEffectMagnet *m_magnet;
        cEffectVibration *m_vibration;
        cEffectViscosity *m_viscosity;
        cEffectPlanarViscousTrap *m_collisionTrap;

        bool m_closed;

        bool m_sticky;

        // list of points in x / y that define this
        vector<cVector3d> m_points;
        double m_height;

        // mesh vertex indices corresponding to the polygon vertices in m_points
        vector<int> m_topVertices;
        vector<int> m_bottomVertices;
};

// computes forces for viscous gentle sand-trapping effect
class cEffectPlanarViscousTrap : public cGenericEffect
{

  protected:
      // will the trap trigger upon collision with the object?
      bool m_trapArmed;

      // is the trap active having already been triggered
      bool m_trapTriggered;

      double m_viscosity;

      double m_viscosityRampUpTimeMs;

      // stiffness of spring that keeps the hand from moving down from maxY
      double m_stiffness;

      // maximum distance traveled beyond the trapOrigin along the trapInwardVector of my parent object
      // essentially, how far perpendicularly have we traveled into the 2-point linear absorbing region
      cVector3d m_maxTrapVectorPosition;

      // where did we first encouter the object, so that we can laterally trap the hand there
      cVector3d m_trapFirstContact;

      double m_timeTriggered;

  public:

    cEffectPlanarViscousTrap(cGenericObject * a_parent);
    virtual ~cEffectPlanarViscousTrap() {};

    void setTrapStiffness(double a_stiffness);

    void setTrapViscosity(double a_viscosity);

    void setTrapViscosityRampUpTime(double durationsMs);

    // arm the trap effect so that it will trigger upon collision with the object
    void armTrap();

    // release and disarm trap effect
    void releaseTrap();

    bool isTrapped();

    bool computeForce(const cVector3d& a_toolPos,
                      const cVector3d& a_toolVel,
                      const unsigned int& a_toolID,
                      cVector3d& a_reactionForce);
};

#endif
