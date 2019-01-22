#ifndef _C_MOVE_TO_POINT_H
#define _C_MOVE_TO_POINT_H

#include <vector>
#include "scenegraph/CShapeSphere.h"
#include "scenegraph/CShapeLine.h"
#include "scenegraph/CGenericObject.h"
#include "effects/CGenericEffect.h"
#include "scenegraph/CWorld.h"

// haptic project which renders forces that restore the tool
// to a bounded plane (i.e. a rectangle surface)
class cMoveToPoint : public cGenericObject
{
    public:
        cMoveToPoint(cWorld* a_world);
        virtual ~cMoveToPoint() {};

        void moveToPoint(cVector3d a_location);
        void abort();

        void setAdvanceForce(double a_force);
        double getAdvanceForce();
        void setViscosity(double a_viscosity);
        double getViscosity();

        void setUseZForce(bool a_useZ);
        bool getUseZForce();

        bool m_active;
        bool m_hasStartPoint;
        cVector3d m_endPoint;
        cVector3d m_startPoint;
        cShapeSphere *m_targetSphere;
        cShapeLine *m_targetLine;

    protected:
        cWorld* m_parentWorld;
        bool m_useZ;

        double m_advanceForce;

        double m_viscosity;
};

// computes forces for cMoveToPoint objects
class cEffectMoveToPoint : public cGenericEffect
{
  public:

    cEffectMoveToPoint(cMoveToPoint *a_parent);
    virtual ~cEffectMoveToPoint() {};

    bool computeForce(const cVector3d& a_toolPos,
                      const cVector3d& a_toolVel,
                      const unsigned int& a_toolID,
                      cVector3d& a_reactionForce);
};

#endif
