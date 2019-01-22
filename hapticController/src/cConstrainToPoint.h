#ifndef _C_CONSTRAIN_TO_POINT_H
#define _C_CONSTRAIN_TO_POINT_H

#include <vector>
#include "scenegraph/CShapeSphere.h"
#include "scenegraph/CShapeLine.h"
#include "scenegraph/CGenericObject.h"
#include "effects/CGenericEffect.h"
#include "scenegraph/CWorld.h"

// haptic project which renders forces that restore the tool
// to a bounded plane (i.e. a rectangle surface)
class cConstrainToPoint : public cGenericObject
{
    public:
        cConstrainToPoint(cWorld* a_world);
        virtual ~cConstrainToPoint() {};

        void constrainToPoint(cVector3d a_location);
        void abort();

        void setUseZForce(bool a_useZ);
        bool getUseZForce();

        bool m_active;
        cShapeSphere *m_targetSphere;
        cVector3d m_point;

    protected:
        cWorld* m_parentWorld;
        bool m_useZ;
};

// computes forces for cMoveToPoint objects
class cEffectConstrainToPoint : public cGenericEffect
{
  public:

    cEffectConstrainToPoint(cConstrainToPoint *a_parent);
    virtual ~cEffectConstrainToPoint() {};

    bool computeForce(const cVector3d& a_toolPos,
                      const cVector3d& a_toolVel,
                      const unsigned int& a_toolID,
                      cVector3d& a_reactionForce);
};

#endif
