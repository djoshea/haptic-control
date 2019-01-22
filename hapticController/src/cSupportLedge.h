#ifndef _C_SUPPORT_LEDGE_H
#define _C_SUPPORT_LEDGE_H

#include "scenegraph/CMesh.h"
#include "scenegraph/CWorld.h"
#include "collisions/CCollisionBasics.h"
#include "forces/CInteractionBasics.h"
#include "scenegraph/CGenericObject.h"
#include "effects/CGenericEffect.h"
#include "geometry.h"

// haptic project which renders forces that restore the tool
// to a bounded plane (i.e. a rectangle surface)
class cSupportLedge : public cGenericObject
{
    public:
        cSupportLedge(cWorld* a_world, double a_height, double a_width, double a_maxForce);

        void setTransparencyLevel(double);

        virtual void render(const int a_renderMode=0);

        void setViscosity(double Kv);
        double getViscosity();

        cMesh *m_planeMesh;
        double m_height; // in z
        double m_width; // in x
        double m_maxForce;
        double m_viscosity;

    protected:
        cWorld* m_parentWorld;
};

// computes forces for cSupportLedge objects
class cEffectSupportLedge : public cGenericEffect
{
  public:

    cEffectSupportLedge(cSupportLedge* a_parent);
    virtual ~cEffectSupportLedge() {};

    bool computeForce(const cVector3d& a_toolPos,
                      const cVector3d& a_toolVel,
                      const unsigned int& a_toolID,
                      cVector3d& a_reactionForce);
};

#endif
