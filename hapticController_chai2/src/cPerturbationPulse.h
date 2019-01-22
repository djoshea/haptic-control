#ifndef _C_PERTURBATION_PULSE_H
#define _C_PERTURBATION_PULSE_H

#include "scenegraph/CMesh.h"
#include "scenegraph/CWorld.h"
#include "collisions/CCollisionBasics.h"
#include "forces/CInteractionBasics.h"
#include "scenegraph/CGenericObject.h"
#include "effects/CGenericEffect.h"
#include "cShapeVector.h"
#include "geometry.h"

// haptic project which renders forces that restore the tool
// to a bounded plane (i.e. a rectangle surface)
class cPerturbationPulse : public cGenericObject
{
    public:
        cPerturbationPulse(cWorld* a_world, double a_maxForce);

        void setTransparencyLevel(double);

        virtual void render(const int a_renderMode=0);

        void startPulse();
        void abort();
        void rampDown(double durationMs);

        cMesh *m_planeMesh;

        cShapeVector *m_vector;
        cVector3d m_pulseForce;
        double m_maxForce;
        double m_pulseDurationMs;
        bool m_pulseActive;
        double m_pulseStartTime;

        // for ramping down the force at the end of the pulse
        bool m_pulseRampingDown;
        double m_pulseRampDownStartTime;
        double m_pulseRampDownDurationMs;

    protected:
        cWorld* m_parentWorld;
};

// computes forces for cPerturbationPulse objects
class cEffectPerturbationPulse : public cGenericEffect
{
  public:

    cEffectPerturbationPulse(cPerturbationPulse* a_parent);
    virtual ~cEffectPerturbationPulse() {};

    bool computeForce(const cVector3d& a_toolPos,
                      const cVector3d& a_toolVel,
                      const unsigned int& a_toolID,
                      cVector3d& a_reactionForce);
};

#endif
