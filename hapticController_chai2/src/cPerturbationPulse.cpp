#include "math/CConstants.h"
#include "scenegraph/CMesh.h"
#include "math/CVector3d.h"
#include "math/CMaths.h"
#include "scenegraph/CGenericObject.h"

#include "geometry.h"
#include "environment.h"
#include "cPerturbationPulse.h"

extern ChaiData chai;

cPerturbationPulse::cPerturbationPulse(cWorld* a_world, double a_maxForce) :
m_maxForce(a_maxForce)
{
    m_parentWorld = a_world;

    // build a mesh object to visually represent the ledge
    m_planeMesh = new cMesh(a_world);

    m_pulseDurationMs = 0;
    m_pulseStartTime = 0;
    m_pulseActive = false;

    // for visualizing perturbation
    m_vector = new cShapeVector(cVector3d(0,0,0), cVector3d(0,0,0), 5, 5);
    m_vector->m_ColorPointA.set(1.0, 0.3, 0.3, 1.0);
    m_vector->m_ColorPointB.set(1.0, 0.3, 0.3, 1.0);
    m_vector->setHapticEnabled(false, true);

    cEffectPerturbationPulse *effect = new cEffectPerturbationPulse(this);
    addEffect(effect);
}

void cPerturbationPulse::render(const int a_renderMode)
{
    m_vector->renderSceneGraph(a_renderMode);
}

void cPerturbationPulse::startPulse() {
    m_pulseActive = true;
    m_pulseRampingDown = false;
    m_pulseStartTime = chai.simClockNetwork.getCurrentTimeSeconds();
    printf("PERTURBATION: START at %g\n", m_pulseStartTime);
}

void cPerturbationPulse::abort() {
    m_pulseActive = false;
    if(m_pulseActive)
        printf("PERTURBATION: ABORT\n");
}

void cPerturbationPulse::rampDown(double durationMs) {
    if (!m_pulseActive) return;
    if (m_pulseRampingDown) return;
    m_pulseRampingDown = true;
    m_pulseRampDownStartTime = chai.simClockNetwork.getCurrentTimeSeconds();
    m_pulseRampDownDurationMs = durationMs;
    printf("PERTURBATION: Ramping down over %g ms\n", durationMs);
}

///////////////////////////////////////////////////////////////////////////////////
// cEffectPerturbationPulse: handles force computations for the cPerturbationPulse class
///////////////////////////////////////////////////////////////////////////////////

cEffectPerturbationPulse::cEffectPerturbationPulse(cPerturbationPulse* a_parent):cGenericEffect(a_parent)
{
}

bool cEffectPerturbationPulse::computeForce(const cVector3d& a_toolPos,
                                  const cVector3d& a_toolVel,
                                  const unsigned int& a_toolID,
                                  cVector3d& a_reactionForce)
{
    cPerturbationPulse *parent = (cPerturbationPulse*)m_parent;
    double currentTime = chai.simClockNetwork.getCurrentTimeSeconds();

    if(parent->m_pulseActive) {
        double deltaMs = (currentTime - parent->m_pulseStartTime) * 1000.0;

        if(deltaMs >= parent->m_pulseDurationMs) {
            // perturbation finished
            printf("PERTURBATION: END\n");
            parent->m_pulseActive = false;
            parent->m_vector->setShowEnabled(false, true);

            a_reactionForce.zero();
            return false;
        } else {
            // perturbation underway
            cVector3d localForce = parent->m_pulseForce;
            if(localForce.length() > parent->m_maxForce) {
                localForce.normalize();
                localForce.mul(parent->m_maxForce);
            }

            if(parent->m_pulseRampingDown) {
                // gradually reduce magnitude linearly
                double rampDeltaMs = (currentTime - parent->m_pulseRampDownStartTime) * 1000.0;
                double rampFraction = 0;

                if (parent->m_pulseRampDownDurationMs > 0)
                    rampFraction = rampDeltaMs / (parent->m_pulseRampDownDurationMs);
                else
                    rampFraction = 1;

                localForce.mul(cClamp01(rampFraction));

                if(rampDeltaMs >= parent->m_pulseRampDownDurationMs) {
                    printf("PERTURBATION: RAMP DOWN FINISHED\n");
                    parent->m_pulseActive = false;
                    parent->m_vector->setShowEnabled(false, true);
                    a_reactionForce.zero();
                    return false;
                }
            }

            double gain = 10;
            parent->m_vector->m_pointA = a_toolPos;
            parent->m_vector->m_pointB = cAdd(a_toolPos, localForce * gain);

            /* move arrow above screen plane so visible */
            parent->m_vector->m_pointA.z = 1;
            parent->m_vector->m_pointB.z = 1;

            parent->m_vector->setShowEnabled(true, true);

            a_reactionForce = localForce;
           // a_reactionForce.zero();
            return true;
       }

    } else {
        // perturbation inactive
        parent->m_vector->setShowEnabled(false, true);
        a_reactionForce.zero();
        return false;
    }
}
