//
//  cErrorClamp.h
//  chai3d
//
//  Created by Xulu Sun on 12/16/16.
//
//

#ifndef _C_ERROR_CLAMP_H
#define _C_ERROR_CLAMP_H


#include "world/CMesh.h"
#include "world/CWorld.h"
#include "forces/CInteractionBasics.h"
#include "world/CGenericObject.h"
#include "effects/CGenericEffect.h"
#include "graphics/CRenderOptions.h"
#include "geometry.h"


using namespace chai3d;

class cErrorClamp;
#define WORKSPACE_WIDTH 250
#define WORKSPACE_HEIGHT 250
#define WALL_STIFFNESS 0.8
#define WALL_VISCOSITY 0.06
#define WALL_WIDTH 2.5

// computes forces for cErrorClamp objects
class cEffectErrorClamp : public cGenericEffect
{
public:

    cEffectErrorClamp(cErrorClamp* a_parent);
    virtual ~cEffectErrorClamp() {};

    bool m_waiting;
    int count;
    bool computeForce(const cVector3d& a_toolPos,
                      const cVector3d& a_toolVel,
                      const unsigned int& a_toolID,
                      cVector3d& a_clampForce);

};


class cErrorClamp : public cGenericObject
{

public:
    cErrorClamp(cWorld* a_world);

    void setTransparencyLevel(double);

    virtual void render(cRenderOptions &a_options);

    bool m_clampActive;
    double m_dampingGain;
    double m_springGain;
  

    cShapeVector *m_vector;

    // perturbation state info
    cVector3d m_pulseForce;  // to store knowledge of the perturbation pulse to tightly couple mass changes to perturbation
    cVector3d m_wallPos1;
    cVector3d m_wallPos2;
    cVector3d m_wallOrientation;

    void setPerturbationForce( cVector3d pulseForce);

    void setClampActive();
    void setClampInactive();
    // void setDampingGain(double a_dampingGain);
    // void setSpringGain(double a_springGain);
    void setWallPositions(cVector3d wallPosition1, cVector3d wallPosition2);
    void setWallOrientation(cVector3d wallOrientation);
    void setDampingGain(double a_dampingGain);
    void setSpringGain(double a_springGain);
    double getDampingGain();
    double getSpringGain();


    void computeForce(const cVector3d& a_toolPos,
                      const cVector3d& a_toolVel,
                      const unsigned int& a_toolID,
                      cVector3d& a_clampForce);




protected:
    cWorld* m_parentWorld;
    cEffectErrorClamp* m_effect;
};


#endif
