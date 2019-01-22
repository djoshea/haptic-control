//
//  cCurlForceField.h
//  chai3d
//
//  Created by Xulu Sun on 6/15/16.
//
//

#ifndef _C_CURL_FORCE_FIELD_H
#define _C_CURL_FORCE_FIELD_H


#include "world/CMesh.h"
#include "world/CWorld.h"
#include "forces/CInteractionBasics.h"
#include "world/CGenericObject.h"
#include "effects/CGenericEffect.h"
#include "graphics/CRenderOptions.h"
#include "geometry.h"

using namespace chai3d;

class cCurlForceField;

// computes forces for cCurlForceField objects
class cEffectCurlForceField : public cGenericEffect
{
public:

    cEffectCurlForceField(cCurlForceField* a_parent);
    virtual ~cEffectCurlForceField() {};

    bool m_waiting;
    int count;
    bool computeForce(const cVector3d& a_toolPos,
                      const cVector3d& a_toolVel,
                      const unsigned int& a_toolID,
                      cVector3d& a_curlForce);

};


class cCurlForceField : public cGenericObject
{

public:
    cCurlForceField(cWorld* a_world, double a_maxForce);

    void setTransparencyLevel(double);

    virtual void render(cRenderOptions &a_options);

    void setXGainVector(cVector3d a_xGainVector);
    cVector3d getXGainVector();

    void setYGainVector(cVector3d a_yGainVector);
    cVector3d getYGainVector();

    bool m_fieldActive;

    cShapeVector *m_vector;
    double m_maxForce;

    // perturbation state info
    cVector3d m_pulseForce;  // to store knowledge of the perturbation pulse to tightly couple mass changes to perturbation
    void setPerturbationForce( cVector3d pulseForce);

    void setFieldActive();
    void setFieldInactive();

    void computeForce(const cVector3d& a_toolPos,
                      const cVector3d& a_toolVel,
                      const unsigned int& a_toolID,
                      cVector3d& a_curlForce);



    cVector3d m_xGainVector;
    cVector3d m_yGainVector;


protected:
    cWorld* m_parentWorld;
    cEffectCurlForceField* m_effect;
};


#endif









//
//
//#ifndef _C_CURL_FIELD_H
//#define _C_CURL_FIELD_H
//
//
//#include "world/CMesh.h"
//#include "world/CWorld.h"
//#include "forces/CInteractionBasics.h"
//#include "world/CGenericObject.h"
//#include "effects/CGenericEffect.h"
//#include "graphics/CRenderOptions.h"
//#include "geometry.h"
//
//using namespace chai3d;
//
//class cCurlField : public cGenericObject
//{
//
//	public:
//		cCurlfield(cWorld* a_world, double a_curlGain);
//
//		//virtual void render(cRenderOptions& a_options);
//
//		void setCurlGain(double curlGain);
//		double getCurlGain();
//
//		void computeForce(const cVector3d& a_toolPos,
//				const cVector3d& a_toolVel,
//				const unsigned int& a_toolID,
//				cVector3d& a_reactionForce);
//
//		double m_curlGain;
//
//	protected:
//		cWorld* m_parentWorld;
//};
//
//// computes forces for cDirectionalViscosity objects
//class cEffectCurlField : public cGenericEffect
//{
//	public:
//
//		cEffectCurlField(cCurlField* a_parent);
//		virtual ~cEffectCurlField() {};
//
//		bool computeForce(const cVector3d& a_toolPos,
//		                      const cVector3d& a_toolVel,
//		                      const unsigned int& a_toolID,
//		                      cVector3d& a_reactionForce);
//
//};
//
//#endif
