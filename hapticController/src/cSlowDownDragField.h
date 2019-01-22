#ifndef _C_SLOW_DOWN_DRAG_FIELD_H
#define _C_SLOW_DOWN_DRAG_FIELD_H

#include <vector>
#include "scenegraph/CShapeSphere.h"
#include "scenegraph/CShapeLine.h"
#include "scenegraph/CGenericObject.h"
#include "effects/CGenericEffect.h"
#include "scenegraph/CWorld.h"

class cSlowDownDragField : public cGenericObject
{

public:
    cSlowDownDragField(cWorld* a_world);

    virtual void render(const int a_renderMode=0);

    cShapeVector *m_vector;
    bool m_fieldActive;
    bool m_useZ;
    double m_maxForce; // the max force applied
    double m_viscosity; // the kv constant for the drag field
    double m_activateSpeed; // the speed at which the field activates

    // perturbation state info
    void setFieldActive();
    void setFieldInactive();
    bool getFieldActive();

    void setMaxForce(double a_maxForce);
    double getMaxForce();

    void setViscosity(double a_viscosity);
    double getViscosity();

    void setActivateSpeed(double a_activateSpeed);
    double getActivateSpeed();

    void setUseZForce(bool a_useZ);
    bool getUseZForce();

    void computeForce(const cVector3d& a_toolPos,
                      const cVector3d& a_toolVel,
                      const unsigned int& a_toolID,
                      cVector3d& a_reactionForce);

protected:
    cWorld* m_parentWorld;
};

class cEffectSlowDownDragField : public cGenericEffect
{
public:

    cEffectSlowDownDragField(cSlowDownDragField* a_parent);
    virtual ~cEffectSlowDownDragField() {};

    bool computeForce(const cVector3d& a_toolPos,
                      const cVector3d& a_toolVel,
                      const unsigned int& a_toolID,
                      cVector3d& a_reactionForce);
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
