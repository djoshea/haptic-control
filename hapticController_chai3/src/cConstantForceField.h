
#ifndef _C_CONSTANT_FORCE_FIELD_H
#define _C_CONSTANT_FORCE_FIELD_H


#include "world/CMesh.h"
#include "world/CWorld.h"
#include "forces/CInteractionBasics.h"
#include "world/CGenericObject.h"
#include "effects/CGenericEffect.h"
#include "graphics/CRenderOptions.h"
#include "geometry.h"

using namespace chai3d;

class cConstantForceField : public cGenericObject
{

	public:
		cConstantForceField(cWorld* a_world, double a_forceMagnitude, double a_forceDirection);

		//virtual void render(cRenderOptions& a_options);

		void setForce(double forceMagnitude, double forceDirection);
		double getForceMagnitude();
		double getForceDirection();
		double getForceX();
		double getForceY();

		void computeForce(const cVector3d& a_toolPos,
				const cVector3d& a_toolVel,
				const unsigned int& a_toolID,
				cVector3d& a_reactionForce);

		double m_forceMagnitude;
		double m_forceDirection;

		void setForceFieldActive();
		void setForceFieldInactive();
		bool m_active;

	protected:
		cWorld* m_parentWorld;

		double m_forceX;
		double m_forceY;
};

// computes forces for cConstantForceField objects
class cEffectConstantForceField : public cGenericEffect
{
	public:

		cEffectConstantForceField(cConstantForceField* a_parent);
		virtual ~cEffectConstantForceField() {};

		bool computeForce(const cVector3d& a_toolPos,
		                      const cVector3d& a_toolVel,
		                      const unsigned int& a_toolID,
		                      cVector3d& a_reactionForce);

};


#endif
