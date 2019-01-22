
#ifndef _C_DIRECTIONAL_VISCOSITY_H
#define _C_DIRECTIONAL_VISCOSITY_H


#include "world/CMesh.h"
#include "world/CWorld.h"
#include "forces/CInteractionBasics.h"
#include "world/CGenericObject.h"
#include "effects/CGenericEffect.h"
#include "graphics/CRenderOptions.h"
#include "geometry.h"

using namespace chai3d;

class cDirectionalViscosity : public cGenericObject
{

	public:
		cDirectionalViscosity(cWorld* a_world, double a_dragCoefficient);

		//virtual void render(cRenderOptions& a_options);

		void setDirectionalViscosityCoeff(double dragCoeff);
		double getDirectionalViscosityCoeff();

		void computeForce(const cVector3d& a_toolPos,
				const cVector3d& a_toolVel,
				const unsigned int& a_toolID,
				cVector3d& a_reactionForce);

		double m_directionalViscosityCoeff;

		void setDragActive();
		void setDragInactive();

		bool m_active;

	protected:
		cWorld* m_parentWorld;
};

// computes forces for cDirectionalViscosity objects
class cEffectDirectionalViscosity : public cGenericEffect
{
	public:

		cEffectDirectionalViscosity(cDirectionalViscosity* a_parent);
		virtual ~cEffectDirectionalViscosity() {};

		bool computeForce(const cVector3d& a_toolPos,
		                      const cVector3d& a_toolVel,
		                      const unsigned int& a_toolID,
		                      cVector3d& a_reactionForce);

};

#endif
