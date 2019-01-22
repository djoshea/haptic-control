#ifndef _C_DIRECTIONAL_MASS_H
#define _C_DIRECTIONAL_MASS_H


#include "world/CMesh.h"
#include "world/CWorld.h"
#include "forces/CInteractionBasics.h"
#include "world/CGenericObject.h"
#include "effects/CGenericEffect.h"
#include "graphics/CRenderOptions.h"
#include "geometry.h"

using namespace chai3d;

class cDirectionalMass : public cGenericObject
{

	public:
		cDirectionalMass(cWorld* a_world, cVector3d a_massCoeffVector, cVector3d a_dragCoeffVector, double a_maxForce);

		void setTransparencyLevel(double);

		virtual void render(cRenderOptions &a_options);

		void setDirectionalMassCoeff(cVector3d massCoeff);
		cVector3d getDirectionalMassCoeff();

		void setDirectionalMassDragCoeff(cVector3d dragCoeff);
		cVector3d getDirectionalMassDragCoeff();

		bool m_massActive;  // NOTE this does nothing right now

		cShapeVector *m_vector;
		double m_maxForce;

		// perturbation state info
		cVector3d m_pulseForce;  // to store knowledge of the perturbation pulse to tightly couple mass changes to perturbation
		void setPerturbationForce( cVector3d pulseForce);

		void setMassActive();
		void setMassInactive();

		void computeForce(const cVector3d& a_toolPos,
				const cVector3d& a_toolVel,
				const unsigned int& a_toolID,
				cVector3d& a_reactionForce);



		cVector3d m_directionalMassVector;
		cVector3d m_directionalDragVector;

//		double m_directionalMassCoeffX;
//		double m_directionalMassDragCoeffX;
//
//		double m_directionalMassCoeffY;
//		double m_directionalMassDragCoeffY;

	protected:
		cWorld* m_parentWorld;
};

// computes forces for cDirectionalMass objects
class cEffectDirectionalMass : public cGenericEffect
{
	public:

		cEffectDirectionalMass(cDirectionalMass* a_parent);
		virtual ~cEffectDirectionalMass() {};

		bool computeForce(const cVector3d& a_toolPos,
		                      const cVector3d& a_toolVel,
		                      const unsigned int& a_toolID,
		                      cVector3d& a_reactionForce);

};

#endif
