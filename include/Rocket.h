#pragma once
#include "State.h"
#include "Matrix3x3.h"

class Rocket {
private:
	const double dryMass;
	const double referenceArea; // S_ref (m^2)
	const double referenceLength; // L_ref (m)
	const double thrustMagnitude; // Motor itki kuvveti (Newton)
	const double massFlowRate; // Saniyede tüketilen yakýt (kg/s)
	const double dragCoefficient;

	const double emptyIxx, emptyIyy, emptyIzz;
	const double fullIxx, fullIyy, fullIzz;

	const double emptyCG_X;
	const double fullCG_X;

	const double engine_X;
	double gimbal_Y;
	double gimbal_Z;
	
	double currentFuelMass;
	const double initialFuelMass;

	Vector3D currentAcceleration;
	
	State currentState;

	Derivative evaluate(const State& state, const Derivative& d, double dt) const;

	Matrix3x3 getInertiaTensor() const;

	Matrix3x3 getInverseInertiaTensor() const;

public:

	Rocket(double dry_m, double fuel_m, double s_ref, double l_ref, double magnitude, double flow_rate, double drag_coeff, double empty_ixx, double empty_iyy, double empty_izz, double full_ixx, double full_iyy, double full_izz, double empty_cg_x, double full_cg_x, double engineX, const State& initial_state);

	double getTotalMass() const;

	double getCurrentCG() const;

	State getState() const { return currentState; }
	void setState(const State& newState) { currentState = newState; }

	void integrate(double dt);

	Vector3D getCurrentAcceleration() const { return currentAcceleration; }
};