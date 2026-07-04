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

	const double emptyIxx;
	const double emptyIyy;
	const double emptyIzz;

	double currentFuelMass;
	
	State currentState;

	Derivative evaluate(const State& state, const Derivative& d, double dt) const;

	Matrix3x3 getInertiaTensor() const;

	Matrix3x3 getInverseInertiaTensor() const;

public:

	Rocket(double dry_m, double fuel_m, double s_ref, double l_ref, double magnitude, double flow_rate, double Ixx, double Iyy, double Izz, const State& initial_state);

	double getTotalMass() const;

	State getState() const { return currentState; }
	void setState(const State& newState) { currentState = newState; }

	void integrate(double dt);
};