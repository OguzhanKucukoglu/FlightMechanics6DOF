#pragma once
#include "dynamics/State.h"
#include "math/Matrix3x3.h"
#include "math/LookupTable1D.h"

class Rocket {
private:
	const double dryMass;
	const double referenceArea; // S_ref (m^2)
	const double referenceLength; // L_ref (m)
	LookupTable1D thrustCurve; // Zaman vs Ýtki tablosu
	const double specificImpulse; // Motorun verimi
	const double dragCoefficient;

	const double emptyIxx, emptyIyy, emptyIzz;
	double emptyIxy = 0.0, emptyIxz = 0.0, emptyIyz = 0.0;

	const double fullIxx, fullIyy, fullIzz;
	double fullIxy = 0.0, fullIxz = 0.0, fullIyz = 0.0;

	const double emptyCG_X;
	const double fullCG_X;

	const double engine_X;
	double gimbal_Y;
	double gimbal_Z;
	
	const double initialFuelMass;

	Vector3D currentAcceleration;
	
	State currentState;

	Derivative evaluate(const State& state, const Derivative& d, double dt) const;

	Matrix3x3 getInertiaTensor(double currentFuel) const;

	Matrix3x3 getInverseInertiaTensor(double currentFuel) const;

public:

	Rocket(double dry_m, double fuel_m, double s_ref, double l_ref, double isp, double drag_coeff, double empty_ixx, double empty_iyy, double empty_izz, double full_ixx, double full_iyy, double full_izz, double empty_cg_x, double full_cg_x, double engineX, const State& initial_state);

	void setThrustCurve(const LookupTable1D& curve) {
		thrustCurve = curve;
	}

	double getCurrentCG(double currentFuel) const;

	State getState() const { return currentState; }
	void setState(const State& newState) { currentState = newState; }

	void integrate(double dt);

	Vector3D getCurrentAcceleration() const { return currentAcceleration; }

	void setGimbal(double pitch, double yaw) {
		gimbal_Y = pitch;
		gimbal_Z = yaw;
	}
};