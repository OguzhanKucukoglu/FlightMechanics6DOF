#include "Rocket.h"

Rocket::Rocket(double dry_m, double fuel_m, double s_ref, double l_ref, double Ixx, double Iyy, double Izz, const State& initial_state)
	: dryMass(dry_m),
	referenceArea(s_ref),
	referenceLength(l_ref),
	emptyIxx(Ixx),
	emptyIyy(Iyy),
	emptyIzz(Izz),
	currentFuelMass(fuel_m),
	currentState(initial_state)
{ }

double Rocket::getTotalMass() const {

	return dryMass + currentFuelMass;
}

Derivative Rocket::evaluate(const State& initial, const Derivative& d, double dt) const {

	State state = stepState(initial, d, dt);

	Derivative output;

	output.velocity = state.velocity;

	output.spin.w = 0.5 * (-state.orientation.x * state.angularVelocity.x - state.orientation.y * state.angularVelocity.y - state.orientation.z * state.angularVelocity.z);
	output.spin.x = 0.5 * (state.orientation.w * state.angularVelocity.x + state.orientation.y * state.angularVelocity.z - state.orientation.z * state.angularVelocity.y);
	output.spin.y = 0.5 * (state.orientation.w * state.angularVelocity.y - state.orientation.x * state.angularVelocity.z + state.orientation.z * state.angularVelocity.x);
	output.spin.z = 0.5 * (state.orientation.w * state.angularVelocity.z + state.orientation.x * state.angularVelocity.y - state.orientation.y * state.angularVelocity.x);

	Vector3D gravity(0.0, -9.81, 0.0);
	Vector3D netForce = gravity;

	double mass = getTotalMass();
	output.acceleration = netForce * (1.0 / mass);

	Vector3D netTorque(0.0, 0.0, 0.0);
	Matrix3x3 inverseTensor = getInverseInertiaTensor();
	output.angularAcceleration = inverseTensor * netTorque;

	return output;
}

void Rocket::integrate(double dt) {

	Derivative zeroDerivative = { Vector3D(0,0,0), Vector3D(0,0,0), Vector3D(0,0,0), Quaternion(0,0,0,0) };

	Derivative k1 = evaluate(currentState, zeroDerivative, 0.0);
	Derivative k2 = evaluate(currentState, k1, dt * 0.5);
	Derivative k3 = evaluate(currentState, k2, dt * 0.5);
	Derivative k4 = evaluate(currentState, k3, dt);

	Derivative netDerivative = (k1 + (k2 * 2.0) + (k3 * 2.0) + k4) * (1.0 / 6.0);

	currentState = stepState(currentState, netDerivative, dt);
}

Matrix3x3 Rocket::getInertiaTensor() const {

	Matrix3x3 tensor;

	tensor.m[0][0] = emptyIxx;
	tensor.m[1][1] = emptyIyy;
	tensor.m[2][2] = emptyIzz;

	return tensor;
}

Matrix3x3 Rocket::getInverseInertiaTensor() const {

	Matrix3x3 inverseTensor;

	inverseTensor.m[0][0] = 1.0 / emptyIxx;
	inverseTensor.m[1][1] = 1.0 / emptyIyy;
	inverseTensor.m[2][2] = 1.0 / emptyIzz;

	return inverseTensor;
}