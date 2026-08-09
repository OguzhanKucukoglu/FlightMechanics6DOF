#include "Rocket.h"
#include <algorithm>
#include <cmath>

Rocket::Rocket(double dry_m, double fuel_m, double s_ref, double l_ref, double magnitude, double flow_rate, double drag_coeff, double empty_ixx, double empty_iyy, double empty_izz, double full_ixx, double full_iyy, double full_izz, double empty_cg_x, double full_cg_x, double engineX, const State& initial_state)
	: dryMass(dry_m),
	referenceArea(s_ref),
	referenceLength(l_ref),
	thrustMagnitude(magnitude),
	massFlowRate(flow_rate),
	dragCoefficient(drag_coeff),
	emptyIxx(empty_ixx),
	emptyIyy(empty_iyy),
	emptyIzz(empty_izz),
	fullIxx(full_ixx),
	fullIyy(full_iyy),
	fullIzz(full_izz),
	emptyCG_X(empty_cg_x),
	fullCG_X(full_cg_x),
	engine_X(engineX),
	gimbal_Y(0.0),
	gimbal_Z(0.0),
	currentFuelMass(fuel_m),
	initialFuelMass(fuel_m),
	currentState(initial_state)
{ }

double Rocket::getTotalMass() const {

	return dryMass + currentFuelMass;
}

double getAtmosphericDensity(double altitude) {

	if (altitude < 0.0) {
		return 1.225;
	}

	if (altitude < 11000.0) {
		double temperature = 15.04 - (0.00649 * altitude);

		double pressure = 101.29 * std::pow((temperature + 273.1) / 288.08, 5.256);

		return pressure / (0.2869 * (temperature + 273.1));
	}
	else if (altitude < 25000.0) {
		double temperature = -56.46;

		double pressure = 22.65 * std::exp(1.73 - (0.000157 * altitude));

		return pressure / (0.2869 * (temperature + 273.1));
	}
	else {
		return 0.0;
	}

}

double Rocket::getCurrentCG() const {

	double fuelRatio = 0.0;
	if (initialFuelMass > 1e-9) {
		fuelRatio = currentFuelMass / initialFuelMass;
	}

	double currentCG_X = emptyCG_X + (fullCG_X - emptyCG_X) * fuelRatio;

	return currentCG_X;
}

Derivative Rocket::evaluate(const State& initial, const Derivative& d, double dt) const {

	State state = stepState(initial, d, dt);

	Derivative output;

	double mass = getTotalMass();

	output.velocity = state.velocity;

	output.spin.w = 0.5 * (-state.orientation.x * state.angularVelocity.x - state.orientation.y * state.angularVelocity.y - state.orientation.z * state.angularVelocity.z);
	output.spin.x = 0.5 * (state.orientation.w * state.angularVelocity.x + state.orientation.y * state.angularVelocity.z - state.orientation.z * state.angularVelocity.y);
	output.spin.y = 0.5 * (state.orientation.w * state.angularVelocity.y - state.orientation.x * state.angularVelocity.z + state.orientation.z * state.angularVelocity.x);
	output.spin.z = 0.5 * (state.orientation.w * state.angularVelocity.z + state.orientation.x * state.angularVelocity.y - state.orientation.y * state.angularVelocity.x);

	Vector3D gravity(0.0, -9.81, 0.0);
	Vector3D netForce = gravity * mass;
	Vector3D netTorque(0.0, 0.0, 0.0);

	if (currentFuelMass > 0.0) {
		
		double tvc_x = thrustMagnitude * std::cos(gimbal_Y) * std::cos(gimbal_Z);
		double tvc_y = thrustMagnitude * std::sin(gimbal_Y);
		double tvc_z = thrustMagnitude * std::sin(gimbal_Z);

		Vector3D localThrust(tvc_x, tvc_y, tvc_z);

		Vector3D worldThrust = state.orientation.rotate(localThrust);
		netForce = netForce + worldThrust;

		double currentCG_X = getCurrentCG();
		Vector3D engineMomentArm(engine_X - currentCG_X, 0.0, 0.0);

		Vector3D tvcTorque = engineMomentArm.crossProduct(localThrust);

		netTorque = netTorque + tvcTorque;
	}

	double v_mag = state.velocity.magnitude();
	if (v_mag > 1e-9) {

		double rho = getAtmosphericDensity(state.position.y);
		double dragMag = 0.5 * rho * v_mag * v_mag * dragCoefficient * referenceArea;

		Vector3D dragVector = state.velocity.normalized() * (-dragMag);

		netForce = netForce + dragVector;

		Vector3D localDrag = state.orientation.conjugate().rotate(dragVector);

		double currentCG_X = getCurrentCG();

		double currentCP_X = -2.5;

		Vector3D cpVector(currentCP_X - currentCG_X, 0.0, 0.0);

		Vector3D aeroTorque = cpVector.crossProduct(localDrag);

		netTorque = netTorque + aeroTorque;
	}

	output.acceleration = netForce * (1.0 / mass);

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

	currentAcceleration = netDerivative.acceleration;

	currentState = stepState(currentState, netDerivative, dt);

	currentFuelMass = std::max(0.0, currentFuelMass - massFlowRate * dt);
}

Matrix3x3 Rocket::getInertiaTensor() const {

	Matrix3x3 tensor;

	double fuelRatio = 0.0;
	if (initialFuelMass > 1e-9) {
		fuelRatio = currentFuelMass / initialFuelMass;
	}

	double currentIxx = emptyIxx + (fullIxx - emptyIxx) * fuelRatio;
	double currentIyy = emptyIyy + (fullIyy - emptyIyy) * fuelRatio;
	double currentIzz = emptyIzz + (fullIzz - emptyIzz) * fuelRatio;

	tensor.m[0][0] = currentIxx;
	tensor.m[1][1] = currentIyy;
	tensor.m[2][2] = currentIzz;

	return tensor;
}

Matrix3x3 Rocket::getInverseInertiaTensor() const {

	Matrix3x3 inverseTensor;

	double fuelRatio = 0.0;
	if (initialFuelMass > 1e-9) {
		fuelRatio = currentFuelMass / initialFuelMass;
	}

	double currentIxx = emptyIxx + (fullIxx - emptyIxx) * fuelRatio;
	double currentIyy = emptyIyy + (fullIyy - emptyIyy) * fuelRatio;
	double currentIzz = emptyIzz + (fullIzz - emptyIzz) * fuelRatio;

	inverseTensor.m[0][0] = 1.0 / currentIxx;
	inverseTensor.m[1][1] = 1.0 / currentIyy;
	inverseTensor.m[2][2] = 1.0 / currentIzz;

	return inverseTensor;
}