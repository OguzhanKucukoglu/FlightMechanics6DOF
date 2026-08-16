#include "vehicle/Rocket.h"
#include "environment/Atmosphere.h"
#include "environment/GravityModel.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

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
	emptyIxy(0.0),
	emptyIxz(0.0),
	emptyIyz(0.0),
	fullIxx(full_ixx),
	fullIyy(full_iyy),
	fullIzz(full_izz),
	fullIxy(0.0),
	fullIxz(0.0),
	fullIyz(0.0),
	emptyCG_X(empty_cg_x),
	fullCG_X(full_cg_x),
	engine_X(engineX),
	gimbal_Y(0.0),
	gimbal_Z(0.0),
	initialFuelMass(fuel_m),
	currentState(initial_state)
{ }

double Rocket::getCurrentCG(double currentFuel) const {

	double fuelRatio = 0.0;
	if (initialFuelMass > 1e-9) {
		fuelRatio = std::clamp(currentFuel / initialFuelMass, 0.0, 1.0);
	}

	double currentCG_X = emptyCG_X + (fullCG_X - emptyCG_X) * fuelRatio;

	return currentCG_X;
}

Derivative Rocket::evaluate(const State& initial, const Derivative& d, double dt) const {

	State state = stepState(initial, d, dt);

	Derivative output;

	double currentFuel = state.fuelMass;
	double mass = dryMass + currentFuel;

	output.velocity = state.velocity;

	output.orientationDerivative = state.orientation.getDerivative(state.angularVelocity);
	
	double altitude = -state.position.z;
	Vector3D gravity = GravityModel::getGravity(altitude);
	Vector3D netForce = gravity * mass;
	Vector3D netTorque(0.0, 0.0, 0.0);

	if (currentFuel > 0.0) {

		output.fuelMassDot = -massFlowRate;
		
		Vector3D thrustBody(
			thrustMagnitude * std::cos(gimbal_Y) * std::cos(gimbal_Z),
			thrustMagnitude* std::cos(gimbal_Y)* std::sin(gimbal_Z),
			-thrustMagnitude * std::sin(gimbal_Y)
		);

		Vector3D worldThrust = state.orientation.rotate(thrustBody);
		netForce = netForce + worldThrust;

		double currentCG_X = getCurrentCG(currentFuel);
		Vector3D engineMomentArm(engine_X - currentCG_X, 0.0, 0.0);

		Vector3D tvcTorque = engineMomentArm.crossProduct(thrustBody);

		netTorque = netTorque + tvcTorque;
	}
	else {
		output.fuelMassDot = 0.0;
	}

	double v_mag = state.velocity.magnitude();
	if (v_mag > 1e-9) {
 
		AtmosphericData env = Atmosphere::getConditions(-state.position.z);
		double rho = env.density;

		// TODO (AeroDB Entegrasyonu): CD sabit olamaz. 
		// Ýleride Mach sayýsý (M) ve Hücum Açýsýna (Alpha) baðlý olarak AeroDB'den çekilecek:
		// double Mach = v_mag / env.speedOfSound;
		// double alpha = calculateAngleOfAttack(state);
		// double currentCD = AeroDB::getCD(Mach, alpha);

		double dragMag = 0.5 * rho * v_mag * v_mag * dragCoefficient * referenceArea;

		Vector3D dragVector = state.velocity.normalized() * (-dragMag);

		netForce = netForce + dragVector;

		Vector3D localDrag = state.orientation.conjugate().rotate(dragVector);

		double currentCG_X = getCurrentCG(currentFuel);

		// TODO (AeroDB Entegrasyonu): Basýnç Merkezi (CP) sabit (-2.5) olamaz.
		// Mach ve Alpha deðiþtikçe CP kayar. Ýleride AeroDB::getCP(Mach, alpha) ile baðlanacak.
		double currentCP_X = -2.5;

		Vector3D cpVector(currentCP_X - currentCG_X, 0.0, 0.0);

		Vector3D aeroTorque = cpVector.crossProduct(localDrag);

		netTorque = netTorque + aeroTorque;
	}

	output.acceleration = netForce * (1.0 / mass);

	Matrix3x3 I = getInertiaTensor(currentFuel);
	Matrix3x3 I_inv = getInverseInertiaTensor(currentFuel);

	Vector3D angularMomentum = I * state.angularVelocity;
	Vector3D gyroscopicTorque = state.angularVelocity.crossProduct(angularMomentum);
	Vector3D effectiveTorque = netTorque - gyroscopicTorque;

	output.angularAcceleration = I_inv * effectiveTorque;

	return output;

}

void Rocket::integrate(double dt) {

	Derivative zeroDerivative = { Vector3D(0,0,0), Vector3D(0,0,0), Vector3D(0,0,0), Quaternion(0,0,0,0), 0.0 };

	Derivative k1 = evaluate(currentState, zeroDerivative, 0.0);
	Derivative k2 = evaluate(currentState, k1, dt * 0.5);
	Derivative k3 = evaluate(currentState, k2, dt * 0.5);
	Derivative k4 = evaluate(currentState, k3, dt);

	Derivative netDerivative = (k1 + (k2 * 2.0) + (k3 * 2.0) + k4) * (1.0 / 6.0);

	currentAcceleration = netDerivative.acceleration;

	currentState = stepState(currentState, netDerivative, dt);
}

Matrix3x3 Rocket::getInertiaTensor(double currentFuel) const {

	Matrix3x3 tensor;

	double fuelRatio = 0.0;
	if (initialFuelMass > 1e-9) {
		fuelRatio = std::clamp(currentFuel / initialFuelMass, 0.0, 1.0);
	}

	double currentIxx = emptyIxx + (fullIxx - emptyIxx) * fuelRatio;
	double currentIyy = emptyIyy + (fullIyy - emptyIyy) * fuelRatio;
	double currentIzz = emptyIzz + (fullIzz - emptyIzz) * fuelRatio;

	double currentIxy = emptyIxy + (fullIxy - emptyIxy) * fuelRatio;
	double currentIxz = emptyIxz + (fullIxz - emptyIxz) * fuelRatio;
	double currentIyz = emptyIyz + (fullIyz - emptyIyz) * fuelRatio;

	tensor.m[0][0] = currentIxx;   tensor.m[0][1] = -currentIxy;  tensor.m[0][2] = -currentIxz;
	tensor.m[1][0] = -currentIxy;  tensor.m[1][1] = currentIyy;   tensor.m[1][2] = -currentIyz;
	tensor.m[2][0] = -currentIxz;  tensor.m[2][1] = -currentIyz;  tensor.m[2][2] = currentIzz;

	return tensor;
}

Matrix3x3 Rocket::getInverseInertiaTensor(double currentFuel) const {

	Matrix3x3 I = getInertiaTensor(currentFuel);
	Matrix3x3 inverseTensor;

	// 3x3 Matrix Elements
	double m00 = I.m[0][0], m01 = I.m[0][1], m02 = I.m[0][2];
	double m10 = I.m[1][0], m11 = I.m[1][1], m12 = I.m[1][2];
	double m20 = I.m[2][0], m21 = I.m[2][1], m22 = I.m[2][2];

	// Calculate the cofactors
	double c00 = m11 * m22 - m12 * m21;
	double c01 = m12 * m20 - m10 * m22;
	double c02 = m10 * m21 - m11 * m20;

	// Determinant
	double det = m00 * c00 + m01 * c01 + m02 * c02;


	if (std::abs(det) < 1e-9) {
		throw std::runtime_error("FATAL ERROR: Singular inertia tensor! Check mass properties (Ixx, Iyy, Izz).");
	}

	double invDet = 1.0 / det;

	// Multiply the adjugate matrix by the inverse of the determinant to create the inverse matrix
	inverseTensor.m[0][0] = c00 * invDet;
	inverseTensor.m[0][1] = (m02 * m21 - m01 * m22) * invDet;
	inverseTensor.m[0][2] = (m01 * m12 - m02 * m11) * invDet;

	inverseTensor.m[1][0] = c01 * invDet;
	inverseTensor.m[1][1] = (m00 * m22 - m02 * m20) * invDet;
	inverseTensor.m[1][2] = (m02 * m10 - m00 * m12) * invDet;

	inverseTensor.m[2][0] = c02 * invDet;
	inverseTensor.m[2][1] = (m01 * m20 - m00 * m21) * invDet;
	inverseTensor.m[2][2] = (m00 * m11 - m01 * m10) * invDet;

	return inverseTensor;
}