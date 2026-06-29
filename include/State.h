#pragma once
#include "Vector3D.h"
#include "Quaternion.h"

struct State {
	Vector3D position; // Konum (m)
	Vector3D velocity; // Doðrusal Hýz (m/s)
	Vector3D angularVelocity; // Açýsal Hýz (rad/s)
	Quaternion orientation; // Yönelim (Kuaterniyon)
};

struct Derivative {
	Vector3D velocity; // Konumun türevi hýzdýr
	Vector3D acceleration; // Hýzýn türevi ivmedir
	Vector3D angularAcceleration; // Açýsal hýzýn türevi
	Quaternion spin; // Yönelimin türevi (Kuaterniyon Kinematiði)

	Derivative operator+(const Derivative& d) const {

		return Derivative{
			velocity + d.velocity,
			acceleration + d.acceleration,
			angularAcceleration + d.angularAcceleration,
			spin + d.spin
		};
	}

	Derivative operator*(double scalar) const {

		return Derivative{
			velocity * scalar,
			acceleration * scalar,
			angularAcceleration * scalar,
			spin * scalar
		};
	}
};

inline State stepState(const State& current, const Derivative& deriv, double dt) {
	State nextState;

	nextState.position = current.position + (deriv.velocity * dt);

	nextState.velocity = current.velocity + (deriv.acceleration * dt);

	nextState.angularVelocity = current.angularVelocity + (deriv.angularAcceleration * dt);

	nextState.orientation = current.orientation + (deriv.spin * dt);

	nextState.orientation.normalize();

	return nextState;
}

inline Derivative evaluate(const State& initial, const Derivative& d, double dt) {

	State state = stepState(initial, d, dt);

	Derivative output;

	output.velocity = state.velocity;

	output.spin.w = 0.5 * (-state.orientation.x * state.angularVelocity.x - state.orientation.y * state.angularVelocity.y - state.orientation.z * state.angularVelocity.z);
	output.spin.x = 0.5 * (state.orientation.w * state.angularVelocity.x + state.orientation.y * state.angularVelocity.z - state.orientation.z * state.angularVelocity.y);
	output.spin.y = 0.5 * (state.orientation.w * state.angularVelocity.y - state.orientation.x * state.angularVelocity.z + state.orientation.z * state.angularVelocity.x);
	output.spin.z = 0.5 * (state.orientation.w * state.angularVelocity.z + state.orientation.x * state.angularVelocity.y - state.orientation.y * state.angularVelocity.x);

	Vector3D gravity(0.0, -9.81, 0.0);
	Vector3D netForce = gravity;
	double mass = 100.0;

	output.acceleration = netForce * (1.0 / mass);

	Vector3D netTorque(0.0, 0.0, 0.0);
	double inertia = 50.0;

	output.angularAcceleration = netTorque * (1.0 / inertia);

	return output;
}

inline void integrate(State& state, double dt) {

	Derivative zeroDerivative = { Vector3D(0,0,0), Vector3D(0,0,0), Vector3D(0,0,0), Quaternion(0,0,0,0) };

	Derivative k1 = evaluate(state, zeroDerivative, 0.0);
	Derivative k2 = evaluate(state, k1, dt * 0.5);
	Derivative k3 = evaluate(state, k2, dt * 0.5);
	Derivative k4 = evaluate(state, k3, dt);

	Derivative netDerivative = (k1 + (k2 * 2.0) + (k3 * 2.0) + k4) * (1.0 / 6.0);

	state = stepState(state, netDerivative, dt);
}