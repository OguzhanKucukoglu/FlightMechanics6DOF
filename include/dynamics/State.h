#pragma once
#include "math/Vector3D.h"
#include "math/Quaternion.h"

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