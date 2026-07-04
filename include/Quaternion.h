#pragma once
#include "Vector3D.h"

class Quaternion{
public:

	double w, x, y, z;

	Quaternion(double w_val = 1.0, double x_val = 0.0, double y_val = 0.0, double z_val = 0.0) : w(w_val), x(x_val), y(y_val), z(z_val) {}

	void normalize();

	Quaternion operator*(const Quaternion& q) const;

	Quaternion operator+(const Quaternion& q) const;
	Quaternion operator*(double scalar) const;

	Quaternion conjugate() const {
		return Quaternion(w, -x, -y, -z);
	}

	Vector3D rotate(const Vector3D& v) const {

		Quaternion q_vec(0.0, v.x, v.y, v.z); // Vektörü saf kuaterniyon yap

		Quaternion q_res = (*this) * q_vec * this->conjugate();

		return Vector3D(q_res.x, q_res.y, q_res.z);
	}
};
