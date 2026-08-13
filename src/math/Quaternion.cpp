#include "math/Quaternion.h"
#include <cmath>

void Quaternion::normalize() {

	double mag = sqrt((w * w) + (x * x) + (y * y) + (z * z));

	if (mag > 1e-9) {
		w /= mag;
		x /= mag;
		y /= mag;
		z /= mag;
	}
	else {
		w = 1.0;
		x = 0.0;
		y = 0.0;
		z = 0.0;
	}
}

Quaternion Quaternion::operator*(const Quaternion& q)const {

	double w_new, x_new, y_new, z_new;

	w_new = (w * q.w) - (x * q.x) - (y * q.y) - (z * q.z);
	x_new = (w * q.x) + (x * q.w) + (y * q.z) - (z * q.y);
	y_new = (w * q.y) - (x * q.z) + (y * q.w) + (z * q.x);
	z_new = (w * q.z) + (x * q.y) - (y * q.x) + (z * q.w);

	return Quaternion(w_new, x_new, y_new, z_new);
}

Quaternion Quaternion::operator+(const Quaternion& q) const {

	return Quaternion(w + q.w, x + q.x, y + q.y, z + q.z);
}

Quaternion Quaternion::operator*(double scalar) const {

	return Quaternion(w * scalar, x * scalar, y * scalar, z * scalar);
}

Quaternion Quaternion::getDerivative(const Vector3D& angularVelocity) {

	return Quaternion(
		0.5 * (-x * angularVelocity.x - y * angularVelocity.y - z * angularVelocity.z),
		0.5 * (w * angularVelocity.x - y * angularVelocity.z - z * angularVelocity.y),
		0.5 * (w * angularVelocity.y - x * angularVelocity.z + z * angularVelocity.x),
		0.5 * (w * angularVelocity.z + x * angularVelocity.y - y * angularVelocity.x)
	);
}