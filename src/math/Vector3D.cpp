#include "math/Vector3D.h"
#include <cmath>

Vector3D Vector3D::operator+(const Vector3D& v) const {

	return Vector3D(x + v.x, y + v.y, z + v.z);
}

Vector3D Vector3D::operator-(const Vector3D& v) const {

	return Vector3D(x - v.x, y - v.y, z - v.z);
}

Vector3D Vector3D::operator*(double scalar) const {

	return Vector3D(x * scalar, y * scalar, z * scalar);
}

double Vector3D::dotProduct(const Vector3D& v) const {

	return (x * v.x) + (y * v.y) + (z * v.z);
}

Vector3D Vector3D::crossProduct(const Vector3D& v) const {

	return Vector3D(
		(y * v.z) - (z * v.y),
		(z * v.x) - (x * v.z),
		(x * v.y) - (y * v.x)
	);
}

double Vector3D::magnitude() const {

	return sqrt((x * x) + (y * y) + (z * z));
}

Vector3D Vector3D::normalized() const {
	double mag = magnitude();

	if (mag > 1e-9) {
		return Vector3D(x / mag, y / mag, z / mag);
	}

	return Vector3D(0.0, 0.0, 0.0);
}