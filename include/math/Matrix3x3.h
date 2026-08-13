#pragma once
#include "math/Vector3D.h"

struct Matrix3x3 {

	double m[3][3] = { 0.0 };

	Matrix3x3() = default;
		
	Vector3D operator*(const Vector3D& v) const {
		return Vector3D(
			(m[0][0] * v.x) + (m[0][1] * v.y) + (m[0][2] * v.z),
			(m[1][0] * v.x) + (m[1][1] * v.y) + (m[1][2] * v.z),
			(m[2][0] * v.x) + (m[2][1] * v.y) + (m[2][2] * v.z)
		);
	}
};
