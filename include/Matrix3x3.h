#pragma once
#include "Vector3D.h"

struct Matrix3x3 {

	double m[3][3];

	Matrix3x3() {
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				m[i][j] = 0.0;
			}
		}
	}

	Vector3D operator*(const Vector3D v) const {
		return Vector3D(
			(m[0][0] * v.x) + (m[0][1] * v.y) + (m[0][2] * v.z),
			(m[1][0] * v.x) + (m[1][1] * v.y) + (m[1][2] * v.z),
			(m[2][0] * v.x) + (m[2][1] * v.y) + (m[2][2] * v.z)
		);
	}
};
