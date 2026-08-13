#pragma once
#include "math/Vector3D.h"

class GravityModel {
public:
	static Vector3D getGravity(double altitude);
};