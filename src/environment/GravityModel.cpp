#include "environment/GravityModel.h"
#include <algorithm>

Vector3D GravityModel::getGravity(double altitude) {

	double h = std::max(0.0, altitude);

	const double g0 = 9.81;
	const double earthR = 6371000.0;

	double currentG = g0 * (earthR / (earthR + h)) * (earthR / (earthR + h));

	return Vector3D(0.0, -currentG, 0.0);
}