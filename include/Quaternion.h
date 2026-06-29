#pragma once

class Quaternion{
public:

	double w, x, y, z;

	Quaternion(double w_val = 1.0, double x_val = 0.0, double y_val = 0.0, double z_val = 0.0) : w(w_val), x(x_val), y(y_val), z(z_val) {}

	void normalize();

	Quaternion operator*(const Quaternion& q) const;

	Quaternion operator+(const Quaternion& q) const;
	Quaternion operator*(double scalar) const;
};
