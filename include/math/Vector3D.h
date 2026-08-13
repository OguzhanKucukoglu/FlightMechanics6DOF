#pragma once

class Vector3D {
public:
	double x, y, z;

	Vector3D(double x_val = 0.0, double y_val = 0.0, double z_val = 0.0) : x(x_val), y(y_val), z(z_val) {}

	Vector3D operator+(const Vector3D& v) const;
	Vector3D operator-(const Vector3D& v) const;
	Vector3D operator*(double scalar) const;

	double dotProduct(const Vector3D& v) const;
	Vector3D crossProduct(const Vector3D& v) const;

	double magnitude() const;
	Vector3D normalized() const;

};