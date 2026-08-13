#pragma once
#include "math/Vector3D.h"
#include "dynamics/State.h"
#include <random>

struct IMU_Measurement {
	Vector3D measuredAcceleration;
	Vector3D measuredAngularVelocity;
};

class IMU {
private:
	double accelerationNoiseStddev;
	double gyroNoiseStddev;

	Vector3D accelerationBias;
	Vector3D gyroBias;

	std::mt19937 generator;
	std::normal_distribution<double> accelerationDist;
	std::normal_distribution<double> gyroDist;

public:
	IMU(double accel_noise, double gyro_noise, Vector3D accel_bias, Vector3D gyro_bias);


	IMU_Measurement readSensor(const State& trueState, const Vector3D& trueAcceleration);
};