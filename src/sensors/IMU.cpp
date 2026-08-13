#include "sensors/IMU.h"

IMU::IMU(double accel_noise, double gyro_noise, Vector3D accel_bias, Vector3D gyro_bias)
	: accelerationNoiseStddev(accel_noise),
	gyroNoiseStddev(gyro_noise),
	accelerationBias(accel_bias),
	gyroBias(gyro_bias),
	generator(std::random_device{}()),
	accelerationDist(0.0, accel_noise),
	gyroDist(0.0, gyro_noise)
{
}

IMU_Measurement IMU::readSensor(const State& trueState, const Vector3D& trueAcceleration) {

	IMU_Measurement measurement;

	measurement.measuredAcceleration.x = trueAcceleration.x + accelerationBias.x + accelerationDist(generator);
	measurement.measuredAcceleration.y = trueAcceleration.y + accelerationBias.y + accelerationDist(generator);
	measurement.measuredAcceleration.z = trueAcceleration.z + accelerationBias.z + accelerationDist(generator);

	measurement.measuredAngularVelocity.x = trueState.angularVelocity.x + gyroBias.x + gyroDist(generator);
	measurement.measuredAngularVelocity.y = trueState.angularVelocity.y + gyroBias.y + gyroDist(generator);
	measurement.measuredAngularVelocity.z = trueState.angularVelocity.z + gyroBias.z + gyroDist(generator);

	return measurement;
}