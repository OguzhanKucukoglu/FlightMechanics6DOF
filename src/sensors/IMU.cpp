#include "sensors/IMU.h"
#include "environment/GravityModel.h"

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

	double altitude = -trueState.position.z;
	Vector3D gWorld = GravityModel::getGravity(altitude);
	Vector3D specificForceNED = trueAcceleration - gWorld;

	Vector3D specificForceBody = trueState.orientation.conjugate().rotate(specificForceNED);
	Vector3D angularVelBody = trueState.angularVelocity;

	measurement.measuredAcceleration.x = specificForceBody.x + accelerationBias.x + accelerationDist(generator);
	measurement.measuredAcceleration.y = specificForceBody.y + accelerationBias.y + accelerationDist(generator);
	measurement.measuredAcceleration.z = specificForceBody.z + accelerationBias.z + accelerationDist(generator);

	measurement.measuredAngularVelocity.x = angularVelBody.x + gyroBias.x + gyroDist(generator);
	measurement.measuredAngularVelocity.y = angularVelBody.y + gyroBias.y + gyroDist(generator);
	measurement.measuredAngularVelocity.z = angularVelBody.z + gyroBias.z + gyroDist(generator);

	return measurement;
}