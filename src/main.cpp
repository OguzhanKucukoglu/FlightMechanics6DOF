#include <iostream>
#include "sensors/IMU.h"
#include "vehicle/Rocket.h"

using namespace std;

int main() {

	State initialState;
	initialState.position = Vector3D(0.0, 0.0, 0.0);
	initialState.velocity = Vector3D(0.0, 0.0, 0.0);
	initialState.angularVelocity = Vector3D(0.0, 0.0, 0.0);
	initialState.orientation = Quaternion(1.0, 0.0, 0.0, 0.0);

	double dryMass = 150.0;
	double fuelMass = 50.0;
	double refArea = 0.05;
	double refLength = 3.0;
	double thrustMag = 5000.0;
	double massFlowRate = 2.0;
	double dragCoeff = 0.4;

	double emptyIxx = 10.0, emptyIyy = 50.0, emptyIzz = 50.0;
	double fullIxx = 15.0, fullIyy = 70.0, fullIzz = 70.0;

	double emptyCG_X = -1.0;
	double fullCG_X = -1.2;
	double engine_X = -3.0;

	Rocket myRocket(dryMass, fuelMass, refArea, refLength,
		thrustMag, massFlowRate, dragCoeff,
		emptyIxx, emptyIyy, emptyIzz,
		fullIxx, fullIyy, fullIzz,
		emptyCG_X, fullCG_X, engine_X, initialState
	);

	double accelNoise = 0.05;
	double gyroNoise = 0.002;

	Vector3D accelBias(0.01, -0.02, 0.005);
	Vector3D gyroBias(0.001, 0.001, -0.001);

	IMU myIMU(accelNoise, gyroNoise, accelBias, gyroBias);

	double dt = 0.01;
	double maxTime = 0.5;

	for (double t = 0.0; t <= maxTime; t += dt) {

		myRocket.integrate(dt);

		State trueState = myRocket.getState();
		Vector3D trueAccel = myRocket.getCurrentAcceleration();

		IMU_Measurement imuData = myIMU.readSensor(trueState, trueAccel);

		cout << "Zaman: " << t << "s\n";
		cout << "[GERCEK] Ivme X : " << trueAccel.x << "\n";
		cout << "[SENSOR] Ivme X : " << imuData.measuredAcceleration.x << "\n";
		cout << "----------------------------------------\n";

	}

	return 0;
}