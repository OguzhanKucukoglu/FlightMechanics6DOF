#include "environment/Atmosphere.h"
#include <cmath>

AtmosphericData Atmosphere::getConditions(double altitude) {
	AtmosphericData data = { 1.225, 101325.0, 288.15 };

	if (altitude < 0.0) {
		return data;
	}

	if (altitude < 11000.0) {
		double tempCelcius = 15.04 - (0.00649 * altitude);

		data.temperature = tempCelcius + 273.1;
		data.pressure = 101.29 * 1000.0 * std::pow((tempCelcius + 273.1) / 288.08, 5.256);
		data.density = data.pressure / (287.05 * data.temperature);
	}
	else if (altitude < 25000.0) {
		double tempCelcius = -56.46;

		data.temperature = tempCelcius + 273.1;
		data.pressure = 22.65 * 1000.0 * std::exp(1.73 - (0.000157 * altitude));
		data.density = data.pressure / (287.05 * data.temperature);
	}
	else {
		data.density = 0.0;
		data.pressure = 0.0;
	}

	return data;
}