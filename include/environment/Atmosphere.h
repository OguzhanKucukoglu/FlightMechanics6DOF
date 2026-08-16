#pragma once

struct AtmosphericData {
	double density; // kg/m^3
	double pressure; // Pa
	double temperature; // Kelvin
	double speedOfSound; // m/s
};

class Atmosphere {
public:

	static AtmosphericData getConditions(double altitude);
};