#pragma once
#include <vector>
#include <utility>
#include <stdexcept>

class LookupTable1D {
private:

	std::vector<std::pair<double, double>> table;

public:

	void addPoint(double x, double y);

	double getValue(double x) const;
};