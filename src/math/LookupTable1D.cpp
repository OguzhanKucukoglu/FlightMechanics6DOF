#include "math/LookupTable1D.h"
#include <algorithm>

void LookupTable1D::addPoint(double x, double y) {
	table.push_back({ x,y });

	std::sort(table.begin(), table.end(), [](const std::pair<double, double>& a, const std::pair<double, double>& b) {
		return a.first < b.first;
	});
}

double LookupTable1D::getValue(double x) const {
	if (table.empty()) return 0.0;
	if (table.size() == 1) return table[0].second;

	if (x <= table.front().first) return table.front().second;
	if (x >= table.back().first) return table.back().second;

    for (size_t i = 0; i < table.size() - 1; ++i) {
        double x0 = table[i].first;
        double x1 = table[i + 1].first;

        if (x >= x0 && x <= x1) {
            double y0 = table[i].second;
            double y1 = table[i + 1].second;

            double t = (x - x0) / (x1 - x0);
            return y0 + t * (y1 - y0);
        }
    }
    return 0.0;
}