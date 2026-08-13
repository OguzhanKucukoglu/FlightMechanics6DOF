#pragma once
#include <iostream>
#include <cmath>
#include <string>

#define EXPECT_NEAR(val1, val2, tol, testName) \
    if (std::abs((val1) - (val2)) <= (tol)) { \
        std::cout << "[PASS] " << testName << "\n"; \
    } else { \
        std::cout << "[FAIL] " << testName << " | Expected: " << (val2) << ", Actual: " << (val1) << "\n"; \
    }