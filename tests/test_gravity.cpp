#include "../include/environment/GravityModel.h"
#include "TestFramework.h"

void runGravityTests() {
    std::cout << "--- GRAVITY MODEL TESTS ---\n";

    // Test 1: Sea-level gravity (0 m) should be -9.81 m/s^2
    Vector3D g_surface = GravityModel::getGravity(0.0);
    EXPECT_NEAR(g_surface.y, -9.81, 1e-4, "Sea Level Gravity (-9.81)");

    // Test 2: Gravity at 1 Earth radius altitude (6371 km) should be 1/4th of surface gravity (Inverse-Square Law)
    Vector3D g_space = GravityModel::getGravity(6371000.0);
    EXPECT_NEAR(g_space.y, -2.4525, 1e-4, "Gravity at 1 R_Earth Altitude (-2.4525)");
}