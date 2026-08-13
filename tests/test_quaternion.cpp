#include "../include/math/Quaternion.h"
#include "../include/math/Vector3D.h"
#include "TestFramework.h"
#include <cmath>

void runQuaternionTests() {
	std::cout << "\n--- QUATERNION KINEMATICS TESTS ---\n";

    // Test 1: Identity Quaternion. An unrotated object in space must have w = 1.0 and vector components = 0.
    Quaternion q_identity(1.0, 0.0, 0.0, 0.0);
    EXPECT_NEAR(q_identity.magnitude(), 1.0, 1e-5, "Identity Quaternion Magnitude = 1");

    // Test 2: Normalization. Quaternions can drift over time due to RK4 integration. 
    // Normalizing must strictly enforce a magnitude of 1.0.
    Quaternion q_unnorm(2.0, 0.0, 2.0, 0.0);
    q_unnorm.normalize();
    EXPECT_NEAR(q_unnorm.magnitude(), 1.0, 1e-5, "Normalization forces magnitude to 1");

    // Test 3: Exact Rotation Test (Right-Handed Rule).
    // If the nose points to +X, rotating 90 degrees around +Z must align the nose perfectly with +Y. 
    // For a 90-deg rotation: w = cos(45) = 0.7071, z = sin(45) = 0.7071
    double half_angle = (90.0 * 3.14159265359 / 180.0) / 2.0;
    Quaternion q_rotZ(std::cos(half_angle), 0.0, 0.0, std::sin(half_angle));

    Vector3D noseVector(1.0, 0.0, 0.0);
    Vector3D rotatedNose = q_rotZ.rotate(noseVector);

    EXPECT_NEAR(rotatedNose.x, 0.0, 1e-5, "90-deg Z-Rotation: X becomes 0");
    EXPECT_NEAR(rotatedNose.y, 1.0, 1e-5, "90-deg Z-Rotation: Y becomes 1");
    EXPECT_NEAR(rotatedNose.z, 0.0, 1e-5, "90-deg Z-Rotation: Z remains 0");
}