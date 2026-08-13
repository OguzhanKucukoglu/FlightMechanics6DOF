#include "../include/vehicle/Rocket.h"
#include "../include/environment/GravityModel.h"
#include "TestFramework.h"

void runDynamicsTests() {
	std::cout << "\n--- ROCKET DYNAMICS (RK4) TESTS ---\n";

    // TEST 1: FREE-FALL
    // Engine off, drag coefficient zero. Pure Newton kinematics under gravity.
    State state;
    state.position = Vector3D(0.0, 10000.0, 0.0);
    state.velocity = Vector3D(0.0, 0.0, 0.0);
    state.angularVelocity = Vector3D(0.0, 0.0, 0.0);
    state.orientation = Quaternion(1.0, 0.0, 0.0, 0.0);
    state.fuelMass = 0.0;

    // Disable atmosphere and propulsion by setting CD = 0.0 and Thrust = 0.0
    Rocket rocket(100.0, 0.0, 0.05, 3.0, 0.0, 0.0, 0.0,
        10.0, 50.0, 50.0, 10.0, 50.0, 50.0,
        -1.0, -1.0, -3.0, state);

    double dt = 0.01;
    double total_time = 1.0;
    int steps = total_time / dt;

    for (int i = 0; i < steps; ++i) {
        rocket.integrate(dt);
    }

    State finalState = rocket.getState();

    // Analytically calculate local gravity at 10 km using the Inverse-Square law
    double g_local = 9.81 * (6371000.0 / 6381000.0) * (6371000.0 / 6381000.0);
    double expected_vy = -g_local * total_time;
    double expected_y = 10000.0 - 0.5 * g_local * total_time * total_time;

    EXPECT_NEAR(finalState.velocity.y, expected_vy, 1e-2, "Free-fall Velocity (RK4 matches analytical)");
    EXPECT_NEAR(finalState.position.y, expected_y, 1e-2, "Free-fall Position (RK4 matches analytical)");

    // TEST 2: GYROSCOPIC COUPLING
    // No external torque applied. Proof that a spinning asymmetric body 
    // auto-generates torque in the 3rd axis due to Euler's equation: w x (Iw)
    State spinState;
    spinState.position = Vector3D(0.0, 0.0, 0.0);
    spinState.velocity = Vector3D(0.0, 0.0, 0.0);
    spinState.angularVelocity = Vector3D(1.0, 1.0, 0.0);
    spinState.orientation = Quaternion(1.0, 0.0, 0.0, 0.0);
    spinState.fuelMass = 0.0;

    Rocket spinRocket(100.0, 0.0, 0.05, 3.0, 0.0, 0.0, 0.0,
        10.0, 50.0, 50.0, 10.0, 50.0, 50.0,
        -1.0, -1.0, -3.0, spinState);

    // Integrate for exactly 1 millisecond to capture the instantaneous Euler angular acceleration
    spinRocket.integrate(0.001);
    State nextSpinState = spinRocket.getState();

    // Analytical Expectation: w_dot = -I^-1 (w x Iw)
    // w = [1, 1, 0], I = diag(10, 50, 50) -> Iw = [10, 50, 0]
    // w x Iw = [0, 0, 40]
    // w_dot = [0, 0, -40 / 50] = [0, 0, -0.8] rad/s^2
    // expected_w_z = 0.0 + (-0.8 * 0.001) = -0.0008

    EXPECT_NEAR(nextSpinState.angularVelocity.z, -0.0008, 1e-6, "Gyroscopic Coupling: Auto-generates Z torque");
}