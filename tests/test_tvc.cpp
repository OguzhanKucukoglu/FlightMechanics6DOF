#include "../include/vehicle/Rocket.h"
#include "../include/environment/GravityModel.h"
#include "TestFramework.h"
#include <cmath>

void runTvcTest() {
	std::cout << "\n--- THRUST VECTOR CONTROL (TVC) TESTS ---\n";

    State state;
    state.position = Vector3D(0.0, 0.0, -1000.0);
    state.velocity = Vector3D(0.0, 0.0, 0.0);
    state.angularVelocity = Vector3D(0.0, 0.0, 0.0);
    state.orientation = Quaternion(1.0, 0.0, 0.0, 0.0);
    state.fuelMass = 50.0;

    LookupTable1D constantThrust;
    constantThrust.addPoint(0.0, 50000.0);
    constantThrust.addPoint(100.0, 50000.0);

    Rocket rocket(50.0, 50.0, 0.05, 3.0, 250.0, 0.0,
        10.0, 50.0, 50.0, 10.0, 50.0, 50.0,
        -1.5, -1.5, -3.0, state);

    rocket.setThrustCurve(constantThrust);

    // 1. PITCH TEST (+5 Degrees)
    // The motor nozzle rotates 5 degrees on the Y-axis (pitch).
    double pitch_angle = 5.0 * 3.14159265359 / 180.0;
    rocket.setGimbal(pitch_angle, 0.0);

    rocket.integrate(0.01); // Integrate 10 milliseconds
    State pitchState = rocket.getState();

    // Positive rotation on the Y-axis = According to the Right Hand Rule, the thumb points East (+Y),
    // fingers curl from the Z (Down) axis to the X (North) axis.
    // The Z component of the thrust vector becomes NEGATIVE (Upward).
    // Because the rear of the rocket is pushed upwards, the nose drops DOWNWARDS. (Negative Y Torque)

    EXPECT_NEAR(pitchState.angularVelocity.y < 0.0, true, 0.1, "Positive Pitch Gimbal -> Negative Y Angular Velocity (Nose Down)");
    EXPECT_NEAR(pitchState.velocity.z < 0.0, true, 0.1, "Positive Pitch Gimbal -> Negative Z Velocity (Pushes Up)");

    // 2. YAW TEST (+5 Degrees)
    // The motor nozzle rotates 5 degrees in the Z-axis (Yaw).
    Rocket yawRocket(50.0, 50.0, 0.05, 3.0, 250.0, 0.0,
        10.0, 50.0, 50.0, 10.0, 50.0, 50.0,
        -1.5, -1.5, -3.0, state);

    yawRocket.setThrustCurve(constantThrust);

    double yaw_angle = 5.0 * 3.14159265359 / 180.0;
    yawRocket.setGimbal(0.0, yaw_angle);

    yawRocket.integrate(0.01);
    State yawState = yawRocket.getState();

    // Positive rotation on the Z-axis = According to the Right Hand Rule, the thumb points downwards (+Z).
    // Fingers curl from X to Y. The Y-component of the thrust vector becomes POSITIVE (to the East).
    // Because the rear of the rocket is pushed to the right (+Y), the nose rotates to the left (-Y). (Negative Z Torque)

    EXPECT_NEAR(yawState.angularVelocity.z < 0.0, true, 0.1, "Positive Yaw Gimbal -> Negative Z Angular Velocity (Nose Left)");
    EXPECT_NEAR(yawState.velocity.y > 0.0, true, 0.1, "Positive Yaw Gimbal -> Positive Y Velocity (Pushes Right)");

    // 3. TVC MAGNITUDE CONSERVATION TEST 
    // Is the total thrust of the motor preserved even when the gimbal is forced to the extreme angles in Y and Z directions?
    Rocket magRocket(50.0, 50.0, 0.05, 3.0, 250.0, 0.0,
        10.0, 50.0, 50.0, 10.0, 50.0, 50.0,
        -1.5, -1.5, -3.0, state);

    magRocket.setThrustCurve(constantThrust);

    // We apply a 45-degree deviation in both the Pitch and Yaw axes (Endpoint)
    double extreme_angle = 45.0 * 3.14159265359 / 180.0;
    magRocket.setGimbal(extreme_angle, extreme_angle);
    magRocket.integrate(0.0);

    // To find the net force: Acceleration * Mass. Then subtract gravity to find the pure impulse (TVC)
    Vector3D netForce = magRocket.getCurrentAcceleration() * 100.0;
    Vector3D gravityForce = GravityModel::getGravity(-magRocket.getState().position.z) * 100.0;
    Vector3D actualThrustNED = netForce - gravityForce;

    EXPECT_NEAR(actualThrustNED.magnitude(), 50000.0, 1e-3, "TVC Thrust Magnitude is strictly preserved at extreme angles");

    // 4. BODY -> NED QUATERNION TEST (+90 Degrees Yaw)
    // If the rocket's nose (+X) is pointing East (+Y) relative to Earth, the TVC thrust should also point East
    State yaw90State = state;

    // 90-degree yaw rotation (around the Z-axis)
    double half_yaw90 = (90.0 * 3.14159265359 / 180.0) / 2.0;
    yaw90State.orientation = Quaternion(std::cos(half_yaw90), 0.0, 0.0, std::sin(half_yaw90));

    Rocket rotRocket(50.0, 50.0, 0.05, 3.0, 250.0, 0.0,
        10.0, 50.0, 50.0, 10.0, 50.0, 50.0,
        -1.5, -1.5, -3.0, yaw90State);

    rotRocket.setThrustCurve(constantThrust);

    // At Gimbal 0 (Motor is pushing straight behind the rocket in the +X direction)
    rotRocket.setGimbal(0.0, 0.0);
    rotRocket.integrate(0.0);

    Vector3D forceNED = (rotRocket.getCurrentAcceleration() * 100.0) - gravityForce;

    EXPECT_NEAR(forceNED.x, 0.0, 1e-3, "Body to NED Rotation: X force becomes 0");
    EXPECT_NEAR(forceNED.y, 50000.0, 1e-3, "Body to NED Rotation: Thrust is directed entirely to +Y (East)");
    EXPECT_NEAR(forceNED.z, 0.0, 1e-3, "Body to NED Rotation: Z force remains 0");
}