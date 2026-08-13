# 6-DOF Rocket Flight Dynamics Engine

This project is a 6 Degrees of Freedom (6-DOF) flight simulation engine for rockets. It computes numerical integration of rigid body equations of motion.

## Simulation Framework Laws (Conventions)

In 6-DOF aerospace simulations, undefined conventions lead to catastrophic algorithmic failures. This project STRICTLY adheres to the following frameworks:

### 1. Coordinate Frame
*   **Current State:** Y-Up (Game Engine Standard).
*   **Target State (Roadmap):** Will transition to **NED (North-East-Down)**.
    *   `+X` = North
    *   `+Y` = East
    *   `+Z` = Down
    *   *Altitude will be calculated as `-Z`.*

### 2. Vehicle Body Frame
*   `+X` = Nose (Forward)
*   `+Y` = Right Wing (Starboard)
*   `+Z` = Belly (Down)

### 3. Attitude Kinematics
*   **Representation:** Quaternions $(q)$
*   **Convention:** Hamilton Convention $(w, x, y, z)$
*   **Rotation:** Right-handed coordinate system.
*   **Kinematic Derivative:** $\dot{q} = \frac{1}{2} q \otimes \omega$ (where $\omega$ is the body-frame angular velocity).