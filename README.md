# FlightMechanics6DOF

A modular C++17 flight dynamics engine for simulating the six-degree-of-freedom (6-DOF) motion of rocket vehicles.

The project is being developed as a physics-first foundation for rocket flight mechanics, with an emphasis on clear mathematical models, modular architecture, numerical integration, vehicle dynamics, and sensor simulation.

> **Project status:** Active development. The API, physical models, coordinate conventions, and architecture may change as the simulation engine evolves.

## Overview

FlightMechanics6DOF models a rocket as a rigid body with translational and rotational motion.

The state of the vehicle contains:

- Position
- Linear velocity
- Attitude
- Angular velocity
- Propellant mass

The simulation integrates the nonlinear equations of motion numerically using a fourth-order Runge-Kutta (RK4) method.

The project is intentionally being built as a modular engine rather than as a visualization-focused simulator. This makes it possible to extend individual parts of the model independently, such as the atmosphere, aerodynamics, propulsion, sensors, estimation, and control systems.

## Current Features

- 6-DOF rigid-body flight dynamics
- Quaternion-based attitude representation
- Hamilton quaternion convention `(w, x, y, z)`
- Right-handed coordinate system
- Fourth-order Runge-Kutta (RK4) integration
- Variable propellant mass
- Fuel-dependent center-of-gravity migration
- Fuel-dependent inertia properties
- Thrust Vector Control (TVC)
- Thrust-generated moments
- Aerodynamic drag
- Aerodynamic moment from center-of-pressure / center-of-gravity offset
- Altitude-dependent gravity model
- Atmospheric density, pressure, and temperature model
- IMU simulation with configurable bias and Gaussian noise
- Modular C++ architecture using separate math, dynamics, environment, sensor, and vehicle components

## Mathematical Model

### State Vector

The vehicle state is represented by:

```text
x = [position, velocity, angular velocity, attitude, fuel mass]
```

with:

```text
position          : Vector3D
velocity          : Vector3D
angularVelocity   : Vector3D
attitude          : Quaternion
fuelMass          : scalar
```

The corresponding state derivative contains:

```text
velocity
linear acceleration
angular acceleration
quaternion derivative
fuel mass flow rate
```

### Translational Dynamics

The translational motion is governed by Newton's second law:

```text
m * a = F_total
```

where the total force is composed of the relevant forces acting on the vehicle, including gravity, thrust, and aerodynamic drag.

### Rotational Dynamics

The rotational dynamics follow the rigid-body Euler equation:

```text
I * omega_dot + omega x (I * omega) = M_total
```

or:

```text
omega_dot = I^-1 * [M_total - omega x (I * omega)]
```

This formulation includes the gyroscopic coupling term and therefore represents full rigid-body rotational dynamics rather than the simplified `I^-1 * M` model.

### Attitude Kinematics

Vehicle attitude is represented using quaternions:

```text
q = [w, x, y, z]
```

The quaternion derivative is computed from the body-frame angular velocity using the project's Hamilton convention.

Quaternion normalization is maintained during integration to avoid numerical drift.

## Coordinate Conventions

Coordinate-frame consistency is critical in a 6-DOF simulation.

### Current World Frame

The current implementation uses a **Y-Up** world convention.

### Planned World Frame

The long-term target is a **NED (North-East-Down)** navigation frame:

```text
+X = North
+Y = East
+Z = Down
```

Under the planned NED convention:

```text
Altitude = -Z
```

The transition to NED is part of the project roadmap.

### Vehicle Body Frame

The vehicle body frame is right-handed and defined as:

```text
+X = Nose / Forward
+Y = Right / Starboard
+Z = Belly / Down
```

### Attitude Convention

```text
Representation : Quaternion
Convention     : Hamilton
Components     : (w, x, y, z)
Rotation       : Right-handed
Angular rate   : Body-frame angular velocity
```

These conventions are treated as part of the mathematical definition of the engine and should remain explicit when new modules are added.

## Architecture

The project is organized around independent simulation components:

```text
FlightMechanics6DOF/
│
├── include/
│   ├── dynamics/
│   │   └── State.h
│   │
│   ├── environment/
│   │   ├── Atmosphere.h
│   │   └── GravityModel.h
│   │
│   ├── math/
│   │   ├── Matrix3x3.h
│   │   ├── Quaternion.h
│   │   └── Vector3D.h
│   │
│   ├── sensors/
│   │   └── IMU.h
│   │
│   └── vehicle/
│       └── Rocket.h
│
├── src/
│   ├── environment/
│   │   ├── Atmosphere.cpp
│   │   └── GravityModel.cpp
│   │
│   ├── math/
│   │   ├── Quaternion.cpp
│   │   └── Vector3D.cpp
│   │
│   ├── sensors/
│   │   └── IMU.cpp
│   │
│   ├── vehicle/
│   │   └── Rocket.cpp
│   │
│   └── main.cpp
│
├── CMakeLists.txt
└── README.md
```

The current architecture separates:

| Module | Responsibility |
|---|---|
| `math` | Vectors, matrices, and quaternions |
| `dynamics` | Vehicle state and state derivatives |
| `environment` | Atmosphere and gravity |
| `sensors` | Sensor measurement simulation |
| `vehicle` | Rocket properties and equations of motion |

This structure is intended to support future additions without turning the vehicle model into a monolithic class.

## Numerical Integration

The equations of motion are integrated using classical fourth-order Runge-Kutta:

```text
k1 = f(x, t)
k2 = f(x + dt*k1/2, t + dt/2)
k3 = f(x + dt*k2/2, t + dt/2)
k4 = f(x + dt*k3,   t + dt)

x_next = x + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
```

The same derivative evaluation framework is used for translational motion, rotational motion, attitude kinematics, and propellant depletion.

## Vehicle Dynamics

The rocket model currently accounts for several effects that influence the flight state.

### Variable Mass

Propellant consumption changes the vehicle mass during flight.

```text
m(t) = dry mass + remaining propellant
```

### Center of Gravity Migration

The center of gravity changes as propellant is consumed.

This affects the moment arms of forces such as thrust and aerodynamic forces.

### Variable Inertia

The vehicle inertia properties are updated as the propellant state changes.

This allows the rotational dynamics to respond to changes in mass distribution.

### Thrust Vector Control

The engine thrust vector can be gimbaled about the vehicle axes.

TVC therefore affects both:

```text
Force
+
Moment
```

through the thrust vector and its moment arm relative to the center of gravity.

### Aerodynamic Drag

The current aerodynamic model includes drag based on atmospheric density, vehicle speed, reference area, and drag coefficient.

The aerodynamic model is intentionally kept simple at this stage and is planned to evolve toward Mach- and angle-of-attack-dependent aerodynamic data.

### Aerodynamic Moment

The aerodynamic force can generate a moment when the center of pressure is offset from the center of gravity:

```text
M_aero = r_CP/CG x F_aero
```

## Environment

### Gravity

Gravity varies with altitude using an inverse-square relationship:

```text
g(h) = g0 * (R / (R + h))^2
```

where:

- `g0` is the reference gravitational acceleration
- `R` is the reference planetary radius
- `h` is altitude

### Atmosphere

The current atmosphere model provides:

- Temperature
- Pressure
- Density

as a function of altitude.

The model is intentionally lightweight and is expected to become more comprehensive as aerodynamic and Mach-dependent models are introduced.

## Sensor Simulation

The project includes an IMU model capable of producing simulated measurements from the vehicle's true state.

The current IMU model includes:

- Accelerometer measurements
- Gyroscope measurements
- Configurable sensor bias
- Gaussian measurement noise

Conceptually:

```text
measurement = true value + bias + noise
```

The sensor layer is intentionally separated from the vehicle dynamics so that future navigation and state-estimation algorithms can consume simulated measurements without depending directly on the true simulation state.

## Design Goals

The project is being developed around several principles:

1. **Physics first**  
   Mathematical consistency is prioritized over visual complexity.

2. **Modularity**  
   Environment, vehicle dynamics, mathematics, and sensors should remain independently replaceable.

3. **Explicit conventions**  
   Coordinate frames, quaternion conventions, and reference frames should never be implicit.

4. **Numerical stability**  
   Integration and state propagation should be treated as core engineering problems.

5. **Extensibility**  
   The engine should provide a foundation for future aerodynamics, propulsion, navigation, estimation, and control modules.

## Roadmap

The roadmap is intentionally incremental.

### Core Flight Mechanics

- [x] 6-DOF state representation
- [x] Quaternion attitude representation
- [x] RK4 integration
- [x] Variable mass
- [x] Variable inertia
- [x] CG migration
- [x] TVC force and moment
- [x] Gravity model
- [x] Basic atmospheric model
- [x] IMU sensor model

### Physics Improvements

- [ ] Complete NED world-frame migration
- [ ] Wind / relative-air-velocity model
- [ ] Speed-of-sound and Mach-number model
- [ ] Mach-dependent aerodynamic coefficients
- [ ] Angle-of-attack / sideslip modeling
- [ ] Aerodynamic force and moment model
- [ ] More realistic propulsion / thrust curves

### Verification and Validation

- [ ] Unit tests for mathematical primitives
- [ ] Quaternion validation tests
- [ ] Free-fall analytical comparison
- [ ] Rigid-body rotational-dynamics validation
- [ ] RK4 convergence tests
- [ ] Energy / momentum consistency checks where applicable
- [ ] Automated regression tests

### Sensors and Navigation

- [ ] More realistic IMU specific-force model
- [ ] Time-varying sensor bias / random walk
- [ ] GPS simulation
- [ ] Barometric altimeter simulation
- [ ] Extended Kalman Filter (EKF)
- [ ] Navigation-state estimation

### Guidance and Control

- [ ] PID control
- [ ] Attitude control
- [ ] TVC feedback control
- [ ] Guidance algorithms
- [ ] LQR-based control
- [ ] Full GNC architecture

## Build

### Requirements

- C++17 compatible compiler
- CMake 3.31 or newer

The project currently uses a CMake-based build system and does not require a GUI framework.

### Build with CMake

```bash
git clone https://github.com/OguzhanKucukoglu/FlightMechanics6DOF.git
cd FlightMechanics6DOF

cmake -S . -B build
cmake --build build
```

Run the generated executable from the build directory.

## Project Status

This is an actively developed research / learning-oriented flight mechanics project.

The current implementation should be considered a **development-stage 6-DOF flight dynamics engine**, not a validated aerospace-grade simulator.

The project prioritizes transparent equations, modular software architecture, and incremental validation. Higher-fidelity aerodynamic, propulsion, navigation, and control models will be introduced progressively.

## Author

**Oğuzhan Küçükoğlu**

## License

No license has been specified yet.
