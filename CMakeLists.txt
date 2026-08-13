cmake_minimum_required(VERSION 3.31.6)
project(FlightMechanics6DOF LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(FlightEngine 
	src/main.cpp
	src/math/Vector3D.cpp
	src/math/Quaternion.cpp
	src/sensors/IMU.cpp
	src/vehicle/Rocket.cpp
	src/environment/Atmosphere.cpp
	src/environment/GravityModel.cpp
	include/dynamics/State.h
	include/math/Matrix3x3.h
	include/math/Quaternion.h
	include/math/Vector3D.h
	include/sensors/IMU.h
	include/vehicle/Rocket.h
	include/environment/Atmosphere.h
	include/environment/GravityModel.h
)

 add_executable(run_tests
	tests/main_test.cpp
	tests/test_gravity.cpp
	tests/test_quaternion.cpp
	tests/test_dynamics.cpp
	src/environment/GravityModel.cpp
	src/environment/Atmosphere.cpp
	src/math/Vector3D.cpp
	src/math/Quaternion.cpp
	src/vehicle/Rocket.cpp
)

target_include_directories(FlightEngine PRIVATE include)
target_include_directories(run_tests PRIVATE include)