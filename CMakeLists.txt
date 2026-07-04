cmake_minimum_required(VERSION 3.31.6)
project(FlightMechanics6DOF LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(FlightEngine 
	src/main.cpp
	src/Vector3D.cpp
 "include/Vector3D.h" "include/Quaternion.h" "src/Quaternion.cpp" "include/State.h" "include/Rocket.h" "src/Rocket.cpp" "include/Matrix3x3.h")

# header dosyalarýnýn nerede aranacaðý
target_include_directories(FlightEngine PRIVATE include)