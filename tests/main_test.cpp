#include <iostream>

extern void runGravityTests();
extern void runQuaternionTests();
extern void runDynamicsTests();

int main() {
    std::cout << "=== 6-DOF FLIGHT ENGINE TEST SUITE ===\n\n";

    runGravityTests();
    runQuaternionTests();
    runDynamicsTests();

    std::cout << "\nTests completed.\n";
    return 0;
}