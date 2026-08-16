#include <iostream>

extern void runGravityTests();
extern void runQuaternionTests();
extern void runDynamicsTests();
extern void runTvcTest();

int main() {
    std::cout << "=== 6-DOF FLIGHT ENGINE TEST SUITE ===\n\n";

    runGravityTests();
    runQuaternionTests();
    runDynamicsTests();
    runTvcTest();

    std::cout << "\nTests completed.\n";
    return 0;
}