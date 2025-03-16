#include <iostream>
#include <thread>
#include <chrono>

int main() {
    std::cout << "Hello, World!" << std::endl;
    while (true) {
        std::cout << "running!" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    return 0;
}
    