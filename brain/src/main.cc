#include "brain.h"
#include <thread>
#include <string>

int main() {
    std::thread server_thread(server);
    server_thread.detach();

    std::cout << "Brain running. Enter mission: <x> <y>" << std::endl;
    float x, y;
    while (std::cin >> x >> y) {
        dispatch_mission(x, y);
    }
    return 0;
}
