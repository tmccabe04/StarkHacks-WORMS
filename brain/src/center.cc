#include "brain.h"
#include <fcntl.h>

void dispatch_mission(float target_x, float target_y) {
    std::lock_guard<std::mutex> lock(minions_mutex);
    if (minions.empty()) {
        std::cout << "No minions available." << std::endl;
        return;
    }

    Minion* closest = nullptr;
    float min_dist = -1.0f;

    for (auto& m : minions) {
        float dist = std::sqrt(std::pow(m.x - target_x, 2) + std::pow(m.y - target_y, 2));
        if (min_dist < 0 || dist < min_dist) {
            min_dist = dist;
            closest = &m;
        }
    }

    if (closest) {
        int pipe_fd = open(closest->pipe_path, O_WRONLY);
        if (pipe_fd != -1) {
            char cmd[64];
            snprintf(cmd, sizeof(cmd), "MOVE_TO %.2f %.2f", target_x, target_y);
            write(pipe_fd, cmd, strlen(cmd));
            close(pipe_fd);
            
            // Update minion state
            closest->x = target_x;
            closest->y = target_y;
            
            std::cout << "Dispatched mission to minion " << closest->pid 
                      << " at distance " << min_dist << std::endl;
        }
    }
}
