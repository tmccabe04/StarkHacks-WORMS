#include "brain.h"
#include <fcntl.h>
#include <vector>
#include <string>
#include <sstream>

void send_to_minion(Minion* m, const std::string& cmd) {
    int pipe_fd = open(m->pipe_path, O_WRONLY);
    if (pipe_fd != -1) {
        write(pipe_fd, cmd.c_str(), cmd.length());
        close(pipe_fd);
    }
}

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
        float dx = target_x - closest->x;
        float dy = target_y - closest->y;
        
        float dist = std::sqrt(dx*dx + dy*dy);
        float angle = std::atan2(dy, dx) * 180.0f / M_PI;

        std::stringstream ss_turn, ss_move;
        ss_turn << "TURN " << angle << "\n";
        ss_move << "MOVE " << dist << "\n";

        send_to_minion(closest, ss_turn.str());
        send_to_minion(closest, ss_move.str());
        
        closest->x = target_x;
        closest->y = target_y;
        
        std::cout << "Dispatched TURN " << angle << " and MOVE " << dist 
                  << " to minion " << closest->pid << std::endl;
    }
}
