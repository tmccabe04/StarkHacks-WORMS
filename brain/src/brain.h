#line 1 "/Users/tommy/Documents/projects/StarkHacks-WORMS/brain/src/brain.h"
#include <sys/types.h>
#include <unistd.h>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <mutex>
#include <cmath>

struct Minion {
    pid_t pid;
    char pipe_path[64];
    float x = 0.0f;
    float y = 0.0f;
};

extern std::vector<Minion> minions;
extern std::mutex minions_mutex;

void server();
void center();
void handleChild(int client_fd, pid_t pid);
void register_minion(pid_t pid);
void unregister_minion(pid_t pid);
void dispatch_mission(float target_x, float target_y);

