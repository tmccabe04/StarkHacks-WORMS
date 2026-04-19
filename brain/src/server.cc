#line 1 "/Users/tommy/Documents/projects/StarkHacks-WORMS/brain/src/server.cc"
#include "brain.h"
#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <fcntl.h>

#define PORT 10101

std::vector<Minion> minions;
std::mutex minions_mutex;

void register_minion(pid_t pid) {
    std::lock_guard<std::mutex> lock(minions_mutex);
    Minion m;
    m.pid = pid;
    snprintf(m.pipe_path, sizeof(m.pipe_path), "/tmp/worm_pipe_%d", pid);
    minions.push_back(m);
}

void unregister_minion(pid_t pid) {
    std::lock_guard<std::mutex> lock(minions_mutex);
    for (auto it = minions.begin(); it != minions.end(); ++it) {
        if (it->pid == pid) {
            minions.erase(it);
            break;
        }
    }
}

void sigchld_handler(int s) {
    int saved_errno = errno;
    while(waitpid(-1, NULL, WNOHANG) > 0);
    errno = saved_errno;
}

void handleChild(int client_fd, pid_t pid) {
    register_minion(pid);

    int pipe_fd = open(minions.back().pipe_path, O_RDONLY | O_NONBLOCK);
    char buffer[1024];

    while (true) {
        ssize_t bytes_read = read(pipe_fd, buffer, sizeof(buffer) - 1);
        if (bytes_read > 0) {
            buffer[bytes_read] = '\0';
            printf("Dispatching to minion %d: %s\n", pid, buffer);
            if (send(client_fd, buffer, bytes_read, 0) < 0) break;
        }

        bytes_read = recv(client_fd, buffer, sizeof(buffer) - 1, MSG_DONTWAIT);
        if (bytes_read == 0) break;
        sleep(1);
    }

    close(pipe_fd);
    unlink(minions.back().pipe_path);
    unregister_minion(pid);
    close(client_fd);
    exit(0);
}

void server() {
    int server_fd, client_fd;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);

    struct sigaction sa;
    sa.sa_handler = sigchld_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESTART;
    sigaction(SIGCHLD, &sa, NULL);

    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    bind(server_fd, (struct sockaddr *)&address, sizeof(address));
    listen(server_fd, 20);

    char ip_str[INET_ADDRSTRLEN];
    printf("Server listening on %s:%d\n", inet_ntop(AF_INET, &address.sin_addr, ip_str, INET_ADDRSTRLEN), ntohs(address.sin_port));

    while (true) {
        client_fd = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen);
        pid_t pid = fork();
        if (pid == 0) {
            close(server_fd);
            handleChild(client_fd, getpid());
        } else {
            close(client_fd);
        }
    }
}
