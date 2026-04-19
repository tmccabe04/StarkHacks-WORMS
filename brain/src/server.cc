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

#define PORT 10101

void sigchld_handler(int s) {
    int saved_errno = errno;
    while(waitpid(-1, NULL, WNOHANG) > 0);
    errno = saved_errno;
}

void handleChild(int client_fd) {
    char buffer[1024];
    while (true) {
        ssize_t bytes_read = recv(client_fd, buffer, sizeof(buffer) - 1, 0);
        if (bytes_read <= 0) {
            if (bytes_read == 0) printf("Minion disconnected\n");
            else perror("recv error");
            break;
        }
        buffer[bytes_read] = '\0';
        printf("Received data from minion: %s\n", buffer);

        const char* goal = "GOAL: MOVE_FORWARD";
        if (send(client_fd, goal, strlen(goal), 0) < 0) {
            perror("send error");
            break;
        }
        sleep(1);
    }
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
    if (sigaction(SIGCHLD, &sa, NULL) == -1) {
        perror("sigaction");
        return;
    }

    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("socket failed");
        return;
    }

    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) {
        perror("setsockopt");
        return;
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        perror("bind failed");
        return;
    }

    if (listen(server_fd, 20) < 0) {
        perror("listen");
        return;
    }

    char ip_str[INET_ADDRSTRLEN];
    printf("Server listening on %s:%d\n", inet_ntop(AF_INET, &address.sin_addr, ip_str, INET_ADDRSTRLEN), PORT);

    while (true) {
        if ((client_fd = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen)) < 0) {
            perror("accept");
            continue;
        }

        printf("New minion connected\n");

        pid_t pid = fork();
        if (pid == 0) {
            close(server_fd);
            handleChild(client_fd);
        } else if (pid > 0) {
            close(client_fd);
        } else {
            perror("fork failed");
            close(client_fd);
        }
    }
}
