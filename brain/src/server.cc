/*
 * Thomas McCabe
 *
 * wait for connection, and call the relevant function
 */

#include "src/brain.h"

int server() {
  while (1) {
    // wait for bluetooth connection logic
    pid_t pid = fork();
    if (pid == 0) {
      // child - start talking to robot
      handleChild();
    }
    else if (pid < 0) {
      perror("fork error");
      return;
    }
  }
}
