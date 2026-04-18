/*
 * Thomas McCabe
 *
 * brain.ino - center of project (spiraturally) initialize should do everything - no need for loop
 */

#include "brain.h"

void setup() {
  pid_t pid = fork();
  if (pid == 0) {
    // child - central control structure
    center();
  }
  else if (pid > 0) {
    // parent - wait for connections, never exit
    server();
  }
}

void loop() { ; }
