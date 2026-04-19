#include <Arduino.h>
#line 1 "/Users/tommy/Documents/projects/StarkHacks-WORMS/brain/brain.ino"
/*
 * Thomas McCabe
 *
 * brain.ino - center of project (spiraturally) initialize should do everything - no need for loop
 */

#include "src/brain.h"

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

