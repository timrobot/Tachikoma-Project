#include <string.h>
#include <stdio.h>
#include "speech.h"

char prevmsg[64];
char phrase[64];

int main() {
  speech::start();

  while (true) {
    char *msg;
    if ((msg = speech::listen())) {
      printf("message: %s\n", msg);
      if (strstr(msg, "fetch")) {
        speech::say("I have received the command to fetch\n");
      } else if (strstr(msg, "stop")) {
        speech::say("I will stop now\n");
      }
    }
  }

  speech::stop();
  return 0;
}
