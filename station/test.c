#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include "xboxctrl.h"

static int stopsig;

void stop(int signo) {
  printf("yo\n");
  stopsig = 1;
}

int main() {
  signal(SIGINT, stop);
  xboxctrl_t ctrl;
  xboxctrl_connect(&ctrl);
  serial_t connection;

  while (stopsig == 0) {
    xboxctrl_update(&ctrl);
    printf("%lf %lf %lf %lf\n", ctrl.LJOY.x, ctrl.LJOY.y, ctrl.RJOY.x, ctrl.RJOY.y);
  }

  xboxctrl_disconnect(&ctrl);
  return 0;
}
