#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include "xboxctrl.h"
#include "slambot.h"
#include <armadillo>
#include <pthread.h>

static int stopsig;
using namespace arma;

static xboxctrl_t ctrl;
static pthread_t thread;
void *thread_ctrl_update(void *args) {
  while (!stopsig) {
    xboxctrl_update(&ctrl);
  }
  return NULL;
}

void stop(int signo) {
  printf("yo\n");
  stopsig = 1;
}

double deadzone(double v) {
  if (-0.3 < v && v < 0.3) {
    return 0.0;
  }
  return v;
}

int main() {
  signal(SIGINT, stop);
  xboxctrl_connect(&ctrl);
  slambot bot;

  pthread_create(&thread, NULL, thread_ctrl_update, NULL);

  while (!stopsig) {
    double lx = deadzone(ctrl.LJOY.x);
    double ly = deadzone(ctrl.LJOY.y);
    double rx = deadzone(ctrl.RJOY.x);
//    double ry = deadzone(ctrl.RJOY.y);
    bot.send(vec({
      -ly - lx - rx,
      -ly + lx + rx,
      -ly + lx - rx,
      -ly - lx + rx,
      0}));
  }

  pthread_join(thread, NULL);

  xboxctrl_disconnect(&ctrl);
  return 0;
}
