#include <stdio.h>
#include <signal.h>
#include "stt.h"

static unsigned char stopsig;

void stop(int param) {
  stopsig = 1;
}

int main(int argc, char **argv) {
  char hyp[256];
  signal(SIGINT, stop);
  stt_start_listening();

  while (!stopsig) {
    int n = stt_listen(hyp);
    if (n) {
      printf("HYPOTHESIS: [%s]\n", hyp);
    }
  }

  stt_stop_listening();
  return 0;
}
