#include <stdio.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>
#include "rawrec.h"
#include "pswrap.h"

static unsigned char stopsig;

void stop_listening(int signo) {
  stopsig = 1;
}

int main(int argc, char *argv[]) {
  // pocketsphinx
  static pswrap_t info;
  
  // gstreamer
  rawrec_t rec[2];
  char *fname[2];
  char *buf;
  int rid;
  struct timespec waittime;
  char outbuf[256];

  // init the fields
  signal(SIGINT, stop_listening);
  fname[0] = "sample0.raw";
  fname[1] = "sample1.raw";
  waittime.tv_sec = 0;
  waittime.tv_nsec = 800000000;
  rid = 0;

  // start the recordings
  pswrap_init(&info);
  start_recording(&rec[0], fname[0]);
  start_recording(&rec[1], fname[1]);

  // do analysis loop
  while (!stopsig) {
    nanosleep(&waittime, NULL); // sleep the process to wait for a command
    stop_recording(&rec[rid]);
    // decipher
    sprintf(outbuf, "deciphering %d...\n", rid);
    if (write(1, outbuf, strlen(outbuf) + 1) == -1) {
      fprintf(stderr, "[listen] Cannot send outbuf\n");
    }
    if ((pswrap_decipher(&info, fname[rid], &buf)) > 0) {
      // print out the result
      sprintf(outbuf, "hypothesis:[%s]\n", buf);
      if (write(1, outbuf, strlen(outbuf) + 1) == -1) {
        fprintf(stderr, "[listen] Cannot send outbuf\n");
      }
    }
    unlink(fname[rid]);
    start_recording(&rec[rid], fname[rid]);
    rid = (rid + 1) % 2;
  }

  // stop the recordings
  stop_recording(&rec[0]);
  stop_recording(&rec[1]);

  // clean up files
  if (access(fname[0], F_OK) == 0) {
    unlink(fname[0]);
  }
  if (access(fname[1], F_OK) == 0) {
    unlink(fname[1]);
  }

  return 0;
}
