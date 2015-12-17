#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/wait.h>
#include <sys/types.h>
#include "stt.h"

static int lisinit;
static int lispid;
static int lisfd[2];
static char readbuf[256];
static int bufindex;
static char phrase[128];
static char interim[128];

/** Start recording
 *  @return 0 on success, otherwise -1
 */
int stt_start_listening(void) {
  if (!lisinit) {
    lisinit = 1;
    // start pocketsphinx process (to get rid of stderr)
    if (pipe(lisfd) == -1) {
      fprintf(stderr, "[stt] Error: cannot create the listener pipe\n");
      return -1;
    }
    lispid = fork();
    if (lispid == 0) {
      close(1);
      if (dup(lisfd[1]) == -1) {
        fprintf(stderr, "[stt] {child} Warning: cannot redirect listener output\n");
        exit(1);
      }
      close(2); // stop seeing stderr INFO
      execlp("./listen", "listen", NULL);
      printf("[stt] {child} Error: bad ps subprocess exec\n");
      exit(1);
    } else if (lispid > 0) {
      // make read nonblocking
      int flags;
      flags = fcntl(lisfd[0], F_GETFL, 0);
      fcntl(lisfd[0], F_SETFL, flags | O_NONBLOCK);
    } else {
      fprintf(stderr, "[stt] Error: could not create ps subprocess\n");
      return -1;
    }
    return 0;
  }
  return -1;
}

/** Gets the current state of signals for the sigframe
 *  @param buffer
 *    the buffer to copy the hypothesis to
 *  @return the length of the buffer
 */
int stt_listen(char *buffer) {
  int found;
  int bytesread;
  found = 0;
  while ((bytesread = read(lisfd[0], interim, 127)) > 0) {
    int i;
    interim[bytesread] = '\0';
    if (bufindex + bytesread >= sizeof(readbuf)) {
      memmove(readbuf, &readbuf[128], 128);
    }
    // state machine
    for (i = 0; i < bytesread; i++) {
      if (interim[i] == '\n') {
        if (strncmp(readbuf, "hypothesis:", 11) == 0) {
          bufindex -= 13;
          strncpy(phrase, &readbuf[12], bufindex);
          phrase[bufindex] = '\0';
          found = 1;
        }
        bufindex = 0;
      } else {
        readbuf[bufindex] = interim[i];
        if (interim[i] != '\0') {
          bufindex++;
        }
      }
      readbuf[bufindex] = '\0';
    }
  }
  if (found) {
    strcpy(buffer, phrase);
    return strlen(phrase);
  } else {
    return 0;
  }
}

/** Stop recording
*/
void stt_stop_listening(void) {
  if (lisinit) {
    close(lisfd[0]);
    close(lisfd[1]);
    kill(lispid, SIGINT);
    waitpid(lispid, NULL, 0);
    lisinit = 0;
  }
}
