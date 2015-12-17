#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/wait.h>
#include <sys/types.h>
#include "visual.h"

int result;
int pipefd[2];
FILE *cmd_output;
char buf[1024];
int status;
int procID;
int CUR_MODE;

void psleep(double time) {
  struct timespec t;
  t.tv_nsec = (int)((time - ((double)(int)time)) * 1000000000);
  t.tv_sec = (int)time;
  nanosleep(&t, NULL);
}

int main() {
    printf("starting visual...\n");
    start_visual();
    printf("started\n");
    int x = 0;
    int t, f, r;
    while(1) {
        pose3d_t *pos = get_position(&f, &t, &r);
        if (!r || t == 2) {
        continue;
        }
        if (f) {
          printf("%d here's output: { %lf %lf %lf }\n", x, pos->x, pos->y, pos->z);
          x++;
          if(x % 100 == 0) {
            if(CUR_MODE == DETECT_BASKET) {
                CUR_MODE = DETECT_BALL;
                set_detection(DETECT_BALL);
            } else {
                CUR_MODE = DETECT_BASKET;
                set_detection(DETECT_BASKET);
            }
          }
        }else {
        printf("not found\n");
        }
        // this simulates while loop of decision engine
//        psleep(0.01);
    }

}
