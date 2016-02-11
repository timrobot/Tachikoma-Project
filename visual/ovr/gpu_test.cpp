#include "highgui.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "ovr.h"
#include "gcube.h"
#include "gpu_util.h"
#include <unistd.h>
#include <signal.h>
#include <sys/wait.h>
#include <cstdlib>

// can be 30-45ish
#define DELAY 5

static int stopsig;

void stopme(int signo) {
  stopsig = 1;
}

int main() {

  // remove all bad files
  int _ = system("rm -f frame*jpg");

  signal(SIGINT, stopme);
  // start the gstreamer thingy
  int pid;
  if ((pid = fork()) == 0) {
    execlp("./twovid_fuuu.sh", "./twovid_fuuu.sh", NULL);
    perror("Broken dreams");
  }

  cv::Mat file;
  // grab and display the frame
  cv::namedWindow("hud");
  double offset = 0.15;
  gcube limg, rimg, combined;
  cv::Mat out;
  limg.create(640, 480, 3);
  rimg.create(640, 480, 3);

  char buf[256];
  char buf2[256];

  for (size_t i = 0; !stopsig; i++) {
    sprintf(buf, "frame%zu.jpg", i);
    sprintf(buf2, "frame%zu.jpg", i+4);

    while (access(buf, F_OK) == -1) ; // detect for delays on the gst side
    while (access(buf2, F_OK) != -1) { // detect for delays on this side
      strcpy(buf, buf2);
      i += 4;
      sprintf(buf2, "frame%zu.jpg", i+4);
    }


    file = cv::imread(buf);
    // statically mapped numbers - assumption is that the frame is going to be 480x640 big
    limg.create(file, 0, 640, 0, 480, false);
    rimg.create(file, 0, 640, 480, 960, false);
    combined = ovr_image(limg, rimg, offset); // waste copy
    out = combined.cv_img();
    cv::imshow("hud", out);
    if (cv::waitKey(DELAY) >= 0) {
      break;
    }
  }

  kill(pid, SIGINT);
  sleep(1);
  kill(pid, SIGKILL);
  waitpid(pid, NULL, 0);

  _ = system("rm -f frame*jpg");

  return 0;
}
