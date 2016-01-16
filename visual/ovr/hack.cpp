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
#define DELAY 10
#define IMGROWS 818
#define IMGCOLS 512

static int stopsig;

void stopme(int signo) {
  stopsig = 1;
}

int main() {

  // remove all bad files
  int _ = system("rm -f frame*jpg");

  cv::Mat file;
  // grab and display the frame
  cv::namedWindow("hud");
  double offset = 0.15;
  gcube limg, rimg, combined;
  cv::Mat out;
  limg.create(IMGROWS, IMGCOLS, 3);
  rimg.create(IMGROWS, IMGCOLS, 3);

  char buf[256];
  char buf2[256];


  for (size_t i = 0; !stopsig; i++) {
    sprintf(buf, "butterfly.jpg");

    file = cv::imread(buf);
    // statically mapped numbers - assumption is that the frame is going to be 480x640 big
    limg.create(file, 0, IMGROWS, 0, IMGCOLS, false);
    rimg.create(file, 0, IMGROWS, 0, IMGCOLS, false);
    combined = ovr_image(limg, rimg, offset); // waste copy
    out = limg.cv_img();
    cv::imshow("hud", out);
    if (cv::waitKey(DELAY) >= 0) {
      break;
    }
  }

  _ = system("rm -f frame*jpg");

  return 0;
}
