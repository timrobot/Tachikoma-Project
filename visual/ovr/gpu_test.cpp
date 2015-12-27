#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "highgui.h"
#include "ovr.h"
#include "gcube.h"

#define DCAMFPS 30

int main(int argc, char **argv) {
  cv::namedWindow("hud");

  double offset = 0.15;
  gcube limg(argv[1]);
  gcube rimg(argv[2]);
  gcube combined;
  combined = ovr_image(limg, rimg, offset);

  printf("combine the images %zd %zd %zd %zd\n", limg.n_rows, limg.n_cols, rimg.n_rows, rimg.n_cols);

  disp_gcube("hud", combined);
  disp_wait();
  return 0;
}
