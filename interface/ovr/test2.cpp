#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "highgui.h"
#include "ovr.h"

#define DCAMFPS 30

int main(int argc, char **argv) {
  cv::namedWindow("hud");

  cv::Mat frame;
  double offset = 0.15;
  arma::cube limg = load_image(argv[1]);
  arma::cube rimg = load_image(argv[2]);
  arma::cube combined;

  for (;;) {
    // this is where we combine the images
    combined = ovr_image(limg, rimg, offset);
    disp_image("hud", combined);
    save_image("onepunch.png", combined);
    if (disp_keyPressed() >= 0) {
      break;
    }
  }
  return 0;
}
