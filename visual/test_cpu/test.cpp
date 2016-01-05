#include "highgui.h"
#include <armadillo>
#include <iostream>
#include "imgproc.h"

using namespace arma;
using namespace std;

int main() {
  mat G = cvt_rgb2gray(load_image("butterfly.jpg"));
  mat K = gauss2(25, 9.0);

  mat blurred = conv2(G, K);

  disp_image("img", blurred);
  disp_wait();
  save_image("test_cpu_blur_25_9.png", blurred);

  return 0;
}
