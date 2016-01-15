#include "highgui.h"
#include "imgproc.h"
#include "feature.h"
#include "draw.h"
#include <armadillo>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>

using namespace arma;
using namespace std;

int main() {
  srand(getpid());
  //cube I = load_image("600px-Line_with_outliers.svg.png");
  cube I = load_image("twolines.bmp");
  //cube I = load_image("Hough_Lines_Tutorial_Original_Image.jpg");
  mat E = canny2(rgb2gray(I), 7, 3.0);
  E = ((E % (E > 0.01)) > 0) % ones(E.n_rows, E.n_cols);

  // after we get the image, try to resize it
  double scale_factor = 1.0;
  double cc = E(0,0);
  uvec ind = find(E != cc);
  vector<vec> A;
  for (uword i : ind) {
    vec index = { (double)(i % E.n_rows), (double)(i / E.n_rows) };
    A.push_back(index);
  }

  disp_image("original", I);
  disp_image("canny", E);
  disp_wait();

  mat R = lines2(E, A);

  // filter out to only get 0.5 or higher radius
  int max_count = -1;
  for (uword i = 0; i < R.n_cols; i++) {
    if (R(0,i) < 0.5) {
      break;
    }
    max_count = (int)i;
  }
  if (max_count > -1) {
    R = R.cols(0,max_count);
  } else {
    R = mat();
  }

  // select the highest one and show it
  cube L = gray2rgb(E);

  for (int k = 0; k < R.n_cols; k++) {
    double theta = R(2, k) * M_PI / 180.0;
    double b = R(1, k) / sin(theta);
    double m = -cos(theta) / sin(theta);
    double y1 = b;
    double y2 = m*I.n_cols + b;
    draw_line(L, vec({ 0, 0, 1 }), {y1, 0}, {y2, (double)I.n_cols});
  }

  disp_image("hough_lines", L);
  disp_wait();
  return 0;
}
