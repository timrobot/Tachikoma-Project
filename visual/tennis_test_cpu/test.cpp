#include "draw.h"
#include "highgui.h"
#include "imgproc.h"
#include <vector>

using namespace std;

arma::cube filter_color(arma::cube I, arma::vec c) {
  for (arma::uword i = 0; i < I.n_rows; i++) {
    for (arma::uword j = 0; j < I.n_cols; j++) {
      if (I(i, j, 0) == c(0) &&
          I(i, j, 1) == c(1) &&
          I(i, j, 2) == c(2)) {
        I(i, j, 0) = c(0);
        I(i, j, 1) = c(1);
        I(i, j, 2) = c(2);
      } else {
        I(i, j, 0) = 0;
        I(i, j, 1) = 0;
        I(i, j, 2) = 0;
      }
    }
  }
  return I;
}

int main() {
  srand(271828183);
  arma::cube I1 = load_image("train_ball.png");
  // there are two balls, need to do hough transform
  arma::cube I2 = load_image("test_ball.jpg");
  arma::cube I3 = I2;

  // same with the tile, start with color segmentation
  arma::uword mx = I1.n_cols/2;
  arma::uword my = I1.n_rows/2;
  vector<arma::vec> hyp1 = {
    arma::vec({ I1(my, mx, 0), I1(my, mx, 1), I1(my, mx, 2) }),
    arma::vec({ I1(0,0,0), I1(0,0,1), I1(0,0,2) }) };
  vector<arma::vec> centroids1;
  I1 = hist_segment2(I1, 2, centroids1, 5, hyp1, true);

  vector<arma::vec> hyp2 = { centroids1[0] };
  vector<arma::vec> centroids2;
  I2 = hist_segment2(I2, 4, centroids2, 5, hyp2, true);
  
  arma::vec best_match = centroids2[0];
  I2 = filter_color(I2, best_match);

  disp_image("filtered", I2);
  disp_wait();

  I3 = edge2(I3, 21, 3.0) * 16.0;
  disp_image("edges", I3);
  disp_wait();

  return 0;
}
