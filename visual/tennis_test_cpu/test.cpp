#include "draw.h"
#include "highgui.h"
#include "imgproc.h"
#include "feature.h"
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

/** Slight hack to the OpenCV 3.0 API
 */
arma::mat opencv_hough_circle(arma::mat &m) {
  arma::cube I(m.n_rows, m.n_cols, 1);
  I.slice(0) = m * 255;
  Mat inputImage = cvt_arma2opencv(I);
  vector<Vec3f> circles;
  HoughCircles(inputImage, circles, CV_HOUGH_GRADIENT, 1, 40, 100, 50, 10, 0);
  arma::mat pts(3, circles.size());
  for (int j = 0; j < circles.size(); j++) {
    pts(0, j) = circles[j][0];
    pts(1, j) = circles[j][1];
    pts(2, j) = circles[j][2];
  }
  return pts;
}

arma::cube filter_color(arma::cube I, arma::vec c, double tolerance) {
  for (arma::uword i = 0; i < I.n_rows; i++) {
    for (arma::uword j = 0; j < I.n_cols; j++) {
      if (abs(I(i, j, 0) - c(0)) < tolerance &&
          abs(I(i, j, 1) - c(1)) < tolerance &&
          abs(I(i, j, 2) - c(2)) < tolerance) {
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
  //arma::cube I1 = load_image("train_ball.png");
  // there are two balls, need to do hough transform
  //arma::cube I2 = load_image("test_ball2.jpg");
  arma::cube inputImage = load_image("test_ball2.jpg");
  disp_image("test image", inputImage);

  // same with the tile, start with color segmentation
  /*arma::uword mx = I1.n_cols/2;
  arma::uword my = I1.n_rows/2;
  vector<arma::vec> hyp1 = {
    arma::vec({ I1(my, mx, 0), I1(my, mx, 1), I1(my, mx, 2) }),
    arma::vec({ I1(0,0,0), I1(0,0,1), I1(0,0,2) }) };
  vector<arma::vec> centroids1;
  I1 = hist_segment2(I1, 2, centroids1, 5, hyp1, true);

  //vector<arma::vec> hyp2 = { centroids1[0] };
  //vector<arma::vec> centroids2;
  //I2 = hist_segment2(I2, 3, centroids2, 5, hyp2, true);
  
  //arma::vec best_match = centroids2[0];
  //I2 = filter_color(I2, best_match);
  //I2 = filter_color(I2, centroids1[0], 0.3);

  disp_image("filtered", I1);
  disp_wait();*/

  arma::mat gray_image = rgb2gray(inputImage);
  arma::mat edges = (canny2(gray_image, 0.0, 0.04) > 0) %
      arma::ones<arma::mat>(gray_image.n_rows, gray_image.n_cols);
  arma::mat circles = opencv_hough_circle(edges);

  cout << circles << endl;

  for (int j = 0; j < MIN((int)circles.n_cols, 10); j++) {
    double x = circles(0, j);
    double y = circles(1, j);
    double r = circles(2, j);
    draw_circle(inputImage, arma::vec({ 1, 0, 0 }),
        arma::vec({ y, x }),
        r
//      arma::vec({ y - r, x - r }),
//      arma::vec({ y + r, x + r })
    );
  }

  disp_image("balls", inputImage);
  disp_wait();

  return 0;
}
