#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/line_descriptor/descriptor.hpp>
#include <opencv2/stitching/detail/motion_estimators.hpp>
#include <vector>
#include <string>
#include <iostream>
#include <sys/types.h>
#include <unistd.h>
#include "highgui.h"
#include "imgproc.h"
#include "draw.h"
//#include "kcluster.h"

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;
using namespace cv::flann;
using namespace cv::line_descriptor;
using namespace cv::detail;
const int nmatches = 35;

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

arma::cube filter_obj(arma::cube I1, arma::cube I2) {
  vector<arma::vec> centroids1, centroids2;
  // THIS IS THE SEGMENTATION (goto imgproc.cpp)
  I1 = hist_segment2(I1, 2, centroids1, 5);

  arma::vec center_color = arma::vec({
      I1(I1.n_rows / 2, I1.n_cols / 2, 0),
      I1(I1.n_rows / 2, I1.n_cols / 2, 1),
      I1(I1.n_rows / 2, I1.n_cols / 2, 2) });
  vector<arma::vec> hyp = { center_color };
  I2 = hist_segment2(I2, 5, centroids2, 5, hyp, true);
  arma::vec best_match = centroids2[0]; // pretend this was the color
  I2 = filter_color(I2, best_match);
  return I2;
}

int main(int argc, char *argv[]) {
  Mat img1 = imread("img01.png");
  Mat img2 = imread("test00.png");

  arma::cube i1 = cvt_opencv2arma(img1) / 255;
  arma::cube i2 = cvt_opencv2arma(img2) / 255;

  i1 = imresize2(i1, 320, 320);
  i2 = imresize2(i2, 320, 320);

  disp_image("train image", i1);
  disp_image("test image", i2);
  disp_wait();

  arma::cube I2 = filter_obj(i1, i2);
  arma::mat g = cvt_rgb2gray(I2);

  disp_image("segmented image", I2);
  disp_wait();

  // histogram algorithm here
  //

  int grows = g.n_rows;
  int gcols = g.n_cols;
  vector<int> colshist(gcols), rowshist(grows);
  for (int i = 0; i < grows; i++) {
    for (int j = 0; j < gcols; j++) {
      if(g(i,j)!=0){
        rowshist[i]++;
        colshist[j]++;
      }
    }
  }

  int yeti = 0; int ysum = 0;
  for(int i=0; i<grows; i++){
    yeti+=rowshist[i]*i;
    ysum+=rowshist[i];
  }
  double centroidy = (double)yeti/(double)ysum;
  int xeti=0; int xsum=0;
  for(int i=0; i<gcols; i++){
    xeti+=colshist[i]*i;
    xsum+=colshist[i];
  }
  double centroidx = (double)xeti/(double)xsum;

  I2 = edge2(I2);

  I2((int)centroidy, (int)centroidx, 0) = 1;
  I2((int)centroidy, (int)centroidx, 1) = 0;
  I2((int)centroidy, (int)centroidx, 2) = 0;
  disp_image("centroids?", I2);
  disp_wait();
  cout << centroidx << ", " << centroidy << endl;

  return 0;
}
