#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/line_descriptor/descriptor.hpp>
#include <opencv2/stitching/detail/motion_estimators.hpp>
#include <cvsba/cvsba.h>
#include <vector>
#include <string>
#include <iostream>
#include <sys/types.h>
#include <unistd.h>
#include "highgui.h"
#include "imgproc.h"
//#include "kcluster.h"

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;
using namespace cv::flann;
using namespace cv::line_descriptor;
using namespace cv::detail;
const int nmatches = 35;

// Camera Matrix for the PS3 Eye
Mat mtx(3, 3, CV_64F);

// Distortion Coefficients for the PS3 Eye
Mat dist(5, 1, CV_64F);

Mat drawMatches(Mat img1, vector<KeyPoint> kp1, Mat img2, vector<KeyPoint> kp2, vector<DMatch> matches) {
  int rows1 = img1.rows;
  int cols1 = img1.cols;
  int rows2 = img2.rows;
  int cols2 = img2.cols;

  Mat color1 = img1, color2 = img2;
//  cvtColor(img1, color1, COLOR_GRAY2BGR);
//  cvtColor(img2, color2, COLOR_GRAY2BGR);
  Mat out(MAX(rows1, rows2), cols1+cols2, CV_8UC3);
  for (int i = 0; i < rows1; i++) {
    for (int j = 0; j < cols1; j++) {
      out.at<Vec3b>(i, j) = color1.at<Vec3b>(i, j);
    }
  }
  for (int i = 0; i < rows2; i++) {
    for (int j = 0; j < cols2; j++) {
      out.at<Vec3b>(i, j+cols1) = color2.at<Vec3b>(i, j);
    }
  }

  for (DMatch m : matches) {
    int img1_idx = m.queryIdx;
    int img2_idx = m.trainIdx;

    int x1 = (int)kp1[img1_idx].pt.x;
    int y1 = (int)kp1[img1_idx].pt.y;
    int x2 = (int)kp2[img2_idx].pt.x;
    int y2 = (int)kp2[img2_idx].pt.y;

    Vec3b color(rand() % 255, rand() % 255, rand() % 255);
    circle(out, Point(x1,y1), 4, color, 1);
    circle(out, Point(x2+cols1,y1), 4, color, 1);
    line(out, Point(x1,y1), Point(x2+cols1,y2), color, 1);
  }
  return out;
}

void drawLines(Mat img1, Mat img2, Mat lines, vector<Point2f> pts1, vector<Point2f> pts2, Mat &ret1, Mat &ret2) {
  int n_rows = img1.rows;
  int n_cols = img1.cols;
  Mat color1, color2;
  cvtColor(img1, color1, COLOR_GRAY2BGR);
  cvtColor(img2, color2, COLOR_GRAY2BGR);
  for (int i = 0; i < lines.rows; i++) {
    Vec3f r = lines.at<Vec3f>(i, 0);
    Point2f pt1 = pts1[i];
    Point2f pt2 = pts2[i];
    Vec3b color(rand() % 255, rand() % 255, rand() % 255);
    int x0 = 0;
    int y0 = -r[2] / r[1];
    int x1 = n_cols;
    int y1 = -(r[2]+r[0]*n_cols)/r[1];
    line(color1, Point(x0,y0), Point(x1,y1), color, 1);
    circle(color1, pt1, 5, color, -1);
    circle(color2, pt2, 5, color, -1);
  }
  ret1 = color1;
  ret2 = color2;
}

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
        I(i, j, 0) = 1;
        I(i, j, 1) = 1;
        I(i, j, 2) = 1;
      }
    }
  }
  return I;
}

int main(int argc, char *argv[]) {
  Mat img1 = imread("img01.png");
  Mat img2 = imread("test00.png");

  arma::cube I1 = cvt_opencv2arma(img1) / 255;
  arma::cube I2 = cvt_opencv2arma(img2) / 255;

  // segment the images
  printf("segmenting the images\n");
  vector<arma::vec> centroids1, centroids2;
  I1 = hist_segment2(I1, 2, centroids1, 5);
  arma::vec center_color = arma::vec({
      I1(I1.n_rows / 2, I1.n_cols / 2, 0),
      I1(I1.n_rows / 2, I1.n_cols / 2, 1),
      I1(I1.n_rows / 2, I1.n_cols / 2, 2) });
  vector<arma::vec> hyp = { center_color };
  I2 = hist_segment2(I2, 4, centroids2, 5, hyp, true);
  arma::vec best_match = centroids2[0]; // pretend this was the color
  I2 = filter_color(I2, best_match);
  I1 = edge2(I1);
  I2 = edge2(I2);
  I1 /= I1.max();
  I2 /= I2.max();
  //I2 = edge2(I2, 7, false, true);
  printf("finished\n");

  img1 = cvt_arma2opencv(I1 * 255);
  img2 = cvt_arma2opencv(I2 * 255);

  disp_image("segmented1", I1);
  disp_image("segmented2", I2);
  disp_wait();

  printf("matching feature points\n");

  Mat gray1, gray2;
  cvtColor(img1, gray1, COLOR_BGR2GRAY);
  cvtColor(img2, gray2, COLOR_BGR2GRAY);

  // feature detector
  Ptr<SIFT> sift = SIFT::create();
  vector<KeyPoint> kp1, kp2;
  Mat des1, des2;
  sift->detectAndCompute(gray1, noArray(), kp1, des1);
  sift->detectAndCompute(gray2, noArray(), kp2, des2);

  Ptr<IndexParams> indexParams = makePtr<KDTreeIndexParams>(5);
  Ptr<SearchParams> searchParams = makePtr<SearchParams>(50);
  FlannBasedMatcher flann(indexParams, searchParams);
  vector< vector<DMatch> > matches;
  flann.knnMatch(des1, des2, matches, 2);

  // ratio test (as per Lowe's paper)
  vector<DMatch> good;
  vector<Point2f> pts1, pts2;
  good.clear();
  pts1.clear();
  pts2.clear();
  for (int i = 0; i < matches.size(); i++) {
    DMatch m = matches[i][0];
    DMatch n = matches[i][1];
    if (m.distance < 0.8 * n.distance) {
      good.push_back(m);
      pts2.push_back(kp2[m.trainIdx].pt);
      pts1.push_back(kp1[m.queryIdx].pt);
    }
  }

  Mat matchFrame = drawMatches(img1, kp1, img2, kp2, good);
  imshow("matches", matchFrame);
  waitKey(0);

  return 0;
}
