#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <cstdlib>
#include <vector>
#include <string>

using namespace std;
using namespace cv;

vector<string> read_images(const std::string &fmt) {
  char imgname[256];
  vector<string> images;
  for (int i = 0;; i++) {
    sprintf(imgname, fmt.c_str(), i);
    cout << "reading " << imgname << endl;
    if (access(imgname, F_OK) == -1) {
      break;
    }
    images.push_back(string(imgname));
  }
  return images;
}

int main(int argc, char *argv[]) {
  if (argc < 2) {
    printf("usage: %s [foldername]\n", argv[0]);
    return 0;
  }

  int w = 8;
  int h = 6;

  // termination criteria
  TermCriteria criteria(3, 30, 0.1);

  // prepare object points
  vector<Vec3f> objp;
  for (int i = 0; i < w * h; i++) {
    objp.push_back(Vec3f(i % w, i / w, 0));
  }

  // arrays to store object points and image points from all the images
  vector< vector<Vec3f> > objpoints;
  vector< vector<Vec2f> > imgpoints;

  vector<string> images = read_images(string(argv[1]) + "img%02d.png");

  for (string &fname : images) {
    Mat img = imread(fname);
    Mat gray;
    cvtColor(img, gray, CV_BGR2GRAY);

    // find the chess board corners
    vector<Point2f> corners;
    bool ret = findChessboardCorners(gray, Size(w, h), corners);

    // if found, add object points, image points
    if (ret) {
      objpoints.push_back(objp);

      cornerSubPix(gray, corners, Size(11,11), Size(-1,-1), criteria);
      vector<Vec2f> imgc;
      for (Point2f &p : corners) {
        imgc.push_back(Vec2f(p));
      }
      imgpoints.push_back(imgc);

      drawChessboardCorners(img, Size(w,h), corners, ret);
      imshow("img", img);
      waitKey(100);
    }
  }

  destroyAllWindows();

  // start calibration
  Mat firstImage = imread(images[0]);
  Mat mtx = Mat::eye(3,3,CV_64F);
  Mat dist(Size(8, 1), CV_64F);
  vector<Mat> rvecs, tvecs;
  int nrows = firstImage.rows;
  int ncols = firstImage.cols;
  printf("starting calculation\n");
  double reproj = calibrateCamera(objpoints, imgpoints, Size(ncols, nrows),
      mtx, dist, rvecs, tvecs, 0, criteria);

  printf("reprojecting...\n");

  // store the new reprojection
  int i = 0;
  for (string fname : images) {
    printf("modding: %s\n", fname.c_str());
    Mat img = imread(fname);
    nrows = img.rows;
    ncols = img.cols;
    Rect *roi;
    Mat newmtx = getOptimalNewCameraMatrix(mtx, dist, Size(ncols, nrows), 1, Size(ncols, nrows), roi);
    Mat dst;
    undistort(img, dst, mtx, dist, newmtx);

    imshow("distorted", img);
    imshow("undistorted", dst);
    waitKey(0);

    char nfname_[256];
    sprintf(nfname_, "mod_%simg%02d.png", argv[1], i++);
    string nfname = string(nfname_);

    cout << "writing to " << nfname << endl;
    imwrite(nfname, dst);

    printf("finished\n");
  }

  cout << "camera matrix:" << mtx << endl;
  cout << "distortion:" << dist << endl;
  cout << "reprojection error:" << reproj << endl;

  return 0;
}
