#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cstdio>
#include <cstdlib>
#include <signal.h>
#include "kinect.h"

using namespace cv;
using namespace std;
static char stopsig;

void stopkinect(int signum) {
  stopsig = 1;
}

uint16_t sigfun(double x) {
  return (uint16_t)(x * 150.0);
}

Vec3b raw2bgr(uint16_t raw) {
  Vec3b bgr;
  switch (raw / 410) {
    case 0:
      bgr[0] = raw * 255 / 410;
      break;
    case 1:
      bgr[0] = 255;
      raw -= 410;
      bgr[1] = raw * 255 / 410;
      break;
    case 2:
      bgr[1] = 255;
      raw -= 820;
      raw = 410 - raw;
      bgr[0] = raw * 255 / 410;
      break;
    case 3:
      bgr[1] = 255;
      raw -= 1230;
      bgr[2] = raw * 255 / 410;
      break;
    case 4:
      bgr[2] = 255;
      raw -= 1640;
      raw = 410 - raw;
      bgr[1] = raw * 255 / 410;
    default:
      bgr[0] = 255;
      bgr[1] = 255;
      bgr[2] = 255;
      break;
  }
  return bgr;
}

Mat getDepthAsBGR(Mat distanceMat) {
  Mat raw2bgrMat(640, 480, CV_8UC3);
  for (int y = 0; y < distanceMat.rows; y++)
    for (int x = 0; x < distanceMat.cols; x++)
      raw2bgrMat.at<Vec3b>(y, x) = raw2bgr(sigfun(distanceMat.at<double>(y, x)));
  return raw2bgrMat;
}

int main(int argc, char ** argv) {
  signal(SIGINT, stopkinect);

  Mat distance(Size(640, 480), CV_16UC1);

  // initialize
  Freenect::Freenect f;
  KinectDevice& kinect = f.createDevice<KinectDevice>(0);
  kinect.startVideo();
  kinect.startDepth();

  // get frames
  while (!stopsig) {
    kinect.getDepth(distance);
    /*Mat depth = getDepthAsBGR(distance);
    imshow("kinect", depth);
    if ((waitKey(1) & 0x0F) == 'q') {
      break;
    }*/
  }

  // end
  kinect.stopVideo();
  kinect.stopDepth();
  exit(0);
  return 0;
}
