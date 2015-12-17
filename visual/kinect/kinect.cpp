#include <cmath>
#include <cstdio>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "kinect.h"

using namespace cv;
using namespace std;

//static int kinectCount;

/* This file accesses the Kinect Device and gets its video and depth frames. If a depth frame is deteced, a new distance frame is created as well */

KinectDevice::KinectDevice(freenect_context *_ctx, int _index) :
    Freenect::FreenectDevice(_ctx, _index),
    depth_buffer(FREENECT_DEPTH_11BIT),
    video_buffer(FREENECT_VIDEO_RGB),
    gamma_buffer(2048),
    new_depth_frame(false),
    new_video_frame(false),
    depthMat(Size(640, 480), CV_16UC1),
    videoMat(Size(640, 480), CV_8UC3),
    new_distance_frame(false),
    distanceMat(Size(640, 480), CV_64F),
    rows(640),
    cols(480) {
  int i;
  for (i = 0; i < 2048; i++) {
    float v = i / 2048.0;
    v = pow(v, 3) * 6;
    this->gamma_buffer[i] = v * 6 * 256;
  }
}

KinectDevice::~KinectDevice() {
}

void KinectDevice::DepthCallback(void *data, uint32_t timestamp) {
  this->depth_lock.lock();
  this->depthMat.data = (uint8_t *)(uint16_t *)data;
  this->new_depth_frame = true;
  this->new_distance_frame = true;
  this->depth_lock.unlock();
}

void KinectDevice::VideoCallback(void *data, uint32_t timestamp) {
  this->video_lock.lock();
  this->videoMat.data = (uint8_t *)data;
  this->new_video_frame = true;
  this->video_lock.unlock();
}

bool KinectDevice::getDepth(Mat& output) {
  this->depth_lock.lock();
  if (this->new_depth_frame) {
    this->depthMat.copyTo(output);
    this->new_depth_frame = false;
    this->depth_lock.unlock();
    return true;
  } else {
    this->depthMat.copyTo(output);
    this->depth_lock.unlock();
    return false;
  }
}

bool KinectDevice::getVideo(Mat& output) {
  this->video_lock.lock();
  if (this->new_video_frame) {
    cvtColor(this->videoMat, output, COLOR_RGB2BGR);
    this->new_video_frame = false;
    this->video_lock.unlock();
    return true;
  } else {
    cvtColor(this->videoMat, output, COLOR_RGB2BGR);
    this->video_lock.unlock();
    return false;
  }
}

bool KinectDevice::getDistance(Mat &output) {
  this->depth_lock.lock();
  if (this->new_distance_frame) {
    for (int y = 0; y < this->depthMat.rows; y++) {
      for (int x = 0; x < this->depthMat.cols; x++) {
        this->distanceMat.at<double>(y, x) = raw2meters(this->depthMat.at<uint16_t>(y, x));
      }
    }
    this->new_distance_frame = false;
    this->depth_lock.unlock();
    return true;
  } else {
    this->distanceMat.copyTo(output);
    this->depth_lock.unlock();
    return false;
  }
}

double KinectDevice::raw2meters(uint16_t raw) {
  // stephane maganate
  return (0.1236 * tan((double)raw / 2842.5 + 1.1863));
}
