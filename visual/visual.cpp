#include "visual.h"

using namespace cv;

int visual::init(bool use_single_cam, bool use_kinectv1, bool use_kinectv2) {
  // starts up the thread that analyzes video input
  // unforunately the framework is not as flexible to accept multiple input devices,
  // so for now just try to get the kinect working as a device by itself

  // init all the values
  _cam = NULL;
  _kinect = NULL;
  _kinect2 = NULL;
  keepRunning = true;

  if (use_single_cam) {
    _cam = new VideoCapture(0);
  }

  if (use_kinectv1) {
    Freenect::Freenect f;
    KinectDevice &kinect = f.createDevice<KinectDevice>(0);
    _kinect = &kinect;
    kinect.startVideo(); // thread!
    kinect.startDepth(); // thread!
  }

  if (use_kinectv2) {
    // dont do anything yet
  }

  // start the thread for constant video update
  videoCapLock = new mutex;
  videoCapThread = new thread(visual::__run__);
}

void visual::destroy(void) {
  keepRunning = false;
  if (videoCapThread) {
    videoCapThread->join();
  }
  if (videoCapLock) {
    delete videoCapLock;
    videoCapLock = NULL;
  }
}
