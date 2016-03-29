#include "visual.h"
#include "ipcdb.h" // place the frame buffers into the ipcdb

int visual::start(bool use_single_cam, bool use_kinectv1, bool use_kinectv2) {
  // starts up the thread that analyzes video input
  // unforunately the framework is not as flexible to accept multiple input devices,
  // so for now just try to get the kinect working as a device by itself

  // init all the values
  _cam = NULL;
  _kinect = NULL;
  _kinect2 = NULL;
  keepRunning = true;

  if (use_single_cam) {
    _cam = new cv::VideoCapture(0);
    videoCapThread = new thread(visual::__run__);
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

  return 0;
}

void visual::stop(void) {
  if (_cam) {
    delete _cam;
    _cam = NULL;
  }
  if (_kinect) {
    delete _kinect;
    _kinect = NULL;
  }
  if (_kinect2) {
    delete _kinect2;
    _kinect2 = NULL;
  }
}

void visual::destroy(void) {
  keepRunning = false;
  if (videoCapThread) {
    videoCapThread->join();
  }
  if (_cam) {
    delete _cam;
    _cam = NULL;
  }
  if (_kinect) {
    delete _kinect;
    _kinect = NULL;
  }
  if (_kinect2) {
    delete _kinect2;
    _kinect2 = NULL;
  }
  videoCapLock.unlock();
}

bool visual::camera_opened(void) {
  return _cam->isOpened() || _kinect || _kinect2;
}

bool visual::depth_opened(void) {
  return _cam->isOpened();
}

int get_color(arma::cube &color_frame) {
  color_frame = rgbframe;
  return 1;
}

int get_depth(arma::mat &depth_frame) {
  depth_frame = depthframe;
  return 1;
}

void visual::run(void) {
  while (keepRunning) {
    if (_cam) {
      cv::Mat &cv_image = _cam->read();
      if (rgbframe.n_elem == 0) {
        rgbframe = arma::cube(frame.rows, frame.cols, 3, fill::zeros);
      } else {
        rgbframe_lock.lock();
        for (arma::uword i = 0; i < image.n_rows; i++) {
          for (arma::uword j = 0; j < image.n_cols; j++) {
            // set the red pixel
            rgbframe(i, j, 0) = (double)(cv_image.at<cv::Vec3b>(i, j)[2]);
            // set the green pixel
            rgbframe(i, j, 1) = (double)(cv_image.at<cv::Vec3b>(i, j)[1]);
            // set the blue pixel
            rgbframe(i, j, 2) = (double)(cv_image.at<cv::Vec3b>(i, j)[0]);
          }
        }
        rgbframe_lock.unlock();
      }
    } else if (_kinect) {
    } else if (_kinect2) {
    }
  }
}
