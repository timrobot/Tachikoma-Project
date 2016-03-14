#ifndef __TK_VISUAL_H__
#define __TK_VISUAL_H__

#include <armadillo>
#include <opencv2/core/core.hpp>
#include <thread>
#include <mutex>

namespace visual {

  /// ENVIRONMENTAL VARIABLES
  cv::VideoCapture *_cam;
  KinectDevice *_kinect;
  KinectDevice2 *_kinect2;
  thread *videoCapThread;
  mutex *videoCapLock;
  bool keepRunning;

  /** Initialize the visual space
   */
  int init(bool use_single_cam = true,
           bool use_kinectv1 = false,
           bool use_kinectv2 = false);

  /** Destroy the visual space
   */
  void destroy(void);

  /** Check to see if the camera has been initialized
   *  @return true if a camera was found, else false
   */
  bool camera_opened(void);

  /** Check to see if the depth has been initialized
   *  @return true if a depth stream was found, else false
   */
  bool depth_opened(void);

  /** Try to grab a color frame
   *  @return an error code if the color frame cannot be accessed
   */
  int get_color(arma::cube &color_frame);

  /** Try to grab a depth frame
   *  @return an error code if the depth frame cannot be accessed
   */
  int get_depth(arma::mat &depth_frame);

  /** A runnable thread for the visual system
   */
  void __run__(void);

}

#endif
