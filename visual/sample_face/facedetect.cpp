#include <opencv2/imgproc/imgproc.hpp>  // Gaussian Blur
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/highgui/highgui.hpp>  // OpenCV window I/O
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/contrib/detection_based_tracker.hpp>
#include <stdio.h>
#include <string>
#include <vector>
#include <iostream>

using namespace cv;
using namespace std;

vector<Rect> faces;

int main(int argc, char *argv[]) {

  // The header files inside opencv2/contrib are for programs that have been contributed or are experimental

  // Declare the parameters that are needed for the tracker to run;
  // All the parameters are mostly the same that you use for haar cascades.

  DetectionBasedTracker::Parameters param;
  param.maxObjectSize = 400;
  param.maxTrackLifetime = 20;
  param.minDetectionPeriod = 7;
  param.minNeighbors = 3;
  param.minObjectSize = 20;
  param.scaleFactor = 1.1;

  //The constructer is called with the cascade of choice and the Parameter structure

  DetectionBasedTracker tracker = DetectionBasedTracker("haarcascade_frontalface_alt.xml",param);
  tracker.run();
  VideoCapture cap(0);
  Mat img,gray;
  cv::Rect_<int> face_i;
  cv::namedWindow("Detection Based Tracker",cv::WINDOW_NORMAL);
  cv::setWindowProperty("Detection Based Tracker", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

  for (;;) {
    cap>>img;
    cv::cvtColor(img,gray,CV_RGB2GRAY);
    // The class instance is run on a gray image.
    tracker.process(gray);
    // The results are a vector of Rect that enclose the item being tracked
    tracker.getObjects(faces);
    // if(faces.size() == 0) tracker.resetTracking();
    for (int i = 0; i < faces.size(); i++) {
      face_i = faces[i];
      // Make a rectangle around the detected item
      rectangle(img, face_i, CV_RGB(0, 255,0), 3);
      string box_text = format("Tracked Area");
      int pos_x = std::max(face_i.tl().x - 10, 0);
      int pos_y = std::max(face_i.tl().y - 10, 0);
      // And now put it into the image:
      putText(img, box_text, Point(pos_x, pos_y), FONT_HERSHEY_SIMPLEX, 1.0, CV_RGB(0,255,0), 2.0);
    }
    cv::imshow("Detection Based Tracker",img);      // Show the results.
    if(cv::waitKey(33) == 27) break;
  }
  tracker.stop();
  return 0;
}
