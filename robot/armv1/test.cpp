#include "armv1.h"
#include "defs.h"
#include <iostream>
#include <signal.h>
#include <cstdint>
#include <fstream>
#include <njson/json.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <thread>

#define KEYID(k) ((k)-'a')

using namespace arma;
using namespace std;
using namespace cv;
using json = nlohmann::json;
static bool stopsig;
Arm arm;
thread ui;
static int iValue[6];

double deg2rad(double deg) {
  return deg * M_PI / 180.0;
}

double rad2deg(double rad) {
  return rad * 180.0 / M_PI;
}

// These are the callbacks
void joint0callback(int iValue, void *userData) {
  double val = deg2rad(iValue - 90);
  arm.set_joint(val, 0);
}

void joint1callback(int iValue, void *userData) {
  double val = deg2rad(iValue - 90);
  arm.set_joint(val, 1);
}

void joint2callback(int iValue, void *userData) {
  double val = deg2rad(iValue);
  arm.set_joint(val, 2);
}

void joint3callback(int iValue, void *userData) {
  double val = deg2rad(iValue - 90);
  arm.set_joint(val, 3);
}

void joint4callback(int iValue, void *userData) {
  double val = deg2rad(iValue - 90);
  arm.set_joint(val, 4);
}

void joint5callback(int iValue, void *userData) {
  double val = deg2rad(iValue);
  arm.set_joint(val, 5);
}

void stopsignal(int) {
  stopsig = true;
}

void run_ui(void) {
  string winname = "Press 'q' to quit";
  namedWindow(winname);
  createTrackbar("Joint0", winname, &iValue[0], 180, joint0callback, NULL);
  createTrackbar("Joint1", winname, &iValue[1], 180, joint1callback, NULL);
  createTrackbar("Joint2", winname, &iValue[2], 180, joint2callback, NULL);
  createTrackbar("Joint3", winname, &iValue[3], 180, joint3callback, NULL);
  createTrackbar("Joint4", winname, &iValue[4], 180, joint4callback, NULL);
  createTrackbar("Claw", winname, &iValue[5], 180, joint5callback, NULL);
  cv::Mat bufimage(480, 640, CV_8UC3);
  char numbuf[64];
  for (;;) {
    bufimage = Scalar(0, 0, 0);

    vec values = arm.sense();
    for (int i = 0; i < (int)values.n_elem; i++) {
      values(i) = rad2deg(values(i));
    }
    values += vec({ 90, 90, 0, 90, 90, 0 });

    for (int i = 0; i < (int)values.n_elem; i++) {
      Point topleft(80, 20 + i * 80);
      Point btmright(values(i) + 80, 60 + i * 80);
      rectangle(bufimage, topleft, btmright, Scalar(0, 255, 0));
      sprintf(numbuf, "%0.2lf", values(i));
      putText(bufimage, string(numbuf), Point(20, 60 + i * 80), FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0), 1, 8, false);
    }

    // display the forward kinematics
    sprintf(numbuf, "%0.2lf", 
    putText(bufimage, string(numbuf), Point(20, 60 + i * 80), FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0), 1, 8, false);

    imshow(winname, bufimage);
    int k = waitKey(30);
    if ((k & 0x7f) == 'q') {
      break;
    }
  }
}

void start_arm() {
  arm.connect();
  if (!arm.connected()) {
    printf("[ARM TEST] Not connected to anything, disconnecting...\n");
    arm.disconnect();
    exit(1);
  }

  string params;
  ifstream params_file("calib_params.json");
  string temp;
  while (getline(params_file, temp)) {
    params += temp;
  }
  params_file.close();
  arm.set_calibration_params(json::parse(params));
  if (!arm.calibrated()) {
    arm.disconnect();
    exit(1);
  }
}

void stop_arm() {
  arm.disconnect();
}

int main() {
  signal(SIGINT, stopsignal);

  // connect to the arm 
  start_arm();
  sleep(1);
  vec values = arm.sense();
  arm.set_pose(values(0), values(1), values(2), values(3), values(4), values(5));
  iValue[0] = rad2deg(values(0)) + 90;
  iValue[1] = rad2deg(values(1)) + 90;
  iValue[2] = rad2deg(values(2));
  iValue[3] = rad2deg(values(3)) + 90;
  iValue[4] = rad2deg(values(4)) + 90;
  iValue[5] = rad2deg(values(5));

  // run loop
  thread ui_thread(run_ui);
  ui_thread.join(); // wait until the ui is finished

  for (int i = 0; i < 100; i++) {
    arm.set_pose(0, 0, 0, 0, 0, 0, false);
  }
  stop_arm();
  return 0;
}
