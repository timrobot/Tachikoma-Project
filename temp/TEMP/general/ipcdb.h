#ifndef __TK_IPCDB_H__
#define __TK_IPCDB_H__

#include "tachikoma.h"
#include "pfilter.h"

#include <armadillo>
#include <mutex>
#include <vector>
#include <opencv2/core/core.hpp>
#include "chili_landmarks.h"

// for stopping the robot, no matter what
static int stopsig;

// for markers
static chili_landmarks chili;
static std::mutex chili_lock;
static arma::mat chilitags(4, 20, arma::fill::zeros);
static std::mutex camframelock;
static cv::Mat cameraFrame;
static arma::vec bearpos;
static double bearsize;
static bool bearfound;

// for sending motion to the robot
static Tachikoma tachikoma;

// for enabling autonomous
static std::mutex autonomous_lock;
static bool auto_enable;
static bool auto_confirmed;
static bool manual_confirmed;

// for getting the position and map
static std::mutex pose_lock;
static arma::vec robot_pose(3, arma::fill::zeros); // x, y, theta
static pfilter pf; // !! takes a long time to blit
static std::mutex map_lock;
static sim_map globalmap;
static std::vector<sim_landmark> landmarks;

// for getting the planned path
static std::mutex path_lock;
static arma::mat pathplan(2, 0, arma::fill::zeros);
static bool dopose;
static arma::vec poseplan(3, arma::fill::zeros);
static double twistplan;
static double grabplan;

// for displaying stuff
static SDL_Surface *screen;

#endif
