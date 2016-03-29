#ifndef __TK_IPCDB_H__
#define __TK_IPCDB_H__

#include "robot/baserobot.h"
#include "robot/tachikoma/tachikoma.h"
#include "robot/armv1/armv1.h"
#include "visual/highgui.h"

#ifdef __NVCC__
#include "visual/gcube.h" // only for nvcc
#endif

#include "visual/imgproc.h"
#include "visual/feature.h"
#include "visual/draw.h"
//#include "speech/tts/tts.h"
#include "slam/particle_filter/pfilter.h"
#include "planner/astar/astar.h"
#include "interface/sdl2window.h"

#include <armadillo>
#include <mutex>

BaseRobot *robot;

arma::cube screen;

mutex rgbframe_lock;
#ifdef __NVCC__
gcube rgbframe;
#else
arma::cube rgbframe;
#endif

mutex depthframe_lock;
#ifdef __NVCC__
gcube depthframe;
#else
arma::mat depthframe;
#endif

#endif
