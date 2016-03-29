#include "planner.h"
#include "visual/visual.h"          // for vision
#include "robot/baserobot.h"        // for robot movement and touch
#include "slam/particle_filter.h"   // for localization
#include "astar/astar.h"

int planner::init(BaseRobot *robot, bool en) {
  body = robot;
  enable = en;
  keepRunning = true;
}

void planner::start(void) {
  planningThread = new thread(planner::run);
}

void planner::stop(void) {
  keepRunning = false;
  if (planningThread) {
    planningThread->join();
    delete planningThread;
    planningThread = NULL;
  }
}

void planner::destroy(void) {
  planner::stop();
}

void planner::run(void) {
  // load a simple action graph
  while (keepRunning) {
    /////////////////////
    //    DANGEROUS    //
    /////////////////////
    if (autonomousEnable) {
      // in here, grab the feature space which we want
    } else {
      // manual control
      manual_input::update(); // autoconnect
      manual_input::control_robot(body);
    }
  }
}
