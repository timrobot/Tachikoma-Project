#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <cmath>
#include <unistd.h>
#include <sys/types.h>
#include <dirent.h>
#include <termios.h>
#include <assert.h>
#include <vector>
#include <iostream>
#include "slambot.h"

#define BASE 1
#define ARM 2
#define WBUFSIZE 128

using namespace arma;

static double limitf(double x, double min, double max);

SlamBot::SlamBot(void) : BaseRobot(999) {
  this->prev_motion = zeros<vec>(5);
  this->motion_const = ones<vec>(5) * 255.0;
  if (this->connect()) {
    this->reset();
    this->send(zeros<vec>(5));
  }
}

SlamBot::~SlamBot(void) {
  if (this->connected()) {
    this->send(zeros<vec>(5));
    this->reset();
    this->disconnect();
  }
}

int SlamBot::numconnected(void) {
  return this->connections.size();
}

void SlamBot::reset(void) {
  this->prev_motion.zeros();
}

void SlamBot::send(const vec &motion) {
  vec new_motion = motion;
  // safety check
  if (new_motion.n_elem != motion_const.n_elem) {
    new_motion = zeros<vec>(motion_const.n_elem);
  }

  // boundary check
  for (int i = 0; i < (int)new_motion.n_elem; i++) {
    new_motion(i) = limitf(new_motion(i), -1.0, 1.0);
  }

  new_motion %= motion_const;

  char msg[WBUFSIZE];
  for (int i = 0; i < (int)this->connections.size(); i++) {
    switch (this->ids[i]) {
      case BASE:
        // dont send dup speeds
        if (new_motion(0) == this->prev_motion(0) &&
            new_motion(1) == this->prev_motion(1) &&
            new_motion(2) == this->prev_motion(2) &&
            new_motion(3) == this->prev_motion(3)) {
//          if (new_motion(0) != 0 || new_motion(1) != 0 || new_motion(2) != 0 || new_motion(3) != 0)
//            break;
        } else {
          this->prev_motion(0) = new_motion(0);
          this->prev_motion(1) = new_motion(1);
          this->prev_motion(2) = new_motion(2);
          this->prev_motion(3) = new_motion(3);
        }
        sprintf(msg, "[%d %d %d %d]\n",
            (int)new_motion(0),
            (int)new_motion(1),
            (int)new_motion(2),
            (int)new_motion(3));
        serial_write(this->connections[i], msg);
        break;
      case 2:
        sprintf(msg, "[%d]\n",
          (int)new_motion(5));
        serial_write(this->connections[i], msg);
      default:
        break;
    }
  }
}

vec SlamBot::recv(void) {
  return zeros<vec>(5);
}

static double limitf(double x, double min, double max) {
  if (x < min) {
    return min;
  } else if (x > max) {
    return max;
  } else {
    return x;
  }
}
