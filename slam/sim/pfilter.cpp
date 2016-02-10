#include "pfilter.h"
#include "sim_lidar.h"
#include <cmath>

using namespace arma;
using namespace std;

/** This is the constructor for the particle filter (think of it as your init)
 *  @param nparticles the number of particles to create
 *  @param map the pointer to the map (just store it)
 *  @param landmarks a list of landmarks, where each landmark is described in sim_landmark.h
 */
pfilter::pfilter(int nparticles, sim_map *map, vector<sim_landmark> &landmarks) {
  // TODO
  // STEP 1: store the map and landmark variables
  // STEP 2: create a bunch of particles, place them into this->particles
  // hint: use .set_pose from the sim_robot class to initialize their poses
  // STEP 3: initialize the health as a vec of ones
}

pfilter::~pfilter(void) {
}

/** Set the noise for each robot
 *  @param vs the velocity sigma2
 *  @param ws the angular velocity sigma2
 */
void pfilter::set_noise(double vs, double ws) {
  // TODO
  // Set the noise for each robot
}

/** Set the size for each robot
 *  @param r the radius of the robot
 */
void pfilter::set_size(double r) {
  // TODO
  // Set the size for each robot
}

static double erfinv(double p) {
  // DO NOT CHANGE THIS FUNCTION
  // approximate maclaurin series (refer to http://mathworld.wolfram.com/InverseErf.html)
  double sqrt_pi = sqrt(M_PI);
  vec a = {
    0.88623,
    0.23201,
    0.12756,
    0.086552
  };
  vec x(a.n_elem);
  for (int i = 0; i < x.n_elem; i++) {
    x(i) = pow(p, 2 * i + 1);
  }
  return dot(a,x);
}

static double gaussianNoise(double sigma) {
  // Utility function: can be use to simulate gaussian noise
  double p = (double)rand() / ((double)RAND_MAX / 2) - 1;
  return erfinv(p) * sigma;
}

/** Move each robot by some velocity and angular velocity
 *  In here, also detect if the robot's position goes out of range,
 *  and handle it
 *  Pretend that the world is circular
 *  @param v the velocity
 *  @param w the angular velocity
 */
void pfilter::move(double v, double w) {
  // TODO
}

/** Weigh the "health" of each particle using gaussian error
 *  @param observations a 2xn matrix, where the first row is the x row,
 *                      and the second row is the y row
 *  @param health the health vector
 */
void pfilter::weigh(mat &observations, vec &health) {
  // TODO
}

/** Resample all the particles based on the health using the resample wheel
 *  @param health the health of all the particles
 */
void pfilter::resample(vec health) {
  //TODO
}

/** Call the weigh and resample functions from here
 *  @param observations the observations of the landmarks
 */
void pfilter::observe(mat observations) {
  // TODO
}

/** Predict the position and calculate the error of the particle set
 *  @param mu (output) the position ( x, y, theta )
 *  @param sigma (output) the error
 */
void pfilter::predict(vec &mu, double &sigma) {
  // TODO
  mu = vec({ -1, -1, 0 });
  sigma = 0;
}

void pfilter::blit(icube &screen) {
  // DO NOT TOUCH THIS FUNCTION
  vec health;
  if (this->health.n_elem != 0) {
    this->health / max(this->health);
  } else {
    this->health = 0.00001;
  }
  int i = 0;
  for (sim_robot &bot : this->particles) {
    int x = (int)round(bot.x);
    int y = (int)round(bot.y);
    if (x >= 0 && x < (int)screen.n_cols &&
        y >= 0 && y < (int)screen.n_rows) {
      screen(y, x, 0) = 0;
      screen(y, x, 1) = 255;
      screen(y, x, 2) = 0;
    }
    i++;
  }
}
