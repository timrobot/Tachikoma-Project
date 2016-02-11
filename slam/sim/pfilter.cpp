#include "pfilter.h"
#include "sim_lidar.h"
#include "sim_robot.h"
#include <cmath>
#include <random>

using namespace arma;
using namespace std;

/** This is the constructor for the particle filter (think of it as your init)
 *  @param nparticles the number of particles to create
 *  @param map the pointer to the map (just store it)
 *  @param landmarks a list of landmarks, where each landmark is described in sim_landmark.h
 */
pfilter::pfilter(int nparticles, sim_map *map, vector<sim_landmark> &landmarks) {
  // STEP 1: store the map and landmark variables
  this->map = map;
  this->landmarks = landmarks;
  // STEP 2: create a bunch of particles, place them into this->particles
  int x=0; int y=0; double t=0;

  //sim_robot p = [nparticles];
  std::default_random_engine generator;
  std::uniform_int_distribution<int> distribution(0,(*map).n_cols);
  std::vector<sim_robot> new_particles;
  for (int i=0;i<nparticles;i++){
    x=distribution(generator);
    y=distribution(generator);
    t=std::rand()*(int)(2*M_PI) % (360);
    sim_robot newbot; 
    newbot.set_pose(x,y,t);
    newbot.set_noise(8.0, 0.5);
    new_particles.push_back(newbot);
  }
  particles.clear();
  particles = new_particles;

  // STEP 3: initialize the health as a vec of ones
  health = ones<vec>(nparticles);
}

pfilter::~pfilter(void) {
}

/** Set the noise for each robot
 *  @param vs the velocity sigma2
 *  @param ws the angular velocity sigma2
 */
void pfilter::set_noise(double vs, double ws) {
  // Set the noise for each robot
  for (sim_robot &bot : this->particles) {
    bot.set_noise(vs, ws);
  }
  this->vs = vs;
  this->ws = ws;
}

/** Set the size for each robot
 *  @param r the radius of the robot
 */
void pfilter::set_size(double r) {
  for (sim_robot &bot : this->particles) {
    bot.set_size(r);
  }
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
  double xp=0; double yp=0; double tp=0;
  for (int i=0;i<particles.size();i++){
    particles[i].move(v,w);
    // circular world!!!!
    sim_robot &bot = particles[i];
    while (bot.x < 0) {
      bot.x += (double)this->map->n_cols;
    }
    while (bot.x >= (double)this->map->n_cols) {
      bot.x -= (double)this->map->n_cols;
    }
    while (bot.y < 0) {
      bot.y += (double)this->map->n_rows;
    }
    while (bot.y >= (double)this->map->n_rows) {
      bot.y -= (double)this->map->n_rows;
    }
  }
}

double gauss(double mu, double sigma2) {
  return 1 / sqrt(2 * M_PI * sigma2) * exp(-0.5 * (mu * mu) / sigma2);
}

/** Weigh the "health" of each particle using gaussian error
 *  @param observations a 2xn matrix, where the first row is the x row,
 *                      and the second row is the y row
 *  @param health the health vector
 */
void pfilter::weigh(mat &observations) {
  vec theta = vec(observations.size());
  vec radius(observations.size());
  for (int i=0;i<particles.size();i++){
    for(int j=0;j<landmarks.size();j++){
      radius[j] = sqrt(pow(landmarks[j].x-particles[i].x, 2) + pow(landmarks[j].y-particles[i].y, 2));
      theta[j]=atan2(landmarks[j].y - particles[i].y, landmarks[j].x - particles[i].x) - particles[i].t;
      double newx = radius[j] * cos(theta[j]);
      double newy = radius[j] * sin(theta[j]);
      double distance =sqrt(pow(newx-observations(0,j),2) + pow(newy-observations(1,j),2)); 
      this->health[i] *= gauss(distance,1);
    }
  }
  resample();
}

/** Resample all the particles based on the health using the resample wheel
 *  @param health the health of all the particles
 */
void pfilter::resample(void) {
  int N = particles.size();
  vector<sim_robot> p2;
  int index = (int)std::rand() % N;
  double beta=0;
  double mw = (max(health));
  for (int i=0;i<N;i++){
    beta+=((double)std::rand()/(double)RAND_MAX)*2*mw;
    while (beta>=health[index]){
      if (beta == 0) {
        break;
      }
      beta-=health[index];
      index=(index+1)%N;
    }
    p2.push_back(particles[index]);
  }
  particles.clear();
  particles=p2;
}

/** Call the weigh and resample functions from here
 *  @param observations the observations of the landmarks
 */
void pfilter::observe(mat observations) {
  // each column of obs matches to each col of landmarks
  health=ones<vec>(particles.size());
  weigh(observations);
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
