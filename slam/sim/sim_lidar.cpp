#include "sim_lidar.h"
#include "sdldef.h"
#include "draw.h"
#include <cmath>

using namespace arma;
using namespace std;

static bool within(int x, int a, int b) {
  return a <= x && x <= b;
}

sim_lidar::sim_lidar(sim_robot *robot) {
  this->robot = robot;
  this->robot->attach_lidar((void *)this);
  this->rs = 0;
  this->ts = 0;
}

mat sim_lidar::grab_points(void) {
  // send out a raycast into the environment
  const int npts = 60;
  mat pts(2, npts);
  for (int t = 0; t < npts; t++) {
    double theta = t * 2 * M_PI / npts;
    double angle = theta + this->robot->t;
    // add all points for the simulation
    int maxr = (int)this->robot->map->n_rows + (int)this->robot->map->n_cols;
    double radius;
    for (int r = 0; r < maxr; r++) {
      // find original x and y
      int x = (int)round(this->robot->x + r * cos(angle));
      int y = (int)round(this->robot->y + r * sin(angle));
      // detect if hyp is out of bounds
      if (!within(x, 0, this->robot->map->n_cols-1) ||
          !within(y, 0, this->robot->map->n_rows-1)) { // skip it if too far away anyway
        radius = 10000; // something really large
        break;
      }
      ivec xpos({ x-1, x, x, x, x+1 });
      ivec ypos({ y, y, y+1, y-1, y });
      int total = 0;
      int nelem = 0;
      for (int i = 0; i < 5; i++) {
        if (!within(xpos(i), 0, this->robot->map->n_cols-1) ||
            !within(ypos(i), 0, this->robot->map->n_rows-1)) {
          continue;
        }
        total += this->robot->map->map(ypos[i], xpos[i]);
        nelem++;
      }
      if (nelem == 0) {
        radius = 10000; // something really large
        break;
      }
      if (total > 0) { // found the radius
        radius = (double)r;
        break;
      }
    }
    pts(0, t) = radius;
    pts(1, t) = theta;
  }
  return pts;
}

static double erfinv(double p) {
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
  double p = (double)rand() / ((double)RAND_MAX / 2) - 1;
  return erfinv(p) * sigma;
}

void sim_lidar::set_noise(double rs, double ts) {
  this->rs = rs;
  this->ts = ts;
}

mat sim_lidar::sense(void) {
  mat pts = this->grab_points();
  for (uword j = 0; j < pts.n_cols; j++) {
    pts(0, j) += gaussianNoise(this->rs);
    pts(1, j) += gaussianNoise(this->ts);
  }
  return pts;
}

void sim_lidar::blit(icube &screen) {
  mat pts = this->grab_points();
  mat rayframe(screen.n_rows, screen.n_cols, fill::zeros);
  for (uword j = 0; j < pts.n_cols; j++) {
    double angle = pts(1, j) + this->robot->t;
    double r = pts(0, j);
    double x = this->robot->x + r * cos(angle);
    double y = this->robot->y + r * sin(angle);
    // draw line
    draw_line(rayframe, 1.0, { this->robot->y, this->robot->x }, { y, x });
  }
  for (int i = 0; i < (int)rayframe.n_rows; i++) {
    for (int j = 0; j < (int)rayframe.n_cols; j++) {
      if (rayframe(i,j) > 0.5) {
        screen(i, j, 0) = 255;
        screen(i, j, 1) = 0;
        screen(i, j, 2) = 0;
      }
    }
  }
}
