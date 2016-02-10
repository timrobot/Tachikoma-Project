#ifndef sim_lidar_h
#define sim_lidar_h

#include <armadillo>
#include <SDL2/SDL.h>
#include "sim_map.h"
#include "sim_robot.h"

class sim_lidar {
  public:
    sim_lidar(sim_robot *robot);
    arma::mat sense(void);
    void blit(arma::icube &screen);
    void set_noise(double rs, double ts);
    sim_robot *robot;
    double rs;
    double ts;

  private:
    arma::mat grab_points(void);
};

#endif
