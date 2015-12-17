#ifndef particlefilter_h
#define particlefilter_h

#include <sys/time.h>
#include <vector>
#include <armadillo>
#include "gridmap.h"

class Particle {
  public:
    arma::vec pose;
    arma::vec sigma;
    double health;

    Particle(double px, double py, double ang, double hp);
    ~Particle(void);
};

class Robot {
  public:
    arma::vec pose;
    double radius;
    
    Robot(double x, double y, double ang, double radius);
    ~Robot(void);
};

class ParticleFilter {
  public:
    ParticleFilter(GridMap &map, int nparticles = 1000);
    ~ParticleFilter(void);
    // RunParticleFilter
    void update(const arma::vec &motion, const std::vector<arma::vec> &obs);
    // PredictedLocalization
    arma::vec predict(double &sigma);

  private:
    //int particle_count;
    GridMap &world;
    
    std::vector<Particle> particles;
//    std::vector<arma::vec> latestobs;
//    struct timeval obstimestamp;
    
    double weight(const Particle &p);
    void resample(void);
};

#endif
