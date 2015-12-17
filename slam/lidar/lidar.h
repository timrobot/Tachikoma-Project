#ifndef __TK_LIDAR_H__
#define __TK_LIDAR_H__

#include <string>
#include <vector>
#include "rplidar.h"

using namespace rp::standalone::rplidar;

typedef struct polar {
  float radius;
  float theta;
} polar_t;

class Lidar {
  public:
    Lidar(void);
    ~Lidar(void);
    void update(void);
    std::vector<polar_t> grabPoints(void);
    std::vector<polar_t> readPoints(void);
    int status(void);
    bool connected(void);

    size_t count;

  private:
    std::string opt_com_path;
    RPlidarDriver *drv;
    rplidar_response_measurement_node_t nodes[720];
    std::vector<polar_t> lidar_data;

    bool checkRPLIDARHealth(void);
};

#endif
