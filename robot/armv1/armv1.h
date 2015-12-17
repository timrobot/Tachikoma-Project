#include <armadillo>
#include <string>
#include <thread>
#include <mutex>
#include <njson/json.hpp>
#include <sys/time.h>
#include "baserobot.h"

class Arm : public BaseRobot {
  public:
    Arm();
    ~Arm();

    bool connect();
    bool connected();
    int numConnected();
    void disconnect();
    void send(
        const arma::mat &arm_theta,
        const arma::mat &arm_vel,
        bool arm_theta_en = false,
        bool arm_vel_en = true);
    arma::vec recv();
    void reset();

    void set_calibration_params(const std::string &filename);
    void set_calibration_params(nlohmann::json cp);
    bool calibrated();

    /* update on recv */
    arma::vec arm_read;
    arma::mat arm_pos;
    /* parameters */
    arma::mat arm_mint;
    arma::mat arm_maxt;
    arma::mat arm_minv;
    arma::mat arm_maxv;
    arma::umat arm_rev;

    /* threaded versions */
    void move(
        const arma::mat &arm_theta,
        const arma::mat &arm_vel,
        bool arm_theta_en = false,
        bool arm_vel_en = true);
    arma::vec sense();

  private:
    std::thread *devmgr;
    std::mutex *rlock;
    std::mutex *wlock;
    bool manager_running;

    void update_task();
    void update_send();
    void update_recv();

    arma::mat buffered_arm_theta;
    arma::mat buffered_arm_vel;
    bool buffered_arm_theta_en;
    bool buffered_arm_vel_en;
    arma::vec buffered_arm_sensors;
    struct timeval prevwtime;

    bool calibration_loaded;
};
