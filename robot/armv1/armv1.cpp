#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <cmath>
#include <cassert>
#include <unistd.h>
#include <sys/types.h>
#include <dirent.h>
#include <termios.h>
#include <vector>
#include "armv1.h"
#include "defs.h"

#define WBUFSIZE  128
#define FPS 10

using namespace arma;
using namespace std;
using json = nlohmann::json;

static double limitf(double value, double minv, double maxv);
//static double cos_rule_angle(double A, double B, double C);
static double enc_transform(double mint, double maxt, double minv, double maxv, int reversed, double value);
static double secdiff(struct timeval &t1, struct timeval &t2);

Arm::Arm() : BaseRobot(ARMV1) {
  this->arm_read = zeros<vec>(DOF);
  this->arm_pos = zeros<mat>(1, 3);
  this->arm_mint = { -M_PI_2, -M_PI_2, -M_PI_4, -M_PI_2, -M_PI_2, 0.0 };
  this->arm_maxt = { M_PI_2, M_PI_2, M_PI_2, M_PI_2, M_PI_2, M_PI_2 };
  this->arm_minv = zeros<mat>(1, DOF);
  this->arm_maxv = zeros<mat>(1, DOF);
  this->arm_rev = zeros<umat>(1, DOF);
  this->calibration_loaded = false;
  this->devmgr = NULL;
  this->rlock = NULL;
  this->wlock = NULL;
  this->manager_running = false;
  this->buffered_arm_theta = zeros<mat>(1, DOF);
  this->buffered_arm_vel = zeros<mat>(1, DOF);
  this->buffered_arm_sensors = zeros<vec>(DOF);
  this->buffered_arm_theta_en = false;
  this->buffered_arm_vel_en = false;
  memset(&this->prevwtime, 0, sizeof(struct timeval));
  gettimeofday(&this->prevwtime, NULL);
}

Arm::~Arm() {
  if (this->connected()) {
    this->send(zeros<mat>(1, DOF), zeros<mat>(1, DOF));
    this->reset();
    this->disconnect();
  }
}

bool Arm::connect() {
  if (!this->connected()) {
    bool status = BaseRobot::connect();
    if (!this->connected() || !status) {
      this->disconnect();
    } else {
      this->reset();
      this->send(zeros<mat>(1, DOF), zeros<mat>(1, DOF));
      this->rlock = new mutex;
      this->wlock = new mutex;
      this->manager_running = true;
      this->devmgr = new thread(&Arm::update_task, this);
    }
    if (this->connected()) {
      printf("[ARM] Connected\n");
    } else {
      fprintf(stderr, "[ARM] Error! Not all devices might not have been connected\n");
    }
  }
  return this->connected();
}

bool Arm::connected() {
  return this->connections.size() > 0;
}

int Arm::numConnected() {
  return this->connections.size();
}

void Arm::disconnect() {
  if (this->manager_running) {
    this->manager_running = false;
    this->devmgr->join();
    delete this->devmgr;
    this->devmgr = NULL;
  }
  BaseRobot::disconnect();
  if (this->rlock) {
    delete this->rlock;
    this->rlock = NULL;
  }
  if (this->wlock) {
    delete this->wlock;
    this->wlock = NULL;
  }
  printf("[ARM] Disconnected\n");
}

void Arm::reset() {
  this->arm_read.zeros();
}

void Arm::send(
    const mat &arm_theta,
    const mat &arm_vel,
    bool arm_theta_en,
    bool arm_vel_en) {
  assert(arm_theta.n_rows == 1 && arm_theta.n_cols == DOF);
  assert(arm_vel.n_rows == 1 && arm_vel.n_cols == DOF);

  int devid;
  double arm[DOF];
  double omega[DOF];

  // safety checks
  for (uword i = 0; i < DOF; i++) {
    arm[i] = limitf(arm_theta(i), this->arm_mint(i), this->arm_maxt(i));
    if (this->calibrated()) {
      arm[i] = enc_transform(
          this->arm_mint(i), this->arm_maxt(i),
          this->arm_minv(i), this->arm_maxv(i),
          this->arm_rev(i), arm[i]);
    } else {
      arm_theta_en = false;
    }
    omega[i] = limitf(arm_vel(i), -1.0, 1.0);
  }
  char instr_activate = 0x80 |
    ((arm_theta_en && this->calibrated()) ? 0x01 : 0x00) |
    (arm_vel_en ? 0x02 : 0x00);

  char msg[WBUFSIZE];
  for (size_t i = 0; i < this->connections.size(); i++) {
    if (this->ids[i] > 0 && this->ids[i] <= 3) {
      switch ((devid = this->ids[i])) {

        // shoulder
        case SHOULDER:
          sprintf(msg, "[%d %d %d %d %d]\n",
              instr_activate,
              (int)(arm[SH_YAW]),
              (int)(arm[EL_PITCH]),
              (int)(omega[SH_YAW] * 255.0),
              (int)(omega[EL_PITCH] * 255.0));
          serial_write(this->connections[i], msg);
          break;

        // elbow
        case ELBOW:
          sprintf(msg, "[%d %d %d]\n",
              instr_activate,
              (int)(arm[SH_PITCH]),
              (int)(omega[SH_PITCH] * 255.0));
          serial_write(this->connections[i], msg);
          break;

        // wrist
        case WRIST:
          sprintf(msg, "[%d %d %d %d %d %d %d]\n",
              instr_activate,
              (int)(arm[WR_PITCH]),
              (int)(arm[WR_ROLL]),
              (int)(arm[HA_OPEN]),
              (int)(omega[WR_PITCH] * 255.0),
              (int)(omega[WR_ROLL] * 255.0),
              (int)(omega[HA_OPEN] * 255.0));
          serial_write(this->connections[i], msg);
          break;

        default:
          break;
      }
    }
  }
}

vec Arm::recv() {
  char *msg;
  int devid;

  // read from device
  for (size_t i = 0; i < this->connections.size(); i++) {
    if (this->ids[i] > 0 && this->ids[i] <= 3) {
      switch ((devid = this->ids[i])) {

        // shoulder
        case SHOULDER:
          if ((msg = serial_read(this->connections[i]))) {
            int sensor1;
            int sensor2;
            sscanf(msg, "[%d %d %d]\n", &this->ids[i],
                &sensor1, &sensor2);
            this->arm_read(SH_YAW) = sensor1;
            this->arm_read(EL_PITCH) = sensor2;
          }
          break;

        case ELBOW:
          if ((msg = serial_read(this->connections[i]))) {
            int sensor1;
            sscanf(msg, "[%d %d]\n", &this->ids[i],
                &sensor1);
            this->arm_read(SH_PITCH) = sensor1;
          }
          break;

        case WRIST:
          if ((msg = serial_read(this->connections[i]))) {
            int sensor1;
            int sensor2;
            int sensor3;
            sscanf(msg, "[%d %d %d %d]\n", &this->ids[i],
                &sensor1, &sensor2, &sensor3);
            this->arm_read(WR_PITCH) = sensor1;
            this->arm_read(WR_ROLL) = sensor2;
            this->arm_read(HA_OPEN) = sensor3;
          }
          break;

        default:
          break;
      }
    }
  }
  return this->arm_read;
}

void Arm::set_calibration_params(const string &filename) {
  FILE *fp;
  if (!(fp = fopen(filename.c_str(), "r"))) {
    return;
  }
  int beg = ftell(fp);
  fseek(fp, 0, SEEK_END);
  int end = ftell(fp);
  fseek(fp, 0, SEEK_SET);
  char *buf = new char[end - beg + 1];
  size_t bytesread = fread((void *)buf, sizeof(char), (size_t)(end - beg), fp);
  buf[bytesread] = '\0';
  this->set_calibration_params(json::parse(buf));
  delete buf;
}

bool Arm::calibrated() {
  return this->calibration_loaded;
}

void Arm::set_calibration_params(json cp) {
  vector<string> dofnames = {
    "shoulder_yaw",
    "shoulder_pitch",
    "elbow_pitch",
    "wrist_pitch",
    "wrist_roll",
    "hand_open"
  };
  vector<int> dofids = { SH_YAW, SH_PITCH, EL_PITCH, WR_PITCH, WR_ROLL, HA_OPEN };
  for (int i = 0; i < DOF; i++) {
    string name = dofnames[i];
    int id = dofids[i];
    this->arm_minv(id) = cp[name]["min"];
    this->arm_maxv(id) = cp[name]["max"];
    this->arm_rev(id) = cp[name]["reversed"] ? 1 : 0;
  }
  this->calibration_loaded = true;
}

void Arm::update_task() {
  while (this->manager_running) {
    this->update_send();
    this->update_recv();
  }
}

void Arm::update_send() {
  struct timeval currtime;
  gettimeofday(&currtime, NULL);
  double secs = secdiff(this->prevwtime, currtime);
  if (secs < 1.0 / (double)FPS) {
    return;
  } else {
    memcpy(&this->prevwtime, &currtime, sizeof(struct timeval));
  }
  this->wlock->lock();
  mat arm_theta = this->buffered_arm_theta;
  mat arm_vel = this->buffered_arm_vel;
  bool arm_theta_en = this->buffered_arm_theta_en;
  bool arm_vel_en = this->buffered_arm_vel_en;
  this->wlock->unlock();
  this->send(arm_theta, arm_vel, arm_theta_en, arm_vel_en);
}

void Arm::update_recv() {
  vec arm_sensors = this->recv();
  this->rlock->lock();
  this->buffered_arm_sensors = arm_sensors;
  this->rlock->unlock();
}

void Arm::move(
    const mat &arm_theta,
    const mat &arm_vel,
    bool arm_theta_en,
    bool arm_vel_en) {
  this->wlock->lock();
  this->buffered_arm_theta = arm_theta;
  this->buffered_arm_vel = arm_vel;
  this->buffered_arm_theta_en = arm_theta_en;
  this->buffered_arm_vel_en = arm_vel_en;
  this->wlock->unlock();
}

vec Arm::sense() {
  this->rlock->lock();
  vec arm_sensors = this->buffered_arm_sensors;
  this->rlock->unlock();
  return arm_sensors;
}

/** STATIC FUNCTIONS **/

static double limitf(double value, double min_value, double max_value) {
  if (value < min_value) {
    return min_value;
  } else if (value > max_value) {
    return max_value;
  } else {
    return value;
  }
}

//static double cos_rule_angle(double A, double B, double C) {
//  return acos((A * A + B * B - C * C) / (2.0 * A * B));
//}

static double enc_transform(double mint, double maxt, double minv, double maxv, int reversed, double value) {
  double enc_range = maxv - minv;
  value = limitf(value, mint, maxt);
  double ratio = enc_range / (maxt - mint);
  if (reversed) {
    value = -value;
  }
  return (value - mint) * ratio + minv;
}

static double secdiff(struct timeval &t1, struct timeval &t2) {
  double usec = (double)(t2.tv_usec - t1.tv_usec) / 1000000.0;
  double sec = (double)(t2.tv_sec - t1.tv_sec);
  return sec + usec;
}
