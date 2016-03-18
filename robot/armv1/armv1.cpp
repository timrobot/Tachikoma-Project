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

static mat genRotateMat(double x, double y, double z);
static double limitf(double value, double minv, double maxv);
static double cos_rule_angle(double A, double B, double C);
static double map_domain(double value, vec from, vec to);
static double secdiff(struct timeval &t1, struct timeval &t2);

Arm::Arm(void) : BaseRobot(ARMV1) {
  this->arm_read = zeros<vec>(DOF);
  this->arm_fback = zeros<vec>(DOF);
  this->arm_current = zeros<vec>(2); // change this if necessary
  this->arm_pos = zeros<vec>(3);
  this->arm_mint = zeros<vec>(DOF);
  this->arm_maxt = zeros<vec>(DOF);
  this->arm_minv = zeros<vec>(DOF);
  this->arm_maxv = zeros<vec>(DOF);
  this->arm_link_length = { 0.2, 5.0, 10.5, 6.5, 4.5, 3.8, 3.75 };
  this->calibration_loaded = false;
  this->devmgr = NULL;
  this->rlock = NULL;
  this->wlock = NULL;
  this->manager_running = false;
  this->buffered_arm_theta = zeros<vec>(DOF);
  this->buffered_arm_vel = zeros<vec>(DOF);
  this->buffered_arm_sensors = zeros<vec>(DOF);
  this->buffered_arm_theta_en = false;
  this->buffered_arm_vel_en = false;
  memset(&this->prevwtime, 0, sizeof(struct timeval));
  gettimeofday(&this->prevwtime, NULL);
}

Arm::~Arm(void) {
  if (this->connected()) {
    this->send(zeros<vec>(DOF), zeros<vec>(DOF), false, false);
    this->reset();
    this->disconnect();
  }
}

bool Arm::connect(void) {
  if (!this->connected()) {
    bool status = BaseRobot::connect();
    if (!this->connected() || !status) {
      this->disconnect();
    } else {
      this->reset();
      this->send(zeros<vec>(DOF), zeros<vec>(DOF));
      this->rlock = new mutex;
      this->wlock = new mutex;
      this->manager_running = true;
      this->devmgr = new thread(&Arm::update_uctrl, this);
    }
    if (this->connected()) {
      printf("[ARM] Connected\n");
    } else {
      fprintf(stderr, "[ARM] Error! Not all devices might not have been connected\n");
    }
  }
  return this->connected();
}

bool Arm::connected(void) {
  return this->connections.size() > 0;
}

int Arm::numConnected(void) {
  return this->connections.size();
}

void Arm::disconnect(void) {
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

void Arm::reset(void) {
  this->arm_read.zeros();
  this->arm_fback.zeros();
}

void Arm::send(
    const vec &arm_theta,
    const vec &arm_vel,
    bool arm_theta_act,
    bool arm_vel_act) {
  assert(arm_theta.n_elem == DOF);
  assert(arm_vel.n_elem == DOF);

  int devid;
  double arm[DOF];
  double omega[DOF];

  // safety checks
  for (uword i = 0; i < DOF; i++) {
    arm[i] = limitf(arm_theta(i), this->arm_mint(i), this->arm_maxt(i));
    if (this->calibrated()) {
      arm[i] = map_domain(arm[i],
          vec({ this->arm_mint(i), this->arm_maxt(i) }),
          vec({ this->arm_minv(i), this->arm_maxv(i) }));
    } else {
      arm_theta_act = false;
    }
    omega[i] = limitf(arm_vel(i), -1.0, 1.0);
  }

  char instr_activate = 0x80 | ((arm_theta_act && this->calibrated()) ? 0x01 : 0x00) |
    (arm_vel_act ? 0x02 : 0x00);

  char msg[WBUFSIZE];
  for (size_t i = 0; i < this->connections.size(); i++) {
    if (this->ids[i] > 0 && this->ids[i] <= 1) {
      switch ((devid = this->ids[i])) {

        case UPPER_ARM:
          sprintf(msg, "[%d %d %d %d %d %d %d %d %d]\n",
              instr_activate,
              (int)(arm[JOINT2]),
              (int)(arm[JOINT3]),
              (int)(arm[JOINT4]),
              (int)(arm[JOINT5]),
              (int)(omega[JOINT2]),
              (int)(omega[JOINT3]),
              (int)(omega[JOINT4]),
              (int)(omega[JOINT5]));
          serial_write(this->connections[i], msg);
          break;

        case LOWER_ARM:
          sprintf(msg, "[%d %d %d %d %d]\n",
              instr_activate,
              (int)(arm[JOINT0]),
              (int)(arm[JOINT1]),
              (int)(omega[JOINT0]),
              (int)(omega[JOINT1]));
          serial_write(this->connections[i], msg);
          break;

        default:
          break;

      }
    }
  }
}

vec Arm::recv(void) {
  char *msg;
  int devid;
  int storage[8];

  // read from device
  for (size_t i = 0; i < this->connections.size(); i++) {
    if (this->ids[i] > 0 && this->ids[i] <= 1) {
      switch ((devid = this->ids[i])) {

        case UPPER_ARM:
          if ((msg = serial_read(this->connections[i]))) {
            sscanf(msg, "[%d %d %d %d %d %d %d %d %d]\n", &this->ids[i],
                &storage[0],
                &storage[1],
                &storage[2],
                &storage[3],
                &storage[4],
                &storage[5],
                &storage[6],
                &storage[7]);
            this->arm_read(JOINT2) = storage[0];
            this->arm_read(JOINT3) = storage[1];
            this->arm_read(JOINT4) = storage[2];
            this->arm_read(JOINT5) = storage[3];
            this->arm_fback(JOINT2) = storage[4];
            this->arm_fback(JOINT3) = storage[5];
            this->arm_fback(JOINT4) = storage[6];
            this->arm_fback(JOINT5) = storage[7];
          }
          break;

        case LOWER_ARM:
          if ((msg = serial_read(this->connections[i]))) {
            sscanf(msg, "[%d %d %d %d %d %d %d]\n", &this->ids[i],
                &storage[0],
                &storage[1],
                &storage[2],
                &storage[3],
                &storage[4],
                &storage[5]);
            this->arm_read(JOINT0) = storage[0];
            this->arm_read(JOINT1) = storage[1];
            this->arm_current(0) = storage[2];
            this->arm_current(1) = storage[3];
            this->arm_fback(JOINT0) = storage[4];
            this->arm_fback(JOINT1) = storage[5];
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

bool Arm::calibrated(void) {
  return this->calibration_loaded;
}

void Arm::set_calibration_params(json cp) {
  vector<string> dofnames = {
    "joint0",
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5"
  };
  for (int i = 0; i < DOF; i++) {
    string name = dofnames[i];
    this->arm_minv(i) = cp[name]["raw_min"];
    this->arm_maxv(i) = cp[name]["raw_max"];
    this->arm_mint(i) = cp[name]["theta_min"];
    this->arm_maxt(i) = cp[name]["theta_max"];
  }
  this->calibration_loaded = true;
}

/** THREADED STUFF **/

void Arm::update_uctrl(void) {
  while (this->manager_running) {
    this->update_send();
    this->update_recv();
  }
}

void Arm::update_send(void) {
  struct timeval currtime;
  gettimeofday(&currtime, NULL);
  double secs = secdiff(this->prevwtime, currtime);
  if (secs < 1.0 / (double)FPS) {
    return;
  } else {
    memcpy(&this->prevwtime, &currtime, sizeof(struct timeval));
  }
  this->wlock->lock();
  vec arm_theta = this->buffered_arm_theta;
  vec arm_vel = this->buffered_arm_vel;
  bool arm_theta_en = this->buffered_arm_theta_en;
  bool arm_vel_en = this->buffered_arm_vel_en;
  this->wlock->unlock();
  this->send(arm_theta, arm_vel, arm_theta_en, arm_vel_en);
}

void Arm::update_recv(void) {
  vec arm_sensors = this->recv();
  this->rlock->lock();
  this->buffered_arm_sensors = arm_sensors;
  this->rlock->unlock();
}

void Arm::move(
    const vec &arm_theta,
    const vec &arm_vel,
    bool arm_theta_act,
    bool arm_vel_act) {
  this->wlock->lock();
  this->buffered_arm_theta = arm_theta;
  this->buffered_arm_vel = arm_vel;
  this->buffered_arm_theta_en = arm_theta_act;
  this->buffered_arm_vel_en = arm_vel_act;
  this->wlock->unlock();
}

vec Arm::sense(void) {
  this->rlock->lock();
  vec arm_sensors = this->buffered_arm_sensors;
  this->rlock->unlock();
  return arm_sensors;
}

void Arm::set_pose(
    double joint0,
    double joint1,
    double joint2,
    double joint3,
    double joint4,
    double joint5,
    bool en) {
  this->wlock->lock();
  this->buffered_arm_theta = { joint0, joint1, joint2, joint3, joint4, joint5 };
  this->buffered_arm_vel = zeros<vec>(DOF);
  this->buffered_arm_theta_en = en;
  this->buffered_arm_vel_en = false;
  this->wlock->unlock();
}

/** KINEMATICS STUFF **/

static mat genRotateMat(double x, double y, double z) {
  mat X = reshape(mat({
        1, 0, 0,
        0, cos(x), -sin(x),
        0, sin(x), cos(x)
        }), 3, 3).t();
  mat Y = reshape(mat({
        cos(y), 0, sin(y),
        0, 1, 0,
        -sin(y), 0, cos(y)
        }), 3, 3).t();
  mat Z = reshape(mat({
        cos(z), -sin(z), 0,
        sin(z), cos(z), 0,
        0, 0, 1
        }), 3, 3).t();
  return Z * Y * X;
}

vec Arm::get_end_effector_pos(int linkid) {
  // solve arm (using D-H notation and forward kinematics)

  // get the radians
  vec rad(DOF);
  for (int i = 0; i < DOF; i++) {
    rad(i) = map_domain( (double)this->arm_read(i),
        vec({ this->arm_minv(i), this->arm_maxv(i) }),
        vec({ this->arm_mint(i), this->arm_maxt(i) }));
  }

  // get the rotations
  vector<mat> rotate = {
    genRotateMat(0, 0, rad(0)),
    genRotateMat(-rad(1), 0, 0),
    genRotateMat(-rad(2), 0, 0),
    genRotateMat(-rad(3), 0, 0),
    genRotateMat(0, 0, rad(4)),
    genRotateMat(0, 0, 0)
  };

  // get the translations
  vector<vec> translate = {
    { 0, 0, this->arm_link_length(0) },
    { 0, 0, this->arm_link_length(1) },
    { 0, 0, this->arm_link_length(2) },
    { 0, 0, this->arm_link_length(3) },
    { 0, 0, this->arm_link_length(4) },
    { 0, 0, this->arm_link_length(5) },
    { 0, 0, this->arm_link_length(6) }
  };

  // get the position using the combination of rotations
  // and the translations up to the linkid [0|1|2|3|4|5|6]
  vec pos = translate[linkid];
  for (int i = linkid - 1; i >= 0; i--) {
    pos = rotate[i] * pos + translate[i];
  }

  return pos;
}

bool Arm::get_position_placement(vec target_pos, vec target_pose, double target_spin, vec &solution_enc) {
  solution_enc = vec(DOF, fill::zeros);
  vec solution_rad(DOF);

  // solve first for the direction of the base
  double r = sqrt(dot(target_pos(span(0, 1)), target_pos(span(0, 1))));
  // solve for the height next
  double h = target_pos(2) - sum(this->arm_link_length(span(0, 1)));
  vec interpos({ r, 0, h });

  // grab the target pose and the distance away necessary to make such a pose
  target_pose /= sqrt(dot(target_pose, target_pose));
  target_pose *= sum(this->arm_link_length(span(4, 6)));
  interpos -= target_pose;

  // find the length
  double l = sqrt(dot(interpos, interpos));
  if (l > sum(this->arm_link_length(span(2, 3)))) {
    return false;
  }

  // determine some offset angle
  double phi = atan2(h, r);
  
  // calculate the triangle edges
  solution_rad(JOINT0) = atan2(target_pos(1), target_pos(0));
  solution_rad(JOINT1) = cos_rule_angle(this->arm_link_length(2), l, this->arm_link_length(3)) + phi;

  // check to make sure that the second joint with the third link does not hit the backplane
  double basewidth = 6.0;
  double baselength = 4.0;
  double baseheight = this->arm_link_length(3);
  vec side({ 0, 0, baseheight });
  side = genRotateMat(-solution_rad(JOINT1), 0, 0) * side;
  vec leftside = genRotateMat(0, 0, solution_rad(JOINT0)) * (side + vec({ -basewidth, -baselength, 0 }));
  vec rightside = genRotateMat(0, 0, solution_rad(JOINT0)) * (side + vec({ basewidth, -baselength, 0 }));
  if (leftside(1) < -8.0 || rightside(1) < -8.0) {
    return false;
  }

  solution_rad(JOINT2) = cos_rule_angle(this->arm_link_length(2), this->arm_link_length(3), l);

  // calculate the angle of the next joint from the offset angle
  solution_rad(JOINT3) = -phi;
  
  // leave grabbing up to the programmer
  solution_rad(JOINT4) = target_spin;
  solution_rad(JOINT5) = M_PI_4;

  for (int i = 0; i < 6; i++) {
    solution_enc(i) = map_domain(solution_rad(i),
        vec({ this->arm_mint(i), this->arm_maxt(i) }),
        vec({ this->arm_minv(i), this->arm_maxv(i) }));
  }
  return true;
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

static double cos_rule_angle(double A, double B, double C) {
  return acos((A * A + B * B - C * C) / (2.0 * A * B));
}

static double map_domain(double value, vec from, vec to) {
  assert(from.n_elem == 2 && to.n_elem == 2);
  value = limitf(value, from(1), from(0));
  double ratio = (to(1) - to(0)) / (from(1) - from(0));
  return (value - from(0)) * ratio + to(0);
}

static double secdiff(struct timeval &t1, struct timeval &t2) {
  double usec = (double)(t2.tv_usec - t1.tv_usec) / 1000000.0;
  double sec = (double)(t2.tv_sec - t1.tv_sec);
  return sec + usec;
}
