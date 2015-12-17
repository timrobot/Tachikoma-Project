/****************************************
 *
 * The purpose of this program is to do 
 * the following for this particular bot:      
 *
 *  1) control the robot through
 *     abstracted methods
 *  2) send back sensor map values
 *
 *  TODO: look into why the some of the
 *  conversion functions dont work
 *
 ***************************************/

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
#include "tachikoma.h"

#define WBUFSIZE  128
#define NCOMCNTR  4
#define FPS 10

using namespace arma;
using namespace std;
using json = nlohmann::json;

static double limitf(double value, double min_value, double max_value);
//static double cos_rule_angle(double A, double B, double C);
static double enc_transform(int jointid, double minv, double maxv, int reversed, double value);
static double secdiff(struct timeval &t1, struct timeval &t2);

/** CLASS FUNCTIONS **/

Tachikoma::Tachikoma(void) : BaseRobot(TACHIKOMA) {
  this->leg_write = zeros<mat>(NUM_LEGS, NUM_JOINTS * 2 + 1);
  this->leg_read = zeros<mat>(NUM_LEGS, NUM_JOINTS + 1);
  this->leg_fback = zeros<mat>(NUM_LEGS, NUM_JOINTS * 2 + 1);
  this->leg_positions = zeros<mat>(NUM_LEGS, 3);
  this->leg_min = zeros<mat>(NUM_LEGS, NUM_JOINTS);
  this->leg_max = zeros<mat>(NUM_LEGS, NUM_JOINTS);
  this->leg_rev = zeros<umat>(NUM_LEGS, NUM_JOINTS);
  this->uctrl_manager = NULL;
  this->read_lock = NULL;
  this->write_lock = NULL;
  this->manager_running = false;
  this->buffered_leg_theta = zeros<mat>(NUM_LEGS, NUM_JOINTS);
  this->buffered_leg_vel = zeros<mat>(NUM_LEGS, NUM_JOINTS);
  this->buffered_wheels = zeros<vec>(NUM_LEGS);
  this->buffered_arm_theta = zeros<mat>(1, 1);
  this->buffered_leg_theta_act = false;
  this->buffered_leg_vel_act = false;
  this->buffered_leg_sensors = zeros<mat>(NUM_LEGS, NUM_JOINTS + 1);
  this->buffered_leg_feedback = zeros<mat>(NUM_LEGS, NUM_JOINTS * 2 + 1);
  this->calibration_loaded = false;
  this->instruction_activate = 0;
  memset(&this->prevwritetime, 0, sizeof(struct timeval));
  gettimeofday(&this->prevwritetime, NULL);
}

Tachikoma::~Tachikoma(void) {
  if (this->connected()) {
    this->send(
        zeros<mat>(NUM_LEGS, NUM_JOINTS),
        zeros<mat>(NUM_LEGS, NUM_JOINTS),
        zeros<vec>(NUM_LEGS),
        zeros<mat>(1, 1), false, false); // change arm specification
    this->reset();
    this->disconnect();
    printf("[TACHIKOMA] Disconnected.\n");
  }
}

bool Tachikoma::connect(void) {
  if (!this->connected()) {
    bool status = BaseRobot::connect();
    if (!this->connected() || !status) {
      // if the device has failed either parent or derived checks, disconnect
      this->disconnect();
    }
    this->reset();
    this->send(
        zeros<mat>(NUM_LEGS, NUM_JOINTS),
        zeros<mat>(NUM_LEGS, NUM_JOINTS),
        zeros<vec>(NUM_LEGS),
        zeros<mat>(1, 1), false, false); // change arm specification

    // create locks for the data
    this->read_lock = new mutex;
    this->write_lock = new mutex;
    // start a runnable thread to query devices
    this->manager_running = true;
    this->uctrl_manager = new thread(&Tachikoma::update_uctrl, this);
  }
  return this->connected();
}

bool Tachikoma::connected(void) {
  return this->connections.size() > 0;
}

int Tachikoma::numconnected(void) {
  return this->connections.size();
}

void Tachikoma::disconnect(void) {
  if (this->manager_running) {
    // signal the manager to stop updating
    this->manager_running = false;
    // wait until the device manager can join back to the parent thread
    this->uctrl_manager->join();
    delete this->uctrl_manager;
    this->uctrl_manager = NULL;
  }
  BaseRobot::disconnect();
  if (this->read_lock) {
    delete this->read_lock;
    this->read_lock = NULL;
  }
  if (this->write_lock) {
    delete this->write_lock;
    this->write_lock = NULL;
  }
}

void Tachikoma::reset(void) {
  this->leg_write.zeros();
  this->leg_read.zeros();
  this->leg_fback.zeros();
}

void Tachikoma::send(
    const mat &leg_theta,
    const mat &leg_vel,
    const vec &wheels,
    const mat &arm_theta,
    bool leg_theta_act,
    bool leg_vel_act) {
  assert(leg_theta.n_rows == NUM_LEGS && leg_theta.n_cols == NUM_JOINTS);
  assert(leg_vel.n_rows == NUM_LEGS && leg_vel.n_cols == NUM_JOINTS);
  assert(wheels.n_elem == NUM_LEGS);

  int devid;
  int legid1;
  int legid2;
  double leg[NUM_LEGS][NUM_JOINTS * 2 + 1]; // njoints x (position, velocity) and wheel

  // set up the leg matrix (safety checks)
  for (uword i = 0; i < NUM_LEGS; i++) {
    for (uword j = 0; j < NUM_JOINTS; j++) {
      leg[i][j + WAIST_POS] = limitf(leg_theta(i, j), -M_PI, M_PI);
      if (leg[i][j + WAIST_POS] == -M_PI) { // special case
        leg[i][j + WAIST_POS] = M_PI;
      }
      // use calibration definitions if they exist
      if (this->calibrated()) {
        leg[i][j + WAIST_POS] = enc_transform(j,
            this->leg_min(i, j), this->leg_max(i, j),
            this->leg_rev(i, j), leg[i][j + WAIST_POS]);
      }
      // velocity index hack (i + WAIST_VEL)
      leg[i][j + WAIST_VEL] = limitf(leg_vel(i, j), -1.0, 1.0);
    }
    leg[i][WHEEL_VEL] = limitf(wheels(i), -1.0, 1.0);
  }

  // instruction char representing action to undertake (global)
  char instr_activate = 0x80 | ((leg_theta_act && this->calibrated()) ? 0x01 : 0x00) |
    (leg_vel_act ? 0x02 : 0x00);

  // write to device (only for legs for now)
  char msg[WBUFSIZE];
  for (size_t i = 0; i < this->connections.size(); i++) {
    if (this->ids[i] > 0 && this->ids[i] <= NUM_DEV) {
      switch ((devid = this->ids[i])) {

        // waist
        case WAIST_UP:
        case WAIST_DOWN:
          switch (devid) {
            case WAIST_UP:
              legid1 = UL;
              legid2 = UR;
              break;
            case WAIST_DOWN:
              legid1 = DL;
              legid2 = DR;
              break;
          }
          sprintf(msg, "[%d %d %d %d %d]\n",
              instr_activate,
              (int)(leg[legid1][WAIST_POS]),
              (int)(leg[legid2][WAIST_POS]),
              (int)(leg[legid1][WAIST_VEL] * 255.0),
              (int)(leg[legid2][WAIST_VEL] * 255.0));
          serial_write(this->connections[i], msg);
          break;

        // thigh
        case THIGH_UL:
        case THIGH_UR:
        case THIGH_DL:
        case THIGH_DR:
          legid1 = devid - THIGH_UL;
          sprintf(msg, "[%d %d %d]\n",
              instr_activate,
              (int)(leg[legid1][THIGH_POS]),
              (int)(leg[legid1][THIGH_VEL] * 255.0));
          serial_write(this->connections[i], msg);
          break;

        // wheel
        case WHEEL_UL:
        case WHEEL_UR:
        case WHEEL_DL:
        case WHEEL_DR:
          // speed hack (will change with definition changes)
          legid1 = devid - WHEEL_UL;
          sprintf(msg, "[%d]\n",
              (int)(leg[legid1][WHEEL_VEL] * 255.0));
          serial_write(this->connections[i], msg);
          break;
        default:
          break;
      }
    }
  }

  // after sending all the new components, update current state of robot
  this->instruction_activate = instr_activate;
}

vec Tachikoma::recv(
    mat &leg_sensors,
    mat &leg_feedback) {
  char *msg;
  int devid;
  int legid1;
  int legid2;
  int enc;
  int sensor1;
  int sensor2;
  int dummy[2];
  // read from device
  for (int i = 0; i < (int)this->connections.size(); i++) {
    if (this->ids[i] > 0 && this->ids[i] <= NUM_DEV) {
      switch ((devid = this->ids[i])) {

        // waist
        case WAIST_UP:
        case WAIST_DOWN:
          if ((msg = serial_read(this->connections[i]))) {
            switch (devid) {
              case WAIST_UP:
                legid1 = UL;
                legid2 = UR;
                break;
              case WAIST_DOWN:
                legid1 = DL;
                legid2 = DR;
                break;
              default:
                legid1 = 0;
                legid2 = 0;
                break;
            }
            sscanf(msg, "[%d %d %d %d %d]\n", &this->ids[i],
                &sensor1, &sensor2, &dummy[0], &dummy[1]);
            this->leg_read(legid1, WAIST_POS) = sensor1;
            this->leg_read(legid2, WAIST_POS) = sensor2;
            this->leg_fback(legid1, WAIST_POS) = dummy[0];
            this->leg_fback(legid2, WAIST_POS) = dummy[1];
          }
          break;

        // thigh
        case THIGH_UL:
        case THIGH_UR:
        case THIGH_DL:
        case THIGH_DR:
          legid1 = devid - THIGH_UL; // hack for speed
          if ((msg = serial_read(this->connections[i]))) {
            sscanf(msg, "[%d %d %d]\n", &this->ids[i],
                &sensor1, &dummy[0]);
            this->leg_read(legid1, THIGH_POS) = sensor1;
            this->leg_fback(legid1, THIGH_POS) = dummy[0];
          }
          break;

        // wheel
        case WHEEL_UL:
        case WHEEL_UR:
        case WHEEL_DL:
        case WHEEL_DR:
          if ((msg = serial_read(this->connections[i]))) {
            legid1 = devid - WHEEL_UL; // hack for speed
            sscanf(msg, "[%d %d %d]\n", &this->ids[i],
                &enc, &dummy[0]);
            this->leg_read(legid1, WHEEL_VEL) = (double)enc;
            this->leg_fback(legid1, WHEEL_VEL) = dummy[0];
          }
          break;
        default:
          break;
      }
    }
  }
  leg_sensors = this->leg_read;
  leg_feedback = this->leg_fback;
  return vectorise(this->leg_read);
}

bool Tachikoma::set_calibration_params(const string &filename) {
  string params;
  ifstream params_file(filename);
  string temp;
  while (getline(params_file, temp)) {
    params += temp;
  }
  params_file.close();
  this->set_calibration_params(json::parse(buf));
  delete buf;
  return true;
}

void Tachikoma::set_calibration_params(json cp) {
  vector<string> legnames = { "ul", "ur", "dl", "dr" };
  vector<int> legids = { UL, UR, DL, DR };
  vector<string> jointnames = { "waist", "thigh" };
  vector<int> jointids = { WAIST, THIGH };
  for (int i = 0; i < NUM_LEGS; i++) {
    for (int j = 0; j < NUM_JOINTS; j++) {
      string legname = legnames[i];
      int legid = legids[i];
      string jointname = jointnames[j];
      int jointid = jointids[j];
      this->leg_min(legid, jointid) = cp[legname][jointname]["min"];
      this->leg_max(legid, jointid) = cp[legname][jointname]["max"];
      this->leg_rev(legid, jointid) = cp[legname][jointname]["reversed"] ? 1 : 0;
    }
  }
  this->calibration_loaded = true;
}

// Note: the following is unnecessary for now

/*vec Tachikoma::leg_fk_solve(const vec &enc, int legid) {
  double cosv;
  double sinv;

// solve leg (using D-H notation)
// set up reference frame 3
double x = knee_length;
double y = 0.0;
double z = 0.0;

double waist = enc(WAIST_POS);
double thigh = enc(THIGH_POS);
double knee  = enc(KNEE_POS);

// solve for the transformation in refrence frame 2
cosv = cos(knee);
sinv = sin(knee);
x = cosv * x + sinv * z + thigh_length;
z = -sinv * x + cosv * z;

// solve for the transformation in reference frame 1
cosv = cos(thigh);
sinv = sin(thigh);
x = cosv * x + sinv * z;
z = -sinv * x + cosv * z + waist_z;

// solve for the transformation in reference frame 0
cosv = cos(waist);
sinv = sin(waist);
x = cosv * x - sinv * y + waist_x[legid];
y = sinv * x + cosv * y + waist_y[legid];

return vec({ x, y, z });
}

vec Tachikoma::leg_ik_solve(const vec &pos, const vec &enc, int legid) {
vec delta(3);

double x = pos(0, legid) - waist_x[legid];
double y = pos(1, legid) - waist_y[legid];
double z = pos(2, legid) - waist_z;

// find the waist angle
delta(WAIST_POS) = atan2(y, x) - enc(WAIST_POS);

// find the knee angle
x = sqrt(x * x + y * y);
double r = sqrt(x * x + z * z);
delta(KNEE_POS) = cos_rule_angle(thigh_length, knee_length, r) - enc(KNEE_POS);

// find the thigh angle
delta(THIGH_POS) = cos_rule_angle(thigh_length, r, knee_length) - atan2(z, x) - enc(THIGH_POS);

return delta;
}*/

bool Tachikoma::calibrated(void) {
  return this->calibration_loaded;
}

void Tachikoma::move(
    const mat &leg_theta,
    const mat &leg_vel,
    const vec &wheels,
    const mat &arm_theta,
    bool leg_theta_act,
    bool leg_vel_act) {
  this->write_lock->lock();
  this->buffered_leg_theta = leg_theta;
  this->buffered_leg_vel = leg_vel;
  this->buffered_wheels = wheels;
  this->buffered_arm_theta = arm_theta;
  this->buffered_leg_theta_act = leg_theta_act;
  this->buffered_leg_vel_act = leg_vel_act;
  this->write_lock->unlock();
}

void Tachikoma::sense(
    mat &leg_sensors,
    mat &leg_feedback) {
  this->read_lock->lock();
  leg_sensors = this->buffered_leg_sensors;
  leg_feedback = this->buffered_leg_feedback;
  this->read_lock->unlock();
}

void Tachikoma::update_uctrl(void) {
  while (this->manager_running) {
    this->update_send();
    this->update_recv();
  }
}

void Tachikoma::update_send(void) {
  struct timeval currenttime;
  gettimeofday(&currenttime, NULL);
  double secs = secdiff(this->prevwritetime, currenttime);
  if (secs < 1.0 / (double)FPS) {
    return;
  } else {
    memcpy(&this->prevwritetime, &currenttime, sizeof(struct timeval));
  }
  this->write_lock->lock();
  arma::mat leg_theta = this->buffered_leg_theta;
  arma::mat leg_vel = this->buffered_leg_vel;
  arma::vec wheels = this->buffered_wheels;
  arma::mat arm_theta = this->buffered_arm_theta;
  bool leg_theta_act = this->buffered_leg_theta_act;
  bool leg_vel_act = this->buffered_leg_vel_act;
  this->write_lock->unlock();
  this->send(leg_theta, leg_vel, wheels, arm_theta, leg_theta_act, leg_vel_act);
}

void Tachikoma::update_recv(void) {
  arma::mat leg_sensors;
  arma::mat leg_feedback;
  this->recv(leg_sensors, leg_feedback);
  this->read_lock->lock();
  this->buffered_leg_sensors = leg_sensors;
  this->buffered_leg_feedback = leg_feedback;
  this->read_lock->unlock();
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

static double enc_transform(int jointid, double minv, double maxv, int reversed, double value) {
  double enc_range = maxv - minv;
  double rad[2];
  // chnage this later
  switch (jointid) {
    case WAIST:
      rad[0] = -M_PI_4;
      rad[1] = M_PI_4;
      break;
    case THIGH:
      rad[0] = -M_PI_4;
      rad[1] = M_PI_4;
      break;
  } // range is only -90 to 90
  value = limitf(value, rad[0], rad[1]);
  double ratio = enc_range / (rad[1] - rad[0]);
  if (reversed) {
    value = -value;
  }
  return (value - rad[0]) * ratio + minv;
}

static double secdiff(struct timeval &t1, struct timeval &t2) {
  double usec = (double)(t2.tv_usec - t1.tv_usec) / 1000000.0;
  double sec = (double)(t2.tv_sec - t1.tv_sec);
  return sec + usec;
}
