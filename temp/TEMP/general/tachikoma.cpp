/****************************************
 *
 * The purpose of this API is to do 
 * the following for this particular bot:
 *
 *	1) control the robot through
 *		 abstracted methods
 *	2) send back sensor map values
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
#include "mathfun.h"

#define WBUFSIZE	128
#define FPS 10

using namespace arma;
using namespace std;
using json = nlohmann::json;

static double secdiff(struct timeval &t1, struct timeval &t2);

/** CLASS FUNCTIONS **/

Tachikoma::Tachikoma(void) : BaseRobot(TACHIKOMA) {
	this->calibration_loaded = false;
	// arm defines
	this->arm_link_length = { 0.0, 10.5, 6.5, 4.5, 3.75 };
	this->arm_write_pos = zeros<vec>(DOF);
	this->arm_start_pos = vec({ 0, 90, 0, 0 }); // degrees
	this->arm_write_vel = zeros<vec>(DOF);
	this->arm_read_pos = zeros<vec>(DOF);
	this->arm_read_vel = zeros<vec>(DOF);
	this->arm_min_pos = zeros<vec>(DOF);
	this->arm_max_pos = zeros<vec>(DOF);
	this->arm_min_enc = zeros<vec>(DOF);
	this->arm_max_enc = zeros<vec>(DOF);
	this->arm_pos_act = false;
	this->arm_vel_act = false;
	// leg defines
	this->leg_write_pos = zeros<mat>(NUM_LEGS, NUM_JOINTS);
	this->leg_start_pos = mat({
				0, 0,
				0, 0,
				0, 0,
				0, 0 });
	this->leg_start_pos.reshape(2, 4); // degrees
	this->leg_start_pos = this->leg_start_pos.t();
	this->leg_write_vel = zeros<mat>(NUM_LEGS, NUM_JOINTS);
	this->leg_read_pos = zeros<mat>(NUM_LEGS, NUM_JOINTS);
	this->leg_read_vel = zeros<mat>(NUM_LEGS, NUM_JOINTS);
	this->leg_min_pos = zeros<mat>(NUM_LEGS, NUM_JOINTS);
	this->leg_max_pos = zeros<mat>(NUM_LEGS, NUM_JOINTS);
	this->leg_min_enc = zeros<mat>(NUM_LEGS, NUM_JOINTS);
	this->leg_max_enc = zeros<mat>(NUM_LEGS, NUM_JOINTS);
	this->leg_pos_act = false;
	this->leg_vel_act = false;
	// wheel defines
	this->wheel_write_vel = zeros<vec>(NUM_LEGS);
	this->wheel_read_pos = zeros<vec>(NUM_LEGS);
	this->wheel_read_vel = zeros<vec>(NUM_LEGS);
	this->wheel_vel_act = false;
	// multithreaded
	this->uctrl_manager = nullptr;
	this->read_lock.unlock();
	this->write_lock.unlock();
	this->manager_running = false;
	this->load_calibration_params("calib_params.json");
	memset(&this->prevwritetime, 0, sizeof(struct timeval));
	gettimeofday(&this->prevwritetime, NULL);
	// connect to the tachikoma
	//if (!this->connect()) {
	//	printf("[TACHIKOMA] Error: cannot connect to any devices!\n");
	//}
}

Tachikoma::~Tachikoma(void) {
	this->disconnect();
	printf("[TACHIKOMA] Disconnected.\n");
}

bool Tachikoma::connect(void) {
	if (!this->connected()) {
		bool status = BaseRobot::connect();
		if (!this->connected() || !status) {
			// if the device has failed either parent or derived checks, disconnect
			this->disconnect();
			return false;
		}
		this->reset();
		this->thread_send(
				this->arm_start_pos,
				this->arm_write_vel,
				this->leg_start_pos,
				this->leg_write_vel,
				this->wheel_write_vel,
				false, false, false, false, false);

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
	if (this->connected()) {
		// signal the manager to stop updating
		this->manager_running = false;
		if (this->uctrl_manager) {
			// wait for the uctrl_manager to join
			this->uctrl_manager->join();
			delete this->uctrl_manager;
			this->uctrl_manager = nullptr;
		}
		// set all the values to false
		this->thread_send(
				this->arm_write_pos,
				this->arm_write_vel, 
				this->leg_write_pos,
				this->leg_write_vel,
				this->wheel_write_vel,
				false, false, false, false, false);
		this->reset();
		// shut down all systems
		BaseRobot::disconnect();
	}
}

void Tachikoma::reset(void) {
	this->arm_pos_act = false;
	this->leg_pos_act = false;
	this->leg_vel_act = false;
	this->wheel_vel_act = false;
}

void Tachikoma::update_uctrl(void) {
	struct timeval currtime;
	vec arm_pos(DOF, fill::zeros);
	vec arm_vel(DOF, fill::zeros);
	mat leg_pos(NUM_LEGS, NUM_JOINTS, fill::zeros);
	mat leg_vel(NUM_LEGS, NUM_JOINTS, fill::zeros);
	vec wheel_pos(NUM_LEGS, fill::zeros);
	vec wheel_vel(NUM_LEGS, fill::zeros);
	bool arm_pos_act;
	bool arm_vel_act;
	bool leg_pos_act;
	bool leg_vel_act;
	bool wheel_vel_act;

	while (this->manager_running) {
		gettimeofday(&currtime, NULL);
		if (secdiff(prevwritetime, currtime) >= 0.05) { // 50ms update
			this->write_lock.lock();
			arm_pos = this->arm_write_pos;
			arm_vel = this->arm_write_vel;
			leg_pos = this->leg_write_pos;
			leg_vel = this->leg_write_vel;
			wheel_vel = this->wheel_write_vel;
			arm_pos_act = this->arm_pos_act;
			arm_vel_act = this->arm_vel_act;
			leg_pos_act = this->leg_pos_act;
			leg_vel_act = this->leg_vel_act;
			wheel_vel_act = this->wheel_vel_act;
			this->write_lock.unlock();
			this->thread_send(arm_pos, arm_vel, leg_pos, leg_vel, wheel_vel,
					arm_pos_act, arm_vel_act, leg_pos_act, leg_vel_act, wheel_vel_act);
			memcpy(&prevwritetime, &currtime, sizeof(struct timeval));
		}
		this->thread_recv(arm_pos, arm_vel, leg_pos, leg_vel, wheel_pos, wheel_vel);
		this->read_lock.lock();
		this->arm_read_pos = arm_pos;
		this->arm_read_vel = arm_vel;
		this->leg_read_pos = leg_pos;
		this->leg_read_vel = leg_vel;
		this->wheel_read_pos = wheel_pos;
		this->wheel_read_vel = wheel_vel;
		this->read_lock.unlock();
	}
}

void Tachikoma::thread_send(
		vec arm_pos, vec arm_vel,
		mat leg_pos, mat leg_vel,
		vec wheel_vel,
		bool arm_pos_act, bool arm_vel_act,
		bool leg_pos_act, bool leg_vel_act,
		bool wheel_vel_act) {
	assert(arm_pos.n_elem == DOF);
	assert(arm_vel.n_elem == DOF);
	assert(leg_pos.n_rows == NUM_LEGS && leg_pos.n_cols == NUM_JOINTS);
	assert(leg_vel.n_rows == NUM_LEGS && leg_vel.n_cols == NUM_JOINTS);
	assert(wheel_vel.n_elem == NUM_LEGS);

	int devid;

	// safety checks for the legs and wheels
	for (int i = 0; i < NUM_LEGS; i++) {
		for (int j = 0; j < NUM_JOINTS; j++) {
			// use calibration definitions
			if (this->calibrated()) {
				leg_pos(i, j) = limit_value(leg_pos(i, j), this->leg_min_pos(i, j), this->leg_max_pos(i, j));
				leg_pos(i, j) = map_value(leg_pos(i, j),
						this->leg_min_pos(i, j), this->leg_max_pos(i, j),
						this->leg_min_enc(i, j), this->leg_max_enc(i, j));
			}
			leg_vel(i, j) = limit_value(leg_vel(i, j), -1.0, 1.0);
		}
		wheel_vel(i) = limit_value(wheel_vel(i), -1.0, 1.0);
	}

	// instruction char representing action to undertake (global)
	char instr_activate =
			((arm_pos_act) ? 0x01 : 0x00) |
			(arm_vel_act ? 0x02 : 0x00) |
			((leg_pos_act && this->calibrated()) ? 0x04 : 0x00) |
			(leg_vel_act ? 0x08 : 0x00) |
			(wheel_vel_act ? 0x10 : 0x00);

	/*cout << "==============================" << endl;
	cout << leg_vel << endl;
	cout << wheel_vel << endl;
	cout << leg_pos_act << endl;
	cout << "==============================" << endl;*/

	// write to device
	char msg[WBUFSIZE];
	for (size_t i = 0; i < this->connections.size(); i++) {
		if (this->ids[i] > 0 && this->ids[i] <= NUM_DEV) {
			switch ((devid = this->ids[i])) {

				// legs and wheels
				case FRONT_LEFT:
				case FRONT_RIGHT:
				case BACK_LEFT:
				case BACK_RIGHT:
					sprintf(msg, "[%d %d %d %d %d %d]\n",
							instr_activate,
							(int)(leg_pos(devid - 1, WAIST)),
							(int)(leg_pos(devid - 1, THIGH)),
							(int)(leg_vel(devid - 1, WAIST) * 255.0),
							(int)(leg_vel(devid - 1, THIGH) * 255.0),
							(int)(wheel_vel(devid - 1) * 255.0));
					serial_write(this->connections[i], msg);
					break;

				// arm portion 1
				case ARM_DEV:
					sprintf(msg, "[%d %d %d %d %d %d %d %d %d]\n",
							instr_activate,
							(int)(arm_pos(PIVOT1)),
							(int)(arm_pos(PIVOT2)),
							(int)(arm_pos(PIVOT3)),
							(int)(arm_pos(PIVOT4)),
							(int)(arm_vel(PIVOT1) * 255.0),
							(int)(arm_vel(PIVOT2) * 255.0),
							(int)(arm_vel(PIVOT3) * 255.0),
							(int)(arm_vel(PIVOT4) * 255.0));
					if (arm_pos_act) {
					  printf ("ARM ENABLED: %s\n", msg);
					}
					serial_write(this->connections[i], msg);

				default:
					break;
			}
		}
	}
}

void Tachikoma::thread_recv(
		vec &arm_pos, vec &arm_vel,
		mat &leg_pos, mat &leg_vel,
		vec &wheel_pos, vec &wheel_vel) {
	if (arm_pos.n_elem != DOF) {
		arm_pos = zeros<vec>(DOF);
	}
	if (arm_vel.n_elem != DOF) {
		arm_vel = zeros<vec>(DOF);
	}
	if (leg_pos.n_rows != NUM_LEGS || leg_pos.n_cols != NUM_JOINTS) {
		leg_pos = zeros<mat>(NUM_LEGS, NUM_JOINTS);
	}
	if (leg_vel.n_rows != NUM_LEGS || leg_vel.n_cols != NUM_JOINTS) {
		leg_vel = zeros<mat>(NUM_LEGS, NUM_JOINTS);
	}
	if (wheel_pos.n_elem != NUM_LEGS) {
		wheel_pos = zeros<vec>(NUM_LEGS);
	}
	if (wheel_vel.n_elem != NUM_LEGS) {
		wheel_vel = zeros<vec>(NUM_LEGS);
	}

	char *msg;
	int devid;
	int pot[4];
	int shaftenc;
	int vel[4];
	//int cr[2];

	// read from device
	for (int i = 0; i < (int)this->connections.size(); i++) {
		if (this->ids[i] > 0 && this->ids[i] <= NUM_DEV) {
			switch ((devid = this->ids[i])) {

				// legs and wheels
				case FRONT_LEFT:
				case FRONT_RIGHT:
				case BACK_LEFT:
				case BACK_RIGHT:
					if ((msg = serial_read(this->connections[i]))) {
						sscanf(msg, "[%d %d %d %d %d %d %d]\n", &this->ids[i],
								&pot[0], &pot[1], &shaftenc, &vel[0], &vel[1], &vel[2]);
						leg_pos(devid - 1, WAIST) = map_value(pot[0],
								this->leg_min_enc(devid - 1, WAIST), this->leg_max_enc(devid - 1, WAIST),
								this->leg_min_pos(devid - 1, WAIST), this->leg_max_pos(devid - 1, WAIST));
						leg_pos(devid - 1, THIGH) = map_value(pot[1],
								this->leg_min_enc(devid - 1, THIGH), this->leg_max_enc(devid - 1, THIGH),
								this->leg_min_pos(devid - 1, THIGH), this->leg_max_pos(devid - 1, THIGH));
						wheel_pos(devid - 1) = shaftenc;
						leg_vel(devid - 1, WAIST) = (double)vel[0] / 255.0;
						leg_vel(devid - 1, THIGH) = (double)vel[1] / 255.0;
						wheel_vel(devid - 1) = (double)vel[2] / 255.0;
						//printf("recvd leg %d\n", this->ids[i]);
					}
					break;

				// arm portion
				case ARM_DEV:
					if ((msg = serial_read(this->connections[i]))) {
						sscanf(msg, "[%d %d %d %d %d %d %d %d %d]\n", &this->ids[i],
								&pot[0], &pot[1], &pot[2], &pot[3],
								&vel[0], &vel[1], &vel[2], &vel[3]);
						/*arm_pos(PIVOT1) = map_value(pot[0],
								this->arm_min_enc(PIVOT1), this->arm_max_enc(PIVOT1),
								this->arm_min_pos(PIVOT1), this->arm_max_pos(PIVOT1));
						arm_pos(PIVOT2) = map_value(pot[1],
								this->arm_min_enc(PIVOT2), this->arm_max_enc(PIVOT2),
								this->arm_min_pos(PIVOT2), this->arm_max_pos(PIVOT2));
						arm_pos(PIVOT3) = map_value(pot[2],
								this->arm_min_enc(PIVOT3), this->arm_max_enc(PIVOT3),
								this->arm_min_pos(PIVOT3), this->arm_max_pos(PIVOT3));
						arm_pos(PIVOT4) = map_value(pot[3],
								this->arm_min_enc(PIVOT4), this->arm_max_enc(PIVOT4),
								this->arm_min_pos(PIVOT4), this->arm_max_pos(PIVOT4));*/
						arm_pos(PIVOT1) = pot[0];
						arm_pos(PIVOT2) = pot[1];
						arm_pos(PIVOT3) = pot[2];
						arm_pos(PIVOT4) = pot[3];
						arm_vel(PIVOT1) = (double)vel[0] / 255.0;
						arm_vel(PIVOT2) = (double)vel[1] / 255.0;
						arm_vel(PIVOT3) = (double)vel[2] / 255.0;
						arm_vel(PIVOT4) = (double)vel[3] / 255.0;
//printf("recvd arm\n");
	/*cout << "==============================" << endl;
	cout << arm_vel << endl;
	cout << arm_pos << endl;
	cout << "==============================" << endl;*/
					}
					break;

				default:
				/*	if ((msg = serial_read(this->connections[i]))) {
						int _id;
						sscanf(msg, "[%d ", &_id);
						printf("recvd random msg: %d = %s\n", _id, msg);
					}*/
					break;
			}
		}
	}
}

void Tachikoma::load_calibration_params(const string &filename) {
	string params;
	ifstream params_file(filename);
	string temp;
	while (getline(params_file, temp)) {
		params += temp;
	}
	params_file.close();
	this->set_calibration_params(json::parse(params));
}

void Tachikoma::set_calibration_params(json cp) {
	vector<string> armnames = { "pivot1", "pivot2", "pivot3", "pivot4" };
	vector<int> armids = { PIVOT1, PIVOT2, PIVOT3, PIVOT4 };
	for (int i = 0; i < DOF; i++) {
		string armname = armnames[i];
		int armid = armids[i];
		this->arm_min_enc(armid) = cp["arm"][armname]["min_enc"];
		this->arm_max_enc(armid) = cp["arm"][armname]["max_enc"];
		this->arm_min_pos(armid) = cp["arm"][armname]["min_pos"];
		this->arm_max_pos(armid) = cp["arm"][armname]["max_pos"];
	}
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
			this->leg_min_enc(legid, jointid) = cp[legname][jointname]["min_enc"];
			this->leg_max_enc(legid, jointid) = cp[legname][jointname]["max_enc"];
			this->leg_min_pos(legid, jointid) = cp[legname][jointname]["min_pos"];
			this->leg_max_pos(legid, jointid) = cp[legname][jointname]["max_pos"];
		}
	}
	this->calibration_loaded = true;
}

bool Tachikoma::calibrated(void) {
	return this->calibration_loaded;
}

void Tachikoma::send(
		vec arm_pos, vec arm_vel,
		mat leg_pos, mat leg_vel,
		vec wheel_vel,
		bool arm_pos_act, bool arm_vel_act,
		bool leg_pos_act, bool leg_vel_act,
		bool wheel_vel_act) {
	this->write_lock.lock();
	this->arm_write_pos = arm_pos;
	this->arm_write_vel = arm_vel;
	this->leg_write_pos = leg_pos;
	this->leg_write_vel = leg_vel;
	this->wheel_write_vel = wheel_vel;
	this->arm_pos_act = arm_pos_act;
	this->arm_vel_act = arm_vel_act;
	this->leg_pos_act = leg_pos_act;
	this->leg_vel_act = leg_vel_act;
	this->wheel_vel_act = wheel_vel_act;
	this->write_lock.unlock();
}

void Tachikoma::recv(
		vec &arm_pos, vec &arm_vel,
		mat &leg_pos, mat &leg_vel,
		vec &wheel_pos, vec &wheel_vel) {
	this->read_lock.lock();
	arm_pos = this->arm_read_pos;
	arm_vel = this->arm_read_vel;
	leg_pos = this->leg_read_pos;
	leg_vel = this->leg_read_vel;
	wheel_pos = this->wheel_read_pos;
	wheel_vel = this->wheel_read_vel;
	this->read_lock.unlock();
}

void Tachikoma::set_arm(
		double pivot1_pos,
		double pivot2_pos,
		double pivot3_pos,
		double grab_pos) {
	this->write_lock.lock();
	this->arm_write_pos = vec({
			pivot1_pos,
			pivot2_pos,
			pivot3_pos,
			grab_pos });
	this->arm_vel_act = false;
	this->arm_pos_act = true;
	this->write_lock.unlock();
}

void Tachikoma::move_arm(
		double pivot1_vel,
		double pivot2_vel,
		double pivot3_vel,
		double grab_vel) {
	this->write_lock.lock();
	this->arm_write_vel = vec({
			pivot1_vel,
			pivot2_vel,
			pivot3_vel,
			grab_vel });
	this->arm_pos_act = false;
	this->arm_vel_act = true;
	this->write_lock.unlock();
}

void Tachikoma::stop_arm(void) {
	this->write_lock.lock();
	this->arm_pos_act = false;
	this->arm_vel_act = false;
	this->write_lock.unlock();
}

void Tachikoma::set_legs(
		double top_left_waist_pos,
		double top_right_waist_pos,
		double bottom_left_waist_pos,
		double bottom_right_waist_pos,
		double top_left_thigh_pos,
		double top_right_thigh_pos,
		double bottom_left_thigh_pos,
		double bottom_right_thigh_pos) {
	this->write_lock.lock();
	this->leg_write_pos = mat({ 
			top_left_waist_pos,
			top_right_waist_pos,
			bottom_left_waist_pos,
			bottom_right_waist_pos,
			top_left_thigh_pos,
			top_right_thigh_pos,
			bottom_left_thigh_pos,
			bottom_right_thigh_pos });
	this->leg_write_pos.reshape(4, 2);
	this->leg_vel_act = false;
	this->leg_pos_act = false; // TODO: change back to regular
	this->write_lock.unlock();
}

void Tachikoma::move_legs(
		double top_left_waist_vel,
		double top_right_waist_vel,
		double bottom_left_waist_vel,
		double bottom_right_waist_vel,
		double top_left_thigh_vel,
		double top_right_thigh_vel,
		double bottom_left_thigh_vel,
		double bottom_right_thigh_vel) {
	this->write_lock.lock();
	this->leg_write_vel = mat({ 
			top_left_waist_vel,
			top_right_waist_vel,
			bottom_left_waist_vel,
			bottom_right_waist_vel,
			top_left_thigh_vel,
			top_right_thigh_vel,
			bottom_left_thigh_vel,
			bottom_right_thigh_vel });
	this->leg_write_vel.reshape(4, 2);
	this->leg_pos_act = false;
	this->leg_vel_act = false; // TODO: change back to regular
	this->write_lock.unlock();
}

void Tachikoma::stop_legs(void) {
	this->write_lock.lock();
	this->leg_pos_act = false;
	this->leg_vel_act = false;
	this->write_lock.unlock();
}

void Tachikoma::set_wheels(
		double top_left_wheel_vel,
		double top_right_wheel_vel,
		double bottom_left_wheel_vel,
		double bottom_right_wheel_vel) {
	this->write_lock.lock();
	this->wheel_write_vel = vec({
			top_left_wheel_vel,
			top_right_wheel_vel,
			bottom_left_wheel_vel,
			bottom_right_wheel_vel });
	this->wheel_vel_act = true;
	this->write_lock.unlock();
}

void Tachikoma::stop_wheels(void) {
	this->write_lock.lock();
	this->wheel_vel_act = false;
	this->write_lock.unlock();
}

vec Tachikoma::get_end_effector_pos(int linkid)
{
	// solve arm (using D-H notation and forward kinematics)
	vec angles = this->arm_read_pos;
	vec lengths = this->arm_link_length;

	// get the rotations
	vector<mat> rotate =
	{
		rotationMat(-angles(0), 0, 0),
		rotationMat(-angles(1), 0, 0),
		rotationMat(-angles(2), 0, 0),
		rotationMat(0, 0, 0)
	};

	// get the translations
	vector<vec> translate =
	{
		{ 0, 0, lengths(0) },
		{ 0, 0, lengths(1) },
		{ 0, 0, lengths(2) },
		{ 0, 0, lengths(3) },
		{ 0, 0, lengths(4) }
	};

	// get the position using the combination of rotations
	// and the translations up to the linkid [0|1|2|3|4]
	vec pos = translate[linkid];
	for (int i = linkid - 1; i >= 0; i--)
	{
		pos = rotate[i] * pos + translate[i];
	}

	return pos;
}


/** STATIC FUNCTIONS **/

static double secdiff(struct timeval &t1, struct timeval &t2) {
	double usec = (double)(t2.tv_usec - t1.tv_usec) / 1000000.0;
	double sec = (double)(t2.tv_sec - t1.tv_sec);
	return sec + usec;
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
double knee	= enc(KNEE_POS);

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
