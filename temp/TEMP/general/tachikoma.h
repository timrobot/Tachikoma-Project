#ifndef __TK_TACHIKOMA_H__
#define __TK_TACHIKOMA_H__

#include <armadillo>
#include <string>
#include <thread>
#include <mutex>
#include <njson/json.hpp>
#include <sys/time.h>
#include "baserobot.h"

// General Definitions //
#define DOF					4
#define PIVOT1			0
#define PIVOT2			1
#define PIVOT3			2
#define PIVOT4			3
#define NUM_LEGS		4
#define NUM_JOINTS	2
#define WAIST				0
#define THIGH				1
#define UL					0
#define UR					1
#define DL					2
#define DR					3

// Device Ids //
#define NUM_DEV			1002
#define FRONT_LEFT	1
#define FRONT_RIGHT	2
#define BACK_LEFT		3
#define BACK_RIGHT	4
#define ARM_DEV			1001

class Tachikoma : public BaseRobot {
	public:
		/** Constructor
		 */
		Tachikoma(void);

		/** Deconstructor
		 */
		~Tachikoma(void);

		/** Try to connect to the robot and start a thread for the device update
		 *	@return true if successful, false otherwise
		 */
		bool connect(void);

		/** Detect if the tachikoma is connected to all ports or not
		 *	@return true if all ports are connected, false otherwise
		 */
		bool connected(void);

		/** Detect the number of devices that are connected to the Tachikoma
		 *	@return the number of ports that are connected
		 */
		int numconnected(void);

		/** Disconnect from all devices
		 */
		void disconnect(void);

		/** Reset the robot's default values
		 */
		void reset(void);

		/** Send vectors of values to the microcontrollers
		 *	@param arm_pos { pivot1, pivot2, pivot3, pivot4, claw } positions
		 *	@param arm_vel { pivot1, pivot2, pivot3, pivot4, claw } velocities
		 *	@param leg_pos 4x2 matrix of positions [legid][jointid]
		 *	@param leg_vel 4x2 matrix of velocities [legid][jointid]
		 *	@param wheel_vel 4x1 vector of velocities [legid]
		 *	@param arm_pos_act (optional) enable the arm position
		 *	@param arm_vel_act (optional) enable the arm velocity
		 *	@param leg_pos_act (optional)	enable the leg position
		 *	@param leg_vel_act (optional) enable the leg velocity
		 *	@param wheel_vel_act (optional) enable the wheel velocity
		 */
		void send(
				arma::vec arm_pos,		// 4x1 vector
				arma::vec arm_vel,		// 4x1 vector
				arma::mat leg_pos,		// 4x2 matrix
				arma::mat leg_vel,		// 4x2 matrix
				arma::vec wheel_vel,	// 4x1 vector
				bool arm_pos_act = false,
				bool arm_vel_act = false,
				bool leg_pos_act = false,
				bool leg_vel_act = false,
				bool wheel_vel_act = false);

		/** Receive a matrix of sensor values from the legs, indicating (for now) angles and touch
		 */
		void recv(
				arma::vec &arm_pos,
				arma::vec &arm_vel,
				arma::mat &leg_pos,
				arma::mat &leg_vel,
				arma::vec &wheel_pos,
				arma::vec &wheel_vel);

		/** Set the calibration parameters for the robot
		 *	@param filename a name of a file
		 *	@param cp the calibration parameters
		 */
		void load_calibration_params(const std::string &filename);
		void set_calibration_params(nlohmann::json cp);

		/** Detect if a robot has been calibrated
		 *	@return true if calibration parameters are found, false otherwise
		 */
		bool calibrated(void);

		/** Solve the xyz coordinate of the leg using forward kinematics
		 *	@param waist, thigh, knee
		 *		the current encoder value vector (waist, thigh, knee)
		 *	@param legid
		 *		the id the leg to solve for
		 *	@return the position vector (x, y, z)
		 */
		arma::vec leg_fk_solve(const arma::vec &enc, int legid);

		/** Solve the encoder values of the legs given a target
		 *	@param pos
		 *		the target position vector (x, y, z)
		 *	@param enc
		 *		the current encoder value vector (waist, thigh, knee)
		 *	@param legid
		 *		the id the leg to solve for
		 *	@return the differential encoder vector (dx, dy, dz)
		 */
		arma::vec leg_ik_solve(const arma::vec &pos, const arma::vec &enc, int legid);

		/** easier than send/recv
		 */
		void set_arm(
				double pivot1_pos,
				double pivot2_pos,
				double pivot3_pos,
				double grab_pos);
		void move_arm(
				double pivot1_vel,
				double pivot2_vel,
				double pivot3_vel,
				double grab_vel);
		void stop_arm(void);
		void set_legs(
				double top_left_waist_pos,
				double top_right_waist_pos,
				double bottom_left_waist_pos,
				double bottom_right_waist_pos,
				double top_left_thigh_pos,
				double top_right_thigh_pos,
				double bottom_left_thigh_pos,
				double bottom_right_thigh_pos);
		void move_legs(
				double top_left_waist_vel,
				double top_right_waist_vel,
				double bottom_left_waist_vel,
				double bottom_right_waist_vel,
				double top_left_thigh_vel,
				double top_right_thigh_vel,
				double bottom_left_thigh_vel,
				double bottom_right_thigh_vel);
		void stop_legs(void);
		void set_wheels(
				double top_left_wheel_vel,
				double top_right_wheel_vel,
				double bottom_left_wheel_vel,
				double bottom_right_wheel_vel);
		void stop_wheels(void);

		arma::vec get_end_effector_pos(int linkid);

	private:
		// update manager
		std::thread *uctrl_manager;
		std::mutex read_lock;
		std::mutex write_lock;
		// used for the device update
		bool manager_running;
		void update_uctrl(void);
		void thread_send(
				arma::vec arm_pos,		// 4x1 vector
				arma::vec arm_vel,		// 4x1 vector
				arma::mat leg_pos,		// 4x2 matrix
				arma::mat leg_vel,		// 4x2 matrix
				arma::vec wheel_vel,	// 4x1 vector
				bool arm_pos_act = false,
				bool arm_vel_act = false,
				bool leg_pos_act = false,
				bool leg_vel_act = false,
				bool wheel_vel_act = false);
		void thread_recv(
				arma::vec &arm_pos,
				arma::vec &arm_vel,
				arma::mat &leg_pos,
				arma::mat &leg_vel,
				arma::vec &wheel_pos,
				arma::vec &wheel_vel);

		struct timeval prevwritetime;
		bool calibration_loaded;

		// arm defines
		arma::vec arm_link_length;
		arma::vec arm_write_pos;
		arma::vec arm_start_pos;
		arma::vec arm_write_vel;
		arma::vec arm_read_pos;
		arma::vec arm_read_vel;
		arma::vec arm_min_pos;
		arma::vec arm_max_pos;
		arma::vec arm_min_enc;
		arma::vec arm_max_enc;
		bool arm_pos_act;
		bool arm_vel_act;
		// leg defines
		arma::mat leg_write_pos;
		arma::mat leg_start_pos;
		arma::mat leg_write_vel;
		arma::mat leg_read_pos;
		arma::mat leg_read_vel;
		arma::mat leg_min_pos;
		arma::mat leg_max_pos;
		arma::mat leg_min_enc;
		arma::mat leg_max_enc;
		bool leg_pos_act;
		bool leg_vel_act;
		// wheel defines
		arma::vec wheel_write_vel;
		arma::vec wheel_read_pos;
		arma::vec wheel_read_vel;
		bool wheel_vel_act;
};

#endif
