/************************************
 * The purpose of this object is to
 * translate different formats of
 * communication to their respective
 * devices. You can think of it as
 * the front end, as well as the
 * starting program.
 ***********************************/
#include <cstring>
#include <cstdio>
#include <ctime>
#include <signal.h>
#include "robot.h"
#include "user.h"
#include "agent.h"

// Signal handler for killing the program
// EMERGENCY SHUTDOWN
static int stop_signal;

/** Signal handler to stop the program
 *  @param signum
 *    the signal number (kernel dependent)
 */
void stop_program(int signum) {
  stop_signal = 1;
}

/** This is the starting program for the robot
 *  @param argc
 *    standard exec argument number
 *  @param argv
 *    standard exec argument list
 *  @return 0 on success, -1 otherwise
 */
int main(int argc, char *argv[]) {
  int input_id;
  enum input_states { S_USER, S_AGENT };

  signal(SIGINT, stop_program);
  // for current testing purposes, connect to the controller first
  if (user_connect(USER_SERVER) == -1) {
    return -1;
  }
  user_log("CORE INITIALIZING...\n");
  // init robot and user
  if (robot::set(TENNIS_BALL_ROBOT) == -1) {
    return -1;
  }
  user_log("initializing agent\n");
  if (agent::wakeup() == -1) {
    user_disconnect();
    return -1;
  }
  input_id = S_AGENT;
  agent::set_enable(true);

  // start getting communication accesses
  user_log("querying...\n");
  while (!stop_signal) {
    switch (input_id) {
      case S_USER:
        user_get_poses(base, arm);
        robot::move(base, arm);
        if (!user_get_override()) {
          user_set_enable(false);
          agent::set_enable(true);
          input_id = S_AGENT;
        }
        break;

      case S_AGENT:
        agent::get_poses(base, arm);
        robot::move(base, arm);
        if (user_get_override()) {
          agent::set_enable(false);
          user_set_enable(true);
          input_id = S_USER;
        }
        break;
    }
  }

  // clean up
  agent::gotosleep();
  user_disconnect();
  robot::unset();
  user_log("CORE COMPLETE.\n");

  return 0;
}
