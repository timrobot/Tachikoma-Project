#include <stdio.h>
#include <signal.h>
#include <string.h>
#include <sys/time.h>
#include <stdlib.h>
#include "httplink.h"
#include "user.h"

#define HZ  10

static int input_id;
static httplink_t server;
static xboxctrl_t ctrl;
static pose3d_t user_base;
static pose3d_t user_arm;
static int new_join;
static struct timeval last_signal;
static int user_en;
static int user_ovr;
static void server_update(void);
static void raise_server_request(int signum);
static void controller_update(void);

// TODO set throttle management for multiple connections
//      as of now, it can only handle one request at a time
//      look at js async request handling for inspiration

/** Connect to the user connection - do not enable just yet.
 *  @param id
 *    the id of the user connection to connect to
 *  @return 0 on success, -1 otherwise
 */
int user_connect(int id) {
  int res;
  struct itimerval timer;
  input_id = id;
  switch (id) {
    case USER_SERVER:
      res = httplink_connect(&server, "sevatbr-v002.appspot.com");
      if (res != -1) {
        // assign unthrottle to sigalrm
        struct sigaction action;
        memset(&action, 0, sizeof(struct sigaction));
        action.sa_handler = raise_server_request;
        sigaction(SIGALRM, &action, NULL);
        user_ovr = 1;
      }
      // enable the timer to raise every 1/HZ time
      timer.it_value.tv_sec = 0;
      timer.it_value.tv_usec = 1000000 / HZ;
      timer.it_interval.tv_sec = 0;
      timer.it_interval.tv_usec = 1000000 / HZ;
      setitimer(ITIMER_REAL, &timer, NULL);
      return res;

    case USER_XBOXCTRL:
      xboxctrl_connect(&ctrl);
      return 0;

    default:
      break;
  }
  return -1;
}

/** Disconnect from the manual connection
 *  @return 0, else -1 on error
 */  
int user_disconnect(void) {
  user_set_enable(USER_DISABLE);
  switch (input_id) {
    case USER_SERVER:
      // kill throttle timer
      return httplink_disconnect(&server);

    case USER_XBOXCTRL:
      xboxctrl_disconnect(&ctrl);
      return 0;

    default:
      break;
  }
  return -1;
}

/** Set enable mode
 *  @param en
 *    enable flag
 */
void user_set_enable(int en) {
  if (user_en != en) {
    user_en = en;
    if (en == 0) {
      memset(&user_base, 0, sizeof(pose3d_t));
      memset(&user_arm, 0, sizeof(pose3d_t));
    }
  }
}

/** User override
 *  @return overridden variable, else 0
 */
int user_get_override(void) {
  switch (input_id) {
    case USER_SERVER:
      server_update();
      return user_ovr;

    case USER_XBOXCTRL:
      return user_ovr;

    default:
      break;
  }
  return 0;
}

/** Get the poses
 *  @param b
 *    the base struct
 *  @param a
 *    the arm struct
 *  @ return 1 if valid device chose, else 0
 */
int user_get_poses(pose3d_t *base, pose3d_t *arm) {
  int val;
  switch (input_id) {
    case USER_SERVER:
      server_update(); // flush the server
      val = new_join;
      new_join = 0;
      memcpy(base, &user_base, sizeof(pose3d_t));
      memcpy(arm, &user_arm, sizeof(pose3d_t));
      return val;

    case USER_XBOXCTRL:
      controller_update();
      memcpy(base, &user_base, sizeof(pose3d_t));
      memcpy(arm, &user_arm, sizeof(pose3d_t));
      return 0;

    default:
      memset(&user_base, 0, sizeof(pose3d_t));
      memset(&user_arm, 0, sizeof(pose3d_t));
      return -1;
  }
}

/** Send a log to the user
 *  @param msg
 *    the log message to send over
 */
void user_log(const char *msg) {
  httplink_send(&server, "/send_log", "post", (char *)msg);
}

/** Get the full controller layout
 *  @return the controller struct
 */
xboxctrl_t *user_get_ctrl(void) {
  return &ctrl;
}

/** Private method to get the information sent over from the server
 *  and set the robot with this information
 */
static void server_update(void) {
  char *msg;
  char *sp, *ep;
  char buf[16];
  size_t buflen;
  int ctrlsig;
  int up, down, left, right;
  int lift, drop, grab, release;

  // get message
  if (!(msg = httplink_recv(&server))) {
    long diff;
    struct timeval currtime;
    // reset after some time
    gettimeofday(&currtime, NULL);
    diff = (currtime.tv_usec - last_signal.tv_usec) +
      (currtime.tv_sec - last_signal.tv_sec) * 1000000;
    if (diff >= 1000000) { // specified time is one second (lost internet connection)
      memset(&user_base, 0, sizeof(pose3d_t));
      memset(&user_arm, 0, sizeof(pose3d_t));
      gettimeofday(&last_signal, NULL);
      new_join = 1;
    }
    return;
  }
  sp = strstr(msg, "feedback: ") + sizeof(char) * strlen("feedback: ");
  ep = strstr(sp, " ");
  buflen = (size_t)ep - (size_t)sp;
  if (buflen > 15) {
    // buffer overflow
    return;
  }
  strncpy(buf, sp, buflen);
  buf[buflen] = '\0';
  ctrlsig = atoi(buf);

  // set the signals
  up =          (ctrlsig & 0x00000001) >> 0;
  down =        (ctrlsig & 0x00000002) >> 1;
  left =        (ctrlsig & 0x00000004) >> 2;
  right =       (ctrlsig & 0x00000008) >> 3;
  lift =        (ctrlsig & 0x00000010) >> 4;
  drop =        (ctrlsig & 0x00000020) >> 5;
  grab =        (ctrlsig & 0x00000040) >> 6;
  release =     (ctrlsig & 0x00000080) >> 7;
  user_ovr =    (ctrlsig & 0x00000100) >> 8;

  memset(&user_base, 0, sizeof(pose3d_t));
  memset(&user_arm, 0, sizeof(pose3d_t));
  user_base.y = (double)(up - down);
  user_base.yaw = (double)(left - right);
  user_arm.pitch = (double)(lift - drop);
  user_arm.yaw = (double)(grab - release);

  // update the time and signal
  gettimeofday(&last_signal, NULL);
  new_join = 1;
}

/** Private method to handle server requesting
 *  @param signum
 *    the id for the signal
 */
static void raise_server_request(int signum) {
  if (!user_en) {
    struct itimerval timer;
    memset(&timer, 0, sizeof(struct itimerval));
    setitimer(ITIMER_REAL, &timer, NULL);
  } else {
    httplink_send(&server, "/get_controls", "get", NULL);
  }
}

/** Private to set the base and arm using information
 *  from the controller
 */
void controller_update(void) {
  xboxctrl_update(&ctrl);
  memset(&user_base, 0, sizeof(pose3d_t));
  memset(&user_arm, 0, sizeof(pose3d_t));

  if (ctrl.LJOY.y < 0.015 && ctrl.LJOY.y > -0.015) {
    user_base.y = 0.0;
  } else {
    user_base.y = ctrl.LJOY.y;
  }
  if (ctrl.RJOY.x < 0.015 && ctrl.RJOY.x > -0.015) {
    user_base.yaw = 0.0;
  } else {
    user_base.yaw = -ctrl.RJOY.x;
  }

  user_arm.pitch = ((ctrl.RT + 1.0) / 2.0 - (ctrl.LT + 1.0) / 2.0);
  user_arm.yaw = (ctrl.RB - ctrl.LB) * 1.0;

  if (ctrl.START) {
    user_ovr = 1;
  } else if (ctrl.SELECT) {
    user_ovr = 0;
  }
}
