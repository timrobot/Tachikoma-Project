#include "action.h"
#include "tachikoma.h"
#include "defs.h"
#include "motion.h"

#define ACTION_IDLE       0
#define ACTION_DRIVE      1
#define ACTION_WALK       2
#define ACTION_STAIR_UP   3
#define ACTION_STAIR_DOWN 4
#define ACTION_STAND      5
#define ACTION_SIT        6

#define M_3_PI_4  2.356194490192345
#define M_5_PI_4  3.9269908169872414
#define M_3_PI_2  4.71238898038469
#define M_7_PI_4  5.497787143782138

/** see if an action is finished or not
 */
bool Tachikoma::action_finished(void);

/** idle pose
*/
void Tachikoma::update_idle(void) {
  this->action_id = ACT_IDLE;
  this->action_state_finished = false;
  this->action_motion = zeros<vec>(16);
  this->action_state_finished = true;
}

/** drive around using the normal poses (either directly forward, or other)
*/
void Tachikoma::update_drive(vec v, double w) {
  this->action_id = ACT_DRIVE;
  this->action_state_finished = false;
  double waist;
  double thigh;
  double knee;
  double waist_rad[4];
  // can't go any faster than normal
  if (dot(v, v) > 1.0) {
    v = normalise(v);
  }
  w = (w > 1.0) ? 1.0 : ((w < -1.0) ? -1.0 : w);
  // define different angles depending on the velocity
  if (v(0) == 0.0 && v(1) > 0.0 && w == 0.0) {
    waist_rad[NW] = M_PI;
    waist_rad[NE] = 0.0;
    waist_rad[SW] = M_PI;
    waist_rad[SE] = 0.0;
  } else {
    waist_rad[NW] = M_3_PI_4;
    waist_rad[NE] = M_PI_4;
    waist_rad[SW] = M_5_PI_4;
    waist_rad[SE] = M_7_PI_4;
  }
  for (int i = 0; i < 4; i++) {
    // set the wheel movement
    double angle = this->leg_sensors(ENC_WAIST, i);
    vec u = { -sin(angle), cos(angle) };
    // give higher weight to the forward/strafe movement
    this->action_motion(WHEEL_IND[i]) = limitf(dot(u, v) * 1.5 + w, -1.0, 1.0);
    // set the waist, thigh, and knee movement
    waist = waist_rad[i] - this->leg_sensors(ENC_WAIST, i);
    thigh = M_PI_2 - this->leg_sensors(ENC_THIGH, i);
    knee  = M_PI_2 - this->leg_sensors(ENC_KNEE, i);
    this->action_motion(WAIST_IND[i]) = (waist > 0.5) ? 1.0 : ((waist < -0.5) ? -1.0 : 0.0);
    this->action_motion(THIGH_IND[i]) = (thigh > 0.5) ? 1.0 : ((thigh < -0.5) ? -1.0 : 0.0);
    this->action_motion(KNEE_IND[i])  = (knee > 0.5)  ? 1.0 : ((knee < -0.5)  ? -1.0 : 0.0);
  }
  this->action_state_finished = true;
}

/** take a step forward
 */
void Tachikoma::update_forward_step(const vec &curr_pos, const vec &target_pos, int legid) {
  this->action_step_finished = false;
  switch (this->action_step_state) {
    case 0:
      vec mid = (curr_pos + target_pos) / 2.0;
      mid += normal(mid) * sigma;
      this->step_f_mid = mid;
      this->action_step_state++;
      this->action_step_finished = false;
    case 1:
      if (!within(curr_pos, this->step_f_mid)) {
        vec delta = this->leg_ik_solve(this->step_f_mid, this->leg_fk_inverse_solve(curr_pos), legid);
        this->action_motion(WAIST_IND[legid]) = delta(ENC_WAIST);
        this->action_motion(THIGH_IND[legid]) = delta(ENC_THIGH);
        this->action_motion(KNEE_IND[legid])  = delta(ENC_KNEE);
        this->action_motion(WHEEL_IND[legid]) = 0.0;
      } else {
        this->action_step_state++;
      }
      break;
    case 2:
      if (!within(curr_pos, this->target_pos)) {
        vec delta = this->leg_ik_solve(this->target_pos, this->leg_fk_inverse_solve(curr_pos), legid);
        this->action_motion(WAIST_IND[legid]) = delta(ENC_WAIST);
        this->action_motion(THIGH_IND[legid]) = delta(ENC_THIGH);
        this->action_motion(KNEE_IND[legid])  = delta(ENC_KNEE);
        this->action_motion(WHEEL_IND[legid]) = 0.0;
      } else {
        this->action_step_state = 0;
        this->action_step_finished = true;
      }
      break;
    default:
      break;
  }
}

void Tachikoma::update_walk(const vec &v, double w) {
  this->action_id = ACT_WALK;
}

void Tachikoma::update_stair_up(void) {
  this->action_id = ACT_STAIR_UP;
  this->action_state_finished = false;
  ActionNode *action = this->action_tree->select("stair_up_para");
  if (!action) {
    return;
  }
  this->action_motion = action->get_motion(this->leg_positions);
  if (action->finished()) {
    action->next();
    this->action_state_finished = true;
  }
}

void Tachikoma::update_stair_down(void) {
  this->action_id = ACT_STAIR_DOWN;
}

void Tachikoma::update_stand(void) {
  this->action_id = ACT_STAND;
}

void Tachikoma::update_sit(void) {
  this->action_id = ACT_SIT:
}

vec linear_step_fcn(void *args) {
  vec path = stop - start;
  return start + path * t;
}

static double limitf(double value, double min_value, double max_value) {
  if (value < min_value) {
    return min_value;
  } else if (value > max_value) {
    return max_value;
  } else {
    return value;
  }
}

/** create all the actions
 */
void Tachikoma::remember_actions(void) {
  this->action_tree = new ActionNode();
  this->action_tree->insert("stair_up_para")->insert("NWNESWSE")->
    insert("fwdstepA", new ActionState(reshape(mat({
        0.3, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0,
        1.2, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0
    }), 4, 4).t(), linear_step_fcn));
  this->action_tree->insert("stair_up_para")->insert("NWNESWSE")->
    insert("fwdstepB", new ActionState(reshape(mat({
        0.7, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0,
        -0.2, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0
    }), 4, 4).t(), linear_step_fcn));
  this->action_tree->insert("stair_up_para")->insert("NENWSWSE")->
    insert("fwdstepA", new ActionState(reshape(mat({
        0.0, 0.0, 0.3, 0.0,
        0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.2, 0.0,
        0.0, 0.0, 0.0, 0.0
    }), 4, 4).t(), linear_step_fcn));
  this->action_tree->insert("stair_up_para")->insert("NENWSWSE")->
    insert("fwdstepB", new ActionState(reshape(mat({
        0.0, 0.0, 0.7, 0.0,
        0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, -0.2, 0.0,
        0.0, 0.0, 0.0, 0.0
    }), 4, 4).t(), linear_step_fcn));
  this->action_tree->insert("stair_up_para")->insert("NENWSESW")->
    insert("fwdstepA", new ActionState(reshape(mat({
        0.0, 0.3, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0,
        0.0, 1.2, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0
    }), 4, 4).t(), linear_step_fcn));
  this->action_tree->insert("stair_up_para")->insert("NENWSESW")->
    insert("fwdstepB", new ActionState(reshape(mat({
        0.0, 0.7, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0,
        0.0, -0.2, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0
    }), 4, 4).t(), linear_step_fcn));
  this->action_tree->insert("stair_up_para")->insert("NWNESESW")->
    insert("fwdstepA", new ActionState(reshape(mat({
        0.0, 0.0, 0.0, 0.3,
        0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.2,
        0.0, 0.0, 0.0, 0.0
    }), 4, 4).t(), linear_step_fcn));
  this->action_tree->insert("stair_up_para")->insert("NWNESESW")->
    insert("fwdstepB", new ActionState(reshape(mat({
        0.0, 0.0, 0.0, 0.7,
        0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, -0.2,
        0.0, 0.0, 0.0, 0.0
    }), 4, 4).t(), linear_step_fcn));
}

/** load the action space tree
 */
void Tachikoma::load_action(const char *name, const char *path) {
  // this is the better function
}

/** learn an action based on a video clip
 */
void Tachikoma::learn_action(const char *name, mat training_set) {
}
}

/** load the action space tree
 */
void Tachikoma::load_action(const char *name, const char *path) {
  // this is the better function
}

/** learn an action based on a video clip
 */
void Tachikoma::learn_action(const char *name, mat training_set) {
}

/** save all hypotheses
 */
void Tachikoma::save_actions(void) {
}

/** free all hypotheses learned
 */
void Tachikoma::forget_actions(void) {
}
