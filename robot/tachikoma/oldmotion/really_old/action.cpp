#include <math.h>
#include "action.h"

// NOTE: OBSOLUTE

using namespace arma;

static double mag(const vec &v);

/** Constructor
 */
ActionState::ActionState(void) {
  this->startPos = vec(4, fill::zeros);
  this->stopPos = vec(4, fill::zeros);
  this->motionFcn = (ActionFcn)NULL;
  this->toleranceError = 0.0;
}

/** Constructor
 *  @param start
 *    the starting point
 *  @param stop
 *    the stopping point
 *  @param motion
 *    the motion function
 *  @param tolerance
 *    the tolerance for declaring a state to be finished
 */
ActionState::ActionState(const vec &v,
    ActionFcn motion, double tolerance) {
  this->startPos = zeros<vec>(v.n_elem);
  this->stopPos = v;
  this->motionFcn = motion;
  this->toleranceError = tolerance;
}

/** Return the intrepolated motion vector from an approxiamation of 1024 samples
 *  @param currPos
 *    the current position of the vector
 *  @return the weighted interpolation of the error and approxiamation derivative
 */
vec ActionState::get_motion_vector(const vec &currPos) {
  // Use binary approxiamation method to determine the closest point to the function
  double begTime, midTime, endTime;
  vec begVec, midVec, endVec;
  vec diff1, diff2;
  double weight1, weight2;
  int i;

  begTime = 0.0;
  endTime = 1.0; // this will always be the end time
  begVec = this->startPos;
  endVec = this->stopPos;
  weight1 = 0.4;
  weight2 = 0.6;

  // 10 iterations, 2 ^ 10 samples traversed, 1024 samples traversed
  for (i = 0; i < 10; i++) {
    midTime = (begTime + endTime) / 2;
    midVec = this->motionFcn(this->startPos, this->stopPos, midTime);
    diff1 = currPos - begVec;
    diff2 = currPos - endVec;
    if (mag(diff1) > mag(diff2)) {
      begVec = midVec;
      begTime = midTime;
    } else {
      endVec = midVec;
      endTime = midTime;
    }
  }

  // do a weighted calculation for the motion vector
  diff1 = normalise(endVec - currPos);
  diff2 = normalise(endVec - begVec);
  return diff1 * weight1 + diff2 * weight2;
}

/** Returns whether or not the current action state has finished
 *  @param currPos
 *    the current position of the vector
 *  @return true if within tolerance, else false
 */
bool ActionState::finished(const vec &currPos) {
  return mag(currPos - this->stopPos) < this->toleranceError;
}

/** Return the motion vector of the piecewise motion sequence
 *  @param currPos
 *    the current position of the vector
 *  @return piecewise motion vector
 */
vec ActionSequence::get_motion_vector(const vec &currPos) {
  if (this->sequence[this->curr_action].finished(currPos)) {
    this->curr_action = (this->curr_action + 1) % this->sequence.size();
  }
  return this->sequence[this->curr_action].get_motion_vector(currPos);
}

/** Returns wheter or not the current action state has finished
 *  @param currPos
 *    the current position of the vector
 *  @return true if finished, else false
 */
bool ActionSequence::finished(const vec &currPos) {
  return this->sequence[this->curr_action].finished(currPos);
}

/** Proceeds to the next action in the sequence
 */
void ActionSequence::next_action(void) {
  this->curr_action++;
  this->curr_action %= this->sequence.size();
}

/** Adds an action to the current action sequence
 *  @param action
 *    the action to be added to the sequence
 */
void ActionSequence::add_action(const ActionState &action) {
  this->sequence.push_back(action);
}

/** Return the magnitude of a vector
 *  @param v
 *    the vector
 *  @return the magnitude
 */
static double mag(const vec &v) {
  return sqrt(dot(v, v));
}
