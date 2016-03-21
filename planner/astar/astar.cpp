//
//	[Authors] = Ming Tai Ha
//              Jon Risinger
//              Timothy Yong
//

#include "astar.h"
#include "maze_gen.h"
#include "heap.h"
#include "heap.cpp"
#include <algorithm>
#include <iostream>
#include <vector>
#include <cassert>
#include <armadillo>

using namespace arma;
using namespace std;

static vector<MotionAction> getNextAction(double x, double y, MotionAction currAction);

/** The goal of this function is to initialize the AStar algorithm,
 *  including any data structures which you are to use in the
 *  computation of the next state
 *  @param map This is the map which you are given
 *  @param goal This is the goal of the robot
 */
AStar::AStar(imat map, ivec &goal, AStarProp prop) :
    isComplete(false),
    isImpossible(false),
    goal(goal),
    prop(prop) {
  this->map = map.t();
  assert(0 <= goal(0) && goal(0) < (int)this->map.n_rows &&
         0 <= goal(1) && goal(1) < (int)this->map.n_cols);
}

AStar::~AStar(void) {
}

/** In this function, you are to get the next state off the
 *  priority queue and then traverse to that state,
 *  computing the cost and placing the state that was traversed
 *  into the search space
 */
void AStar::compute(ivec &start, vector<ivec> &path) {
  this->isComplete = false;
  this->isImpossible = false;

  heap<MotionAction> opened;
  opened.push(start, sum(abs(start - goal)));
  // create a matrix of parents that have been closed
  imat closed(this->map.n_rows, this->map.n_cols, fill::zeros);
  imat stategrid(this->map.n_rows, this->map.n_cols, fill::zeros);
  imat costs(this->map.n_rows, this->map.n_cols, fill::zeros);

  // after pushing the initial state, start trying to get the next state
  while (!opened.empty()) {
    // grab a state
    MotionAction next_state = opened.pop();
    int x = next_state.x;
    int y = next_state.y;

    closed(x, y) = true;
    int currcost = costs(x, y);
    // if this state is the goal state, then return the path
    if (x == this->goal(0) && y == this->goal(1)) {
      path.clear();
      while (x != start(0) || y != start(1)) {
        path.push_back(next_state);
        imat actions = reshape(imat({
          0, 0, 1, -1,
          -1, 1, 0, 0
        }), 4, 2).t();
        ivec action = actions.col(stategrid(x, y) - 1);
        next_state += action;
        x = next_state(0);
        y = next_state(1);
      }
      path.push_back(next_state);
      reverse(path.begin(), path.end());
      this->isComplete = true;
      return;
    }
    // otherwise try to find new neighbors and add them in
    imat neighbor4 = reshape(imat({
      0, 0, -1, 1,
      1, -1, 0, 0
    }), 4, 2).t();
    for (int j = 0; j < (int)neighbor4.n_cols; j++) {
      int x_ = x + neighbor4(0, j);
      int y_ = y + neighbor4(1, j);
      if (x_ < 0 || x_ >= (int)stategrid.n_cols ||
          y_ < 0 || y_ >= (int)stategrid.n_rows ||
          this->map(x_, y_) > 0.5) {
        continue;
      }
      if (!stategrid(x_, y_) && !closed(x_, y_)) {
        stategrid(x_, y_) = j + 1;
        costs(x_, y_) = currcost + 1; // assume 1 cost
        opened.push(ivec({ x_, y_ }), costs(x_, y_) +
          sum(abs(ivec({ x_, y_ }) - this->goal))); // manhattan distance
      }
    }
  }
  this->isImpossible = true;
}

/** Return whether or not the goal is impossible to reach
 *  @return true if it is impossible, false otherwise
 */
bool AStar::impossible(void) {
  return this->isImpossible;
}

/** Return whether or not the goal has been reached
 *  @return true if goal is reached, false otherwise
 */
bool AStar::complete(void) {
  return this->isComplete;
}

/** Return a vector of possible actions at this particular action
 *  @param x x position
 *  @param y y position
 *  @param currAction the current action of the robot
 */
static vector<MotionAction> getNextAction(double x, double y, MotionAction currAction, imat &map) {
  imat neighbor4 = reshape(imat({
      0, 0, -1, 1,
      1, -1, 0, 0
    }), 4, 2).t();
  ivec neighborActions = { MOVE_FORWARD, MOVE_BACKWARD, MOVE_LEFT, MOVE_RIGHT };
  vector<MotionAction> actionlist;
  for (int i = 0; i < 4; i++) {
    MotionAction action(x + neighbor4(0, i), y + neighbor4(1, i));
    // check feasibility of the action
    if (action.x >= map.n_cols || action.y >= map.n_rows || map(action.x, action.y) > 0.5) {
      continue;
    }
    action.cost = 1;
    action.gcost = currAction.cost + action.cost;
    action.id = neighborActions[i];
    actionlist.push_back(action);
  }
  return actionlist;
}
