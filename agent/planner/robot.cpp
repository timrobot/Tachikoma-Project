#include "robot.h"
#include <cassert>

Robot::Robot(ivec pos) {
  assert(pos.n_elem == 2);
  this->x = pos(0);
  this->y = pos(1);
  this->searchalgo = NULL;
}

Robot::~Robot(void) {
  if (this->searchalgo) {
    delete this->searchalgo;
    this->searchalgo = NULL;
  }
}

static imat map_mdist(int n_rows, int n_cols, ivec start, ivec goal) {
  imat mm(n_rows, n_cols);
  for (int i = 0; i < n_rows; i++) {
    for (int j = 0; j < n_cols; j++) {
      mm(i, j) = mdist(j, i, goal(0), goal(1));
    }
  }
  return mm;
}

void Robot::search(imat map, ivec &start, ivec &goal,
    int forward_mode, int heuristic_mode, int tie_mode) {
  this->goal = goal;
  this->forward_mode = forward_mode;
  set_cost(map_mdist(map.n_rows, map.n_cols, start, goal));
  if (forward_mode == F_FORWARD) {
    this->searchalgo = new AStar(map, start, goal, F_FORWARD, heuristic_mode, tie_mode);
  } else {
    this->searchalgo = new AStar(map, goal, start, F_FORWARD, heuristic_mode, tie_mode);
  }
  set_adaptive(heuristic_mode);
}

void Robot::run(void) {
  if (!searchalgo->complete() && !searchalgo->impossible()) {
    searchalgo->compute();
  }
}

bool Robot::complete(void) {
  if (!this->searchalgo) {
    return true;
  }
  return this->x == this->goal(0) &&
    this->y == this->goal(1);
}

bool Robot::stuck(void) {
  if (!this->searchalgo) {
    return false;
  }
  return this->searchalgo->impossible();
}

ivec Robot::getMotion(void) {
  vector<ivec> path;
  this->searchalgo->final_decision(path);
  if (this->forward_mode == F_FORWARD) {
    return path[path.size()-2];
  } else {
    return path[1];
  }
}



imat mod_cost(imat cost, imat interim, ivec goal) {
  vector<ivec> fringe;
  vector<int> running_cost;
  fringe.push_back(ivec({ goal(1), goal(0) }));
  running_cost.push_back(0);
  umat visited(cost.n_rows, cost.n_cols, fill::zeros);
  visited(goal(1), goal(0)) = 1;
  
  for (int i = 0; i < fringe.size(); i++) {
    ivec pt = fringe[i];
    int rcost = running_cost[i];
    cost(pt(0), pt(1)) = rcost;
    ivec y = { pt(0) - 1, pt(0), pt(0), pt(0) + 1 };
    ivec x = { pt(1), pt(1) - 1, pt(1) + 1, pt(1) };
    for (int j = 0; j < 4; j++) {
      if (0 <= x(j) && x(j) < (int)cost.n_cols &&
          0 <= y(j) && y(j) < (int)cost.n_rows &&
          visited(y(j), x(j)) == 0 &&
          interim(y(j), x(j)) > -1) {
        visited(y(j), x(j)) = 1;
        fringe.push_back(ivec({ y(j), x(j) }));
        running_cost.push_back(rcost + 1);
      }
    }
  }

  //cout << "\n\n*****************\n";
  //cout << "Cost:\n";
  //cout << cost;
  //cout << "\n\n*****************\n";

  return cost;
}

void Robot::move(ivec newpos) {
  this->x = newpos(0);
  this->y = newpos(1);
  if (this->x == newpos(0) && this->y == newpos(1)) {

  }
  if (this->searchalgo) {
    imat map = this->searchalgo->map;
    ivec start({ this->x, this->y });
    int heuristic_mode = this->searchalgo->heuristic_mode;
    int tie_mode = this->searchalgo->tie_mode;
    delete this->searchalgo;
    // switch up backward and forward
    set_cost(mod_cost(get_cost(), get_interim(), this->goal));
    if (this->forward_mode == F_FORWARD) {
      this->searchalgo = new AStar(map, start, this->goal, this->forward_mode, heuristic_mode, tie_mode);
    } else {
      this->searchalgo = new AStar(map, this->goal, start, this->forward_mode, heuristic_mode, tie_mode);
    }
  }
}
