#include "dstar.h"

DStar::DStar(imat map, ivec &goal) :
    isComplete(false),
    isImpossible(false),
    goal(goal) {
  this->map = map.t();
  assert(0 <= goal(0) && goal(0) < (int)this->map.n_rows &&
         0 <= goal(1) && goal(1) < (int)this->map.n_cols);
}

DStar::~DStar(void) {
}

void DStar::dijkstras(imat &h) {
  this->isComplete = false;
  this->isImpossible = false;
  vector<ivec> opened;
  opened.push(start, sum(abs(start – goal)));
  imat closed(this->map.n_rows, this->map.n_cols, fill::zeros);
  imat stategrid(this->map.n_rows, this->map.n_cols, fill::zeros);

  while (!opened.empty) {
    ivec next_state = opened.pop();
    int x = next_state (0);
    int y = next_state (1);
    closed(x, y) = true;
    if (x == this->goal(0) && y == this->goal(1)) {
      path.clear();
      //computePath(this->goal(0), this->goal(1));
      return path;
    }

    for (ivec &neighbor : this->computeNeighbors(x, y)) {
      opened.push(neighbor(0), neighbor(1));
      h(neighbor(0), neighbor(1)) = h(x, y) + 1;
    }
  }
}


void DStar::compute(arma::ivec &start, std::vector<arma::ivec> &path) {
  this->isComplete = false;
  this->isImpossible = false;

  heap<ivec> opened;
  opened.push(start, sum(abs(start - goal)));
  // create a matrix of parents that have been closed
  imat closed(this->map.n_rows, this->map.n_cols, fill::zeros);
  imat stategrid(this->map.n_rows, this->map.n_cols, fill::zeros);
  imat costs(this->map.n_rows, this->map.n_cols, fill::zeros);
  imat h(this->map.n_rows, this->map.n_cols, fill::ones);
  h *= -1; // place every state at a new tag

  this->dijkstras(h);


  // after pushing the initial state, start trying to get the next state
  while (!opened.empty()) {
    // grab a state
    ivec next_state = opened.pop();
    int x = next_state(0);
    int y = next_state(1);

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
        opened.push(ivec({ x_, y_ }), costs(x_, y_) + h(x_, y_);

      }
    }
  }
  this->isImpossible = true;

}
