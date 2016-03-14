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

void DStar::compute(arma::ivec &start, std::vector<arma::ivec> &path) {
  this->isComplete = false;
  this->isImpossible = false;

  heap<ivec> opened;
  // this will require a "k"
}
