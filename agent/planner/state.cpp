//
//	[Author] = Ming Tai Ha
//

#include "state.h"
#include <cstring>
#include "heuristic.h"

using namespace std;
using namespace arma;

static imat gd_value;
static imat interim;
static int adaptive;
static bool adaptive_isset;

state::state(int x, int y, state * parent, imat &map) {
	this->x = x;
	this->y = y;
	g_value = -1;
	h_value = -1;
  f_value = g_value + h_value;
	this->parent = parent;
  this->map = map;
  memset(&this->children, 0, sizeof(state *) * 4); // reset the children
}

state::~state() {
}

void state::setG(int start_x, int start_y) {
	g_value = (this->parent == NULL) ? 0 : this->parent->g_value + 1;
  interim(y,x) = g_value;
  f_value = g_value + h_value;
}

void state::setH(int goal_x, int goal_y, int start_x, int start_y) {
  if (adaptive == H_REPEATED || !adaptive_isset) {
   	h_value = mdist(this->x, this->y, goal_x, goal_y);
  } else if (adaptive == H_ADAPTIVE) {
    h_value = gd_value(y,x);
  }
  if (adaptive == H_ADAPTIVE && !adaptive_isset && h_value == 0) {
    adaptive_isset = true;
  }
  f_value = g_value + h_value;
}

bool state::operator<(const state &other) {
	return this->f_value < other.f_value;
}

bool state::operator>(const state &other) {
	return this->f_value > other.f_value;
}

bool state::operator<=(const state &other) {
	return this->f_value <= other.f_value;
}

bool state::operator>=(const state &other) {
	return this->f_value >= other.f_value;
}

bool state::operator==(const state &other) {
	return this->f_value == other.f_value;
}

bool state::operator!=(const state &other) {
	return this->f_value != other.f_value;
}

void state::clear(void) {
  for (int i = 0; i < 4; i++) {
    if (this->children[i]) {
      this->children[i]->clear();
      delete this->children[i];
    }
  }
}

void set_cost(imat gd_default) {
  gd_value = gd_default;
  interim = ones<imat>(gd_value.n_rows, gd_value.n_cols) * -1;
  adaptive_isset = false;
}
void set_adaptive(int heuristic_mode) { // hack
  adaptive = heuristic_mode;
}
imat get_cost(void) {
  return gd_value;
}
imat get_interim(void) {
  return interim;
}

ostream &operator<<(ostream &out, state &st) {
	out << "{" << &st << "} [x y] = [" << st.x << " " << st.y << "]\n"
    "\tCOST: [g h] = [" << st.g_value << " " << st.h_value << "]\n"
    "\tPARENT: " << st.parent << "\n"
    "\tOBSTACLE: " << (st.map(st.y, st.x) == 1) << "\n";
}
