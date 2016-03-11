//
//	[Author] = Ming Tai Ha
//

#include "state.h"
#include <cstring>
#include "heuristic.h"

using namespace std;
using namespace arma;

state::state(int x, int y, state *parent) :
    x(x),
    y(y),
    parent(parent),
    g_value(-1),
    children(0) {
}

state::~state() {
}

void state::setG(int start_x, int start_y) {
	g_value = parent ? parent->g_value + 1 : 0;
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
  for (state *child : children) {
    child->clear();
    delete child;
  }
  children.clear();
}

ostream &operator<<(ostream &out, state &st) {
	out << "{" << &st << "} [x y] = [" << st.x << " " << st.y << "]\n"
    "\tCOST: [g h] = [" << st.g_value << " " << st.h_value << "]\n"
    "\tPARENT: " << st.parent << "\n"
    "\tOBSTACLE: " << (st.map(st.y, st.x) == 1) << "\n";
}
