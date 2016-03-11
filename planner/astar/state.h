//
//	[Author] = Ming Tai Ha
//

#ifndef state_h
#define state_h

#include <iostream>

class state {
	public:
		int x;
		int y;
		state *parent;
    svec children;
		double g_value;

		state(int x, int y, state *parent);
		~state();
    void setG(int start_x, int start_y);
		bool operator<(const state &other);
		bool operator>(const state &other);
		bool operator<=(const state &other);
		bool operator>=(const state &other);
		bool operator==(const state &other);
		bool operator!=(const state &other);
    void clear(void);
};

std::ostream &operator<<(std::ostream &out, state &cur_state);

#endif
