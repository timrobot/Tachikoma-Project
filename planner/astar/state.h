//
//	[Author] = Ming Tai Ha
//

#ifndef state_h
#define state_h

#include <iostream>
#include <armadillo>

#define H_REPEATED 0
#define H_ADAPTIVE 1

class state {

	public:
	
		int x;							// Current x-coordinates
		int y;							// Current y-coordinates
		state * parent;					// Pointer to the parent state
		state * children[4];		// Vector of pointers to each child node
		double g_value;					// Cost of getting to the current state
		double h_value;					// Estimated cost of getting to the goal
										//		from the current state
    double f_value; // f_value
    arma::imat map;

		state(int x, int y, state * parent, arma::imat &map);
		~state();
		void setG(int start_x, int start_y);
		void setH(int goal_x, int goal_y, int start_x = 0, int start_y = 0);
		bool operator<(const state &other);
		bool operator>(const state &other);
		bool operator<=(const state &other);
		bool operator>=(const state &other);
		bool operator==(const state &other);
		bool operator!=(const state &other);
    void clear(void);
};

std::ostream &operator<<(std::ostream &out, state &cur_state);
void set_cost(arma::imat cost_matrix);
void set_adaptive(int heuristic_mode);
arma::imat get_cost(void);
arma::imat get_interim(void);

#endif
