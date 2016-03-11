//
//	[Author] = Ming Tai Ha
//

#ifndef searchtree_h
#define searchtree_h

#include "heuristic.h"
#include <iostream>
#include <vector>
#include "svec.h"
#include <armadillo>

using namespace std;
using namespace arma;

class searchtree {
	public:
		searchtree();
		searchtree(int start_x, int start_y, int goal_x, int goal_y, imat &map);
		void init(int start_x, int start_y, int goal_x, int goal_y, imat &map);
		~searchtree();
		void addChildren(state * cur);
		void addToTree(state * node);

		imat map;
		imat opened;
		imat closed;
		int start_x;
		int start_y;
		int goal_x;
		int goal_y;
		state *root;
		heap pqueue;
};

#endif
