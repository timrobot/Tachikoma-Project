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

ostream &operator<<(ostream &out, state &cur_state);

class heap_n {

	public:

		heap_n();
		~heap_n();
		void swap(int a, int b);
		void siftup();
		void siftdown();
		void insert(state * item);
		state * remove();
		bool isEmpty();

		svec queue;
};

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
		state * root;
		heap_n pqueue;

};

#endif
