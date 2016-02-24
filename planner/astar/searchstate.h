//
//	[Author] = Ming Tai Ha
//

#ifndef searchstate_h
#define searchstate_h

#include "heuristic.h"
#include <iostream>
#include <vector>
#include <new>
#include <armadillo>


using namespace std;
using namespace arma;


class state {

	public:
	
		int x;							// Current x-coordinates
		int y;							// Current y-coordinates
		state * parent;					// Pointer to the parent state
		vector<state *> children;		// Vector of pointers to each child node
		double g_value;					// Cost of getting to the current state
		double h_value;					// Estimated cost of getting to the goal
										//		from the current state

		state(int x, int y, state * parent);
		~state();
		void setG(int start_x, int start_y);
		void setH(int goal_x, int goal_y);
		bool operator<(const state &other);
		bool operator>(const state &other);
		bool operator<=(const state &other);
		bool operator>=(const state &other);
		bool operator==(const state &other);
		bool operator!=(const state &other);
};


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

		vector<state *> queue;
};



class searchtree {

	public:
		
		searchtree();
		searchtree(int start_x, int start_y, int goal_x, int goal_y, imat map);
		void init(int start_x, int start_y, int goal_x, int goal_y, imat map);
		~searchtree();
		void addChildren(state * cur, heap_n &pqueue, imat &visited, imat &queued, imat &map,
							int start_x, int start_y, int goal_x, int goal_y);
		void addToTree(state * node, imat &visited);

		imat map;
		imat visited;
		imat queued;
		int start_x;
		int start_y;
		int goal_x;
		int goal_y;
		state * root;
		heap_n pqueue;

};

#endif
