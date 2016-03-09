#include "batch_astar.h"
#include "searchtree.h"
#include <algorithm>
#include <iostream>
#include <vector>
#include <cassert>
#include <armadilllo>

using namespace arma;
using namespace std;



/*
*
*/
Batch_AStar( imat *map, ivec &goal ) {
	this->map = map;
	this->goal = goal;
}

/*
*
*/
AStar::~AStar( void ) {
}

/*
*
*/
bool heap_traversal( heap_n *openned, ivec position ) {

	for( ivec pos : openned->queue ) {
		if(pos ==  position ){
			return true;
		}
	}
	return false;
}

/*
*
*/
vector<successor_t> get_successor( ivec position ) {
	int x = position[0];
	int y = position[1];
	successor_t s;
	vector<succussor_t> successors;



	if ( map(x+1,y) ) {
		s.position[0] = x+1;
		s.position[1] = y;
		s.cost = successor_cost();
		successor.pushback(s);
	}

	if ( map(x-1,y) ) {
		s.position[0] = x-1;
		s.position[1] = y;
		s.cost = successor_cost();
		successor.pushback(s);
	}

	if ( map(x,y+1) ) {
		s.position[0] = x;
		s.position[1] = y+1;
		s.cost = successor_cost();
		successor.pushback(s);
	}

	if ( map(x, y-1) ) {	
		s.position[0] = x;
		s.position[1] = y-1;
		s.cost = successor_cost();
		successor.pushback(s);
	}

	return successors;
}


vector<ivec> trace(icube parent, ivec position) {
	vector<ivec> path;
	ivec current = goal;

	while (current != position) {
		int x = parent(current[0], current[1], 0);
		int y = parent(current[0], current[1], 1);
		path.pushback(current);
		current = ivec({x,y});
	}

	path.pushback(position)
	return path;
}

/*
*
*/
vector<ivec> AStar::compute( ivec position ) {
	heap_n openned;
	imat   closed;
	icube  parent;
	bool   inClosed;
	bool   inOpenned;
	double pCost;

	ivec start = position;

	openned.push( position, 0 );

	while( !openned.isEmpty )
	{
		position = openned.pop( &pCost );
		closed( position[0], position[1] ) = 1;
		if ( posisiton == goal ) {
			return trace( parent, start );
		} else {
			vector<successor_t> successors = get_successor( position );

			for (successor_t s : successor ) {
				if ( closed(s.position[0], s.position[1]) == 1  ) {
					inClosed = true;			
				} else if ( heap_traversal(openned, s.position) == true ) {
					inOpenned = true;		
				} else {
					openned.push(s.position, (s.cost + pCost));
					parent(s.position[0], s.position[1], 0) = position[0];
					parent(s.position[0], s.position[1], 1) = position[1];
				}	
			}		
		}
	}	
}

