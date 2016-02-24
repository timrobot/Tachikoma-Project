#include "mapcheck.h"
#include <iostream>
#include <cassert>

using namespace arma;
using namespace std;

#define TEST_bfscheck_cpp 0

// use bfs to validate the map (to check if there is a path from the
// start state (x1, y1) to the goal state (x2, y2)

// This function returns true when there is a path between the given
// start states and goal state. If no such path exists, then return
// false

bool isValidMap(imat map, int x1, int y1, int x2, int y2) {
  assert(0 <= x1 && x1 < (int)map.n_cols); 				// Check if x1 is within the maze
  assert(0 <= y1 && y1 < (int)map.n_rows);				// Check if y1 is within the maze
  assert(0 <= x2 && x2 < (int)map.n_cols);				// Check if x2 is within the maze
  assert(0 <= y2 && y2 < (int)map.n_rows);				// Check if y2 is within the maze
  // flood the thingy with bfs
  if (map(y1, x1) == 1 || map(y2, x2) == 1) {
    return false;
  }

														// Declaring a vector of integers vectors to store the location of the nodes,
  vector<ivec> fringe;									// 		where the (x,y) coordinates of the nodes is an integer vector
  fringe.push_back(ivec({ y1, x1 }));					// Add the starting location to the fringe
  umat visited(map.n_rows, map.n_cols, fill::zeros);	// Create an unsigned zeros matrix with the same dimensions as the matrix map
  visited(y1, x1) = 1;									// Setting the first location to 1 to indicate that it is visited
  
  														// Looping over all the nodes in the fringe. The size of the fringe increases
  for (int i = 0; i < fringe.size(); i++) {				// 		as the BFS discovers more nodes and adds them to the fringe
    ivec pt = fringe[i];									// Declaring an ivec vector to store the (x,y) of currently observed fringe node
    if (pt(0) == y2 && pt(1) == x2) {						// If the current fringe node is the goal state, then return true
      return true;
    }
    // add the neighbors
    ivec y = { pt(0) - 1, pt(0), pt(0), pt(0) + 1 };	// Declare a vector of the y-coordinates of the nodes adjacent to the current
														// fringe node
    ivec x = { pt(1), pt(1) - 1, pt(1) + 1, pt(1) };	// Declare a vector of the x-coordinates of the nodes adjacent to the current
														// fringe node
    for (int j = 0; j < 4; j++) {						// Looping through the 4 adjacent nodesoptions
      if (0 <= x(j) && x(j) < (int)map.n_cols &&			// Checking if the x-coordinate of the adjacent node is in the matrix
          0 <= y(j) && y(j) < (int)map.n_rows &&			// Checking if the y-coordinate of the adjacent node is in the matrix
          visited(y(j), x(j)) == 0 &&						// Checking whether the node has been visited or not (Not visited == 0)
          map(y(j), x(j)) == 0) {							// Checking whether the adjacent node is a wall or not (Wall == 1)
        visited(y(j), x(j)) = 1;								// Mark the location as visited
        fringe.push_back(ivec({ y(j), x(j) }));					// Adding the adjacent node to the fringe
      }
    }
  }
  return false;											// Return false. This occurs when we have exhausted all the nodes in the map
  														// 		but have not found a path from the start state to the end state
}

#if TEST_bfscheck_cpp

int main() {
  imat map1 = reshape(imat({
      0, 1, 0,
      0, 0, 0,
      0, 1, 0 }), 3, 3).t();
  imat map2 = reshape(imat({
      0, 1, 0,
      0, 1, 0,
      0, 1, 0 }), 3, 3).t();
  cout << isValidMap(map1, 0, 0, 2, 2) << endl;
  cout << isValidMap(map2, 0, 0, 2, 2) << endl;
  return 0;
}

#endif
