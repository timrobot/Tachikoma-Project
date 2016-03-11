//
//	[Author] = Ming Tai Ha
//

#include "astar.h"
#include "searchtree.h"
#include "maze_gen.h"
#include <algorithm>
#include <iostream>
#include <vector>
#include <cassert>
#include <armadillo>

using namespace arma;
using namespace std;

/** The goal of this function is to initialize the AStar algorithm,
 *  including any data structures which you are to use in the
 *  computation of the next state
 *  @param map This is the map which you are given
 *  @param start This is the starting position of the robot
 *  @param goal This is the goal of the robot
 */
AStar::AStar(imat map, ivec &start, ivec &goal, int forward_mode, int heuristic_mode, int tie_mode) :
    fin(NULL),
    isComplete(false),
    start(start),
    goal(goal),
    map(map),
    prop(prop),
    tree(start(0), start(1), goal(0), goal(1), map) {
}

AStar::~AStar(void) {
}

/** In this function, you are to get the next state off the
 *  priority queue and then traverse to that state,
 *  computing the cost and placing the state that was traversed
 *  into the search space
 */
void AStar::compute(void) {
  assert(!pqueue.empty());
  state *choice;
  svec breaktie;

  // STEP 1: Grab a list of minimum positions from the priority queue
  if (isComplete) { // already done, don't do any more
    return;
  }
  state * s = tree.pqueue.remove();
  breaktie.push_back(s);
  while (!tree.pqueue.isEmpty()) {
    s = tree.pqueue.remove();
    if ((*s) != (*breaktie[0])) {
      tree.pqueue.insert(s);
      break;
    }
    breaktie.push_back(s);
  }

  // STEP 2: Use g_value tie breaking to choose a position from the queue,
  //         and place the rest back into the queue
  struct {
    bool operator()(state *a, state *b) {
      return a->g_value < b->g_value; // get the min value
    }
  } gMin;
  struct {
    bool operator()(state *a, state *b) {
      return a->g_value > b->g_value; // get the max value
    }
  } gMax;
  // resolve the tie
  if (this->tie_mode == G_MIN) {
    sort(breaktie.begin(), breaktie.end(), gMin);
  } else {
    sort(breaktie.begin(), breaktie.end(), gMax);
  }
  choice = breaktie[0];
  for (int i = 1; i < breaktie.size(); i++) {
    tree.pqueue.insert(breaktie[i]);
  }
  breaktie.clear(); // clear the vector for later usage

  // STEP 3: Detect if the current node is the goal node;
  //         if it is, RETURN (do not do anything)

  if (choice->x == tree.goal_x && choice->y == tree.goal_y) {
    tree.addToTree(choice);
    isComplete = 1;
    fin = choice;
  } else {
    // STEP 4: Compute the cost of the 4-connected neighborhood and
    //         add them to the priority queue if they have not been
    //         added before
    tree.addToTree(choice);
    tree.addChildren(choice);
    isComplete = 0;
  }	
}

/** Grab the entire tree of nodes from the search space
 *  @param path a vector of (x, y) tuples
 */
void AStar::decision_space(vector<ivec> &path) {
  path.clear();
  for (int i = 0; i < map.n_rows; i++) {
    for (int j = 0; j < map.n_cols; j++) {
      if (tree.closed(i, j) == 1) {
        path.push_back({j, i});
      }
    }
  }
}

/** Grab the final path of nodes from the search space
 *  to the goal
 *  @param path a vector of (x, y) tuples
 */
void AStar::final_decision(vector<ivec> &path) {
  path.clear();
  for (state *step = fin; step != NULL; step = step->parent) {
    path.push_back({ step->x, step->y });
  }	
}

/** Return whether or not the goal is impossible to reach
 *  @return true if it is impossible, false otherwise
 */
bool AStar::impossible(void) {
  return !isComplete && tree.pqueue.isEmpty();
}

/** Return whether or not the goal has been reached
 *  @return true if goal is reached, false otherwise
 */
bool AStar::complete(void) {
  return isComplete;
}
