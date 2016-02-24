//
//	[Author] = Ming Tai Ha
//

#ifndef astar_h
#define astar_h

#include <vector>
#include <armadillo>
#include "searchtree.h"

#define G_MIN 0
#define G_MAX 1
#define F_FORWARD 0
#define F_BACKWARD 1

class AStar {
  public:
    AStar(arma::imat map, arma::ivec &start, arma::ivec &goal,
        int forward_mode = F_FORWARD,
        int heuristic_mode = H_REPEATED,
        int tie_mode = G_MAX);
    ~AStar(void);
    void compute(void);
    void decision_space(std::vector<arma::ivec> &path);
    void final_decision(std::vector<arma::ivec> &path);
    bool complete(void);
    bool impossible(void);

    arma::imat map;
    arma::ivec start;
    arma::ivec goal;

    // stuff for the decision making capability
    searchtree tree;
	  state * fin;
	  bool isComplete;

    // flags
    int forward_mode;
    int heuristic_mode;
    int tie_mode;
};

#endif
