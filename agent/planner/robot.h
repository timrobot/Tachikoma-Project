#ifndef robot_h
#define robot_h

#include "astar.h"

class Robot {
  public:
    Robot(arma::ivec pos);
    ~Robot(void);
    void search(arma::imat map, arma::ivec &start, arma::ivec &goal,
        int forward_mode = F_FORWARD, int heuristic_mode = H_REPEATED, int tie_mode = G_MAX);
    void run(void);
    bool complete(void); // upper bound for intelligence
    bool stuck(void); // lower bound for intelligence
    void move(arma::ivec newpos);
    arma::ivec getMotion(void);

    int x;
    int y;
    arma::ivec goal;
    AStar *searchalgo;
    bool forward_mode;
};

#endif
