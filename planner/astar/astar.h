//
//	[Author] = Ming Tai Ha
//

#ifndef astar_h
#define astar_h

#include <vector>
#include <armadillo>
#include "searchtree.h"

enum Ftype { F_FORWARD, F_BACKWARD };
enum Gtype { G_MIN, G_MAX };
enum Htype { H_MANHATTAN, H_EUCLIDEAN };

class AStarProp {
  public:
    AStarProp(enum Ftype f, enum Gtype g, enum Htype h, bool adaptive) :
      f(f), g(g), h(h), adaptive(adaptive) {}
    enum Ftype f;
    enum Gtype g;
    enum Htype h;
    bool adaptive;
};

class AStar {
  public:
    AStar(Map &map, arma::ivec &start, arma::ivec &goal,
      AStarProp prop = AStarProp(F_FORWARD, G_MAX, H_EUCLIDEAN, false));
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
    AStarProp prop;

};

#endif
