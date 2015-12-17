#ifndef __TACHI_ACTION_H__
#define __TACHI_ACTION_H__

// NOTE: OBSOLETE

#include <armadillo>

// Remember that we need some form of relative positioning!

typedef arma::mat (*ActionFcn)(const arma::mat &start, const arma::mat &stop, double t);

class ActionState {
  public:
    ActionState(void);
    ActionState(const arma::mat &start, const arma::mat &stop,
        ActionFcn motion, double tolerance = 1.0);
    arma::mat get_motion(const arma::mat &currPos); // VEL
    bool finished(const arma::mat &currpos);

    arma::mat startPos;
    arma::mat stopPos;
    ActionFcn motionFcn;
    double toleranceError;

  private:
    arma::mat motion;
};

// for now think of it as a tree

class ActionNode { // Piecewise functions
  public:
    ActionState *action; // either this is null or the other thing is null
    std::string name;
    arma::mat get_motion(const arma::mat &currPos); // VEL
    void next_action(void);
    void add_action(const ActionState &action);
    bool finished(const arma::vec &currPos);

    std::vector<ActionNode *> neighbors;
    std::map<std::string, int> subaction_hash; // use to directly jump to a different action
    int subaction_ind;

    ActionNode *select(const std::string name);
    ActionNode *insert(const std::string name,
        ActionState *action = NULL);
};

#endif
