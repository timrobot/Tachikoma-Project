#ifndef __TK_SLAMBOT_H__
#define __TK_SLAMBOT_H__

#include <armadillo>
#include "baserobot.h"

class SlamBot : public BaseRobot {
  public:
    SlamBot(void);
    ~SlamBot(void);
    int numconnected(void);
    void send(const arma::vec &motion);
    arma::vec recv(void);
    void reset(void);

  private:
    arma::vec prev_motion;
    arma::vec motion_const;
};

#endif
