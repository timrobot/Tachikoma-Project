#ifndef __TK_DSTAR_H__
#define __TK_DSTAR_H__

#include <vector>
#include <armadillo>

class DStar {
  public:
    DStar(arma::imat map, arma::ivec &goal);
    ~DStar(void);
    void compute(arma::ivec &start, std::vector<arma::ivec> &path);
    bool complete(void);
    bool impossible(void);
};

#endif
