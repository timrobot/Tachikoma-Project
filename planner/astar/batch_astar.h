#ifndef batch_astar_h
#define batch_aster_h

#include <vector>
#include <armadillo>
#include "searchtree.h"

#define G_MIN 0
#define G_MAX 1
#define F_FORWARD 0
#define F_BACKAWARD 1

struct successor_t {
	ivec position;
	double cost;
}

class Batch_AStar {
	public: 
		AStar( arma::imat *map, arma::ivec &goal );
		~Astar(void);
		std::vector<arma::ivec> compute( arma::ivec position );
		auto successor_cost();
		std::vector<succcessor_t> get_succesor( arma::ivec position );
		std::vector<arma::ivec> trace( arma::icube parent, arma::ivec position );		

		arma::imat *map;
		arma::ivec goal;
};

#endif

