#ifndef __TK_pfilter_H__
#define __TK_pfilter_H__

#include "sim_map.h"
#include "sim_robot.h"
#include "sim_landmark.h"
#include <vector>
#include <armadillo>
#include <SDL2/SDL.h>

class pfilter {
	public:
		pfilter(void);
		pfilter(int nparticles, sim_map *map, std::vector<sim_landmark> &landmarks, double x, double y, double t, double initial_sigma);
		~pfilter(void);
		void move(double vx, double vy, double w);
		void observe(arma::mat observations);
		void predict(arma::vec &mu, arma::mat &sigma);
		void set_noise(double vs, double ws);
		void set_size(double r);
		void blit(arma::cube &screen, int mux, int muy);

		std::vector<sim_robot> particles;
		sim_map *map;

	private:
		void weigh(arma::mat &observations);
		void resample(void);

		double vs;
		double ws;
		arma::vec health;
		std::vector<sim_landmark> landmarks;
};

#endif
