#ifndef CHILI_LANDMARKS_H
#define CHILI_LANDMARKS_H

#include <opencv2/core/core.hpp>
#include <mutex>
#include <armadillo>

class chili_landmarks
{
	public:
		chili_landmarks();
		~chili_landmarks();
		void update(cv::Mat &cameraFrame, std::mutex &camframelock,
		arma::vec & bearpos, double &bearsize, bool &bearfound
			);

		double tags [1024][4];
		// tags[id][0] -> is being detected
		// tags[id][1] -> x distance
		// tags[id][1] -> y distance
};

#endif
