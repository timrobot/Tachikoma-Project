#ifndef videotest_h
#define videotest_h

#include <mutex>
#include <armadillo>
#include <opencv2/core/core.hpp>
void detectBear(cv::Mat &camframe, std::mutex &camlock, arma::vec & bearpos, double &bearsize, bool &bearfound);

#endif
