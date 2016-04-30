#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>
#include <armadillo>
#include "highgui.h"

using namespace std;
using namespace cv;

void detectcan(cv::Mat &frame) {
	int h1 = 100;
	int h2 = 179;
	int s1 = 125;
	int s2 = 255;
	int v1 = 50;
	int v2 = 170;
	cv::Mat hsv;
	cv::cvtColor(frame, hsv, COLOR_BGR2HSV);
	cv::Mat hsv2mask, hsv2;
	cv::inRange(hsv, cv::Scalar(h1, s1, v1), cv::Scalar(h2, s2, v2), hsv2mask);

	arma::mat I;
	cvt_opencv2arma(hsv2mask, I);
	if (arma::accu(I) < 10.0) {
		continue;
	}
	arma::rowvec r = arma::sum(I, 0);
	arma::colvec c = arma::sum(I, 1);
	double midx = arma::sum(r * arma::cumsum(arma::ones<arma::vec>(r.n_elem))) / arma::sum(r);
	double midy = arma::sum(c % arma::cumsum(arma::ones<arma::vec>(c.n_elem))) / arma::sum(c);

	double covx = sqrt(arma::sum(r * arma::square(arma::cumsum(arma::ones<arma::vec>(r.n_elem)) - midx)) / arma::sum(r));
	//double estw = covx;
	//double esth = estw / 0.5;
	//estw *= 3; // strange constants
	//esth *= 3;

	cokecanpos = vec({ midx, midy });
	cokecanwidth = covx * 3;
}

int main(int argc, char *argv[]) {
	if (argc != 8) {
		cout << "usage: ./test img h1 h2 s1 s2 v1 v2\n";
		return 1;
	}
	int h1 = atoi(argv[2]);
	int h2 = atoi(argv[3]);
	int s1 = atoi(argv[4]);
	int s2 = atoi(argv[5]);
	int v1 = atoi(argv[6]);
	int v2 = atoi(argv[7]);

	// convert to hsv
	Mat img = imread(argv[1]);
	Mat hsv;
	cvtColor(img, hsv, COLOR_BGR2HSV);

	// in range masking
	Mat hsv2mask, hsv2;
	inRange(hsv, Scalar(h1, s1, v1), Scalar(h2, s2, v2), hsv2mask);
	//hsv.copyTo(hsv2, hsv2mask);

	arma::mat I;
	cvt_opencv2arma(hsv2mask, I);
	arma::rowvec r = arma::sum(I, 0);
	arma::colvec c = arma::sum(I, 1);
	double midx = arma::sum(r * arma::cumsum(arma::ones<arma::vec>(r.n_elem))) / arma::sum(r);
	double midy = arma::sum(c % arma::cumsum(arma::ones<arma::vec>(c.n_elem))) / arma::sum(c);
	Vec3b color(255, 0, 0);
	circle(img, Point((int)midx, (int)midy), 4, color, 1);

	double covx = sqrt(arma::sum(r * arma::square(arma::cumsum(arma::ones<arma::vec>(r.n_elem)) - midx)) / arma::sum(r));
	double estw = covx;
	double esth = estw / 0.5;
	estw *= 3; // strange constants
	esth *= 3;

	rectangle(img, Rect(midx-estw/2,midy-esth/2,estw,esth), Scalar(0, 0, 255), 2);

	// use histogram binning to get the center

	// create blob params
	/*SimpleBlobDetector::Params params;

	// Change thresholds
	params.minThreshold = 10;
	params.maxThreshold = 200;

	// Filter by Area.
	params.filterByArea = true;
	params.minArea = 1500;

	// Filter by Circularity
	params.filterByCircularity = true;
	params.minCircularity = 0.1;

	// Filter by Convexity
	params.filterByConvexity = true;
	params.minConvexity = 0.87;

	// Filter by Inertia
	params.filterByInertia = true;
	params.minInertiaRatio = 0.01;

	Ptr<SimpleBlobDetector> blob = SimpleBlobDetector::create(params);
	vector<KeyPoint> kp;
	Mat des;
	Mat hsvd = hsv2mask * -0.75 + 255;
	blob->detect(hsvd, kp);

	cout << "found " << kp.size() << " matches\n";

	Mat kpimg;
	drawKeypoints(img, kp, kpimg, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);*/

	imshow("oldhsv", hsv);
	imshow("newhsv", hsv2mask);
	imshow("img", img);
	waitKey(0);
	return 1;
}
