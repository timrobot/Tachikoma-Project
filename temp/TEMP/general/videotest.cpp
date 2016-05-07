#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/features2d.hpp>
#include <iostream>
#include <armadillo>
#include "highgui.h"
#include "imgproc.h"
#include "videotest.h"

using namespace std;
using namespace cv;

void detectBear(cv::Mat &camframe, std::mutex &camlock, arma::vec & bearpos, double &bearsize, bool &bearfound) {
  const int h1 = 0;
  const int h2 = 100;
  const int s1 = 128;
  const int s2 = 255;
  const int v1 = 128;
  const int v2 = 255;
	arma::vec xbuf(10, arma::fill::zeros);
	arma::vec ybuf(10, arma::fill::zeros);
	arma::vec xcbuf(10, arma::fill::zeros);
	arma::vec ycbuf(10, arma::fill::zeros);
	int xind = 0, yind = 0;
	bool stable = false;
	VideoCapture cam(1);
	Mat img;
	while (1) {
	  cam.read(img);
	  //if (!img.data) {
	//	continue;
	 // }
		camlock.lock();
		img.copyTo(camframe);
		camlock.unlock();
		Mat hsv;
		cvtColor(img, hsv, COLOR_BGR2HSV);

		// in range masking
		Mat hsv2mask, hsv2;
		inRange(hsv, Scalar(h1, s1, v1), Scalar(h2, s2, v2), hsv2mask);
		//hsv.copyTo(hsv2, hsv2mask);

		arma::mat I;
		cvt_opencv2arma(hsv2mask, I);
		
		//blur
		//I = conv2(I, gauss2(7, 16));
		
		arma::rowvec r = arma::sum(I, 0);
		arma::colvec c = arma::sum(I, 1);
		double midx = arma::sum(r * arma::cumsum(arma::ones<arma::vec>(r.n_elem))) / arma::sum(r);
		double midy = arma::sum(c % arma::cumsum(arma::ones<arma::vec>(c.n_elem))) / arma::sum(c);
		stable = true;
		if (midx < 1 || (midx > (int)I.n_cols - 1) || arma::sum(r) == 0) {
			midx = 0;
		}
		if (midy < 1 || (midy > (int)I.n_rows - 1) || arma::sum(c) == 0) {
			midy = 0;
		}

		// remove noise
		if (midx == 0 || midy == 0) {
			stable = false;
		}
		xbuf[xind] = midx;
		ybuf[yind] = midy;
		midx = arma::mean(xbuf);
		midy = arma::mean(ybuf);
		if (sqrt(arma::var(xbuf) * arma::var(xbuf) + arma::var(ybuf) + arma::var(ybuf)) > 70.0) {
			stable = false;
		}
		
		Scalar color(255, 0, 0);
		//cout << "arma:: " << midx << ", " << midy << endl;
		//cout << "stable: " << stable << endl;
		circle(img, Point((int)midx, (int)midy), 4, color, 0);

		double covx = sqrt(arma::sum(r * arma::square(arma::cumsum(arma::ones<arma::vec>(r.n_elem)) - midx)) / arma::sum(r));
		double covy = sqrt(arma::sum(c % arma::square(arma::cumsum(arma::ones<arma::vec>(c.n_elem)) - midy)) / arma::sum(c));
		xcbuf[xind] = covx;
		ycbuf[yind] = covy;
		covx = arma::accu(xcbuf) / (int)xbuf.n_elem;
		covy = arma::accu(ycbuf) / (int)ybuf.n_elem;
		xind = (xind + 1) % (int)xbuf.n_elem;
		yind = (yind + 1) % (int)ybuf.n_elem;
		//double estw = covx * 3;
		//double esth = covy * 3;
		//blur the image to make it more accurate
		/*I = conv2(I, gauss2(7, 16));
		disp_image("convI", I);
		int minw = (int)midx;
		int maxw = (int)midx;
		for (int minw = (int)midx; minw >= 0; minw--) {
		  if (I(midy, minw) < 0.05) {
			break;
		  }
		}
		for (int maxw = (int)midx; maxw < 640; maxw++) {
		  if (I(midy, maxw) < 0.05) {
			break;
		  }
		}
		int minh = (int)midy;
		int maxh = (int)midy;
		for (int minh = (int)midy; minh >= 0; minh--) {
		  if (I(minh, midx) < 0.05) {
			break;
		  }
		}
		for (int maxh = (int)midy; maxh < 480; maxh++) {
		  if (I(maxh, midx) < 0.05) {
			break;
		  }
		}
		double covy = (double)(maxh - minh);
		double covx = (double)(maxw - minw);*/
		covx = (covx) * 3.3;
		covy = (covy) * 3.3;
		bearpos = arma::vec({ midx - 320, 479 - (midy - 240) });
		bearsize = 533 / (covx) * 12;
		bearfound = stable;

		rectangle(img, Rect(midx-covx/2,midy-covy/2,covx,covy), Scalar(0, 0, 255), 2);

		imshow("oldhsv", hsv);
		imshow("newhsv", hsv2mask);
		imshow("img", img);
		waitKey(30);
	}
}
#if 0

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
	VideoCapture cam(0);

	arma::vec xbuf(10, arma::fill::zeros);
	arma::vec ybuf(10, arma::fill::zeros);
	int xind = 0, yind = 0;
	bool stable = false;

	while (1) {
		Mat img;
		cam.read(img);
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
		stable = true;
		if (midx < 1 || (midx > (int)I.n_cols - 1) || arma::sum(r) == 0) {
			midx = 0;
		}
		if (midy < 1 || (midy > (int)I.n_rows - 1) || arma::sum(c) == 0) {
			midy = 0;
		}

		// remove noise
		if (midx == 0 || midy == 0) {
			stable = false;
		}
		xbuf[xind] = midx;
		ybuf[yind] = midy;
		xind = (xind + 1) % (int)xbuf.n_elem;
		yind = (yind + 1) % (int)ybuf.n_elem;
		midx = arma::mean(xbuf);
		midy = arma::mean(ybuf);
		if (sqrt(arma::var(xbuf) * arma::var(xbuf) + arma::var(ybuf) + arma::var(ybuf)) > 70.0) {
			stable = false;
		}
		
		Scalar color(255, 0, 0);
		cout << "arma:: " << midx << ", " << midy << endl;
		cout << "stable: " << stable << endl;
		circle(img, Point((int)midx, (int)midy), 4, color, 0);

		double covx = sqrt(arma::sum(r * arma::square(arma::cumsum(arma::ones<arma::vec>(r.n_elem)) - midx)) / arma::sum(r));
		double covy = sqrt(arma::sum(c % arma::square(arma::cumsum(arma::ones<arma::vec>(c.n_elem)) - midy)) / arma::sum(c));
		double estw = covx * 3;
		double esth = covy * 3;

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
		if (waitKey(30) & 0xff == 'Q') {
			break;
		}
	}
	return 1;
}
#endif
