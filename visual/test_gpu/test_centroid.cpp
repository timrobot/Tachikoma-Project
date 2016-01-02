#include "gpu_util.h"
#include "gcube.h"
#include "highgui.h"
#include "imgproc.h"
#include "centroid.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace std;

int main(int argc, char *argv[]) {
  gcube I(15, 20, 1, gfill::ones);
  double x, y;
  gpu_centroid(I, x, y);
  cout << x << ", " << y << endl;
  return 0;
}
