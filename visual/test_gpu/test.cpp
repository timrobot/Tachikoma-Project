#include "highgui.h"
#include "gcube.h"
#include "gpu_util.h"
#include <iostream>

using namespace arma;
using namespace std;

int main() {
  //imread
//  cube I = load_image("smallimage.bmp"); // red, green, blue
  cube I = load_image("butterfly.jpg");

  mat red = I.slice(0);
  mat green = I.slice(1);
  mat blue = I.slice(2);

  mat gray = 0.3 * red + 0.6 * green + 0.1 * blue; // industry standard
  mat blurred = weight_avg(gray);

  // (hand wavy) derivative
  mat e = gray - blurred;


  e /= e.max();
  disp_image("edges", e);
  disp_wait();

  return 0;
}
