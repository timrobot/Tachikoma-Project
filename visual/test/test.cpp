#include "highgui.h"
#include <armadillo>
#include <iostream>

using namespace arma;
using namespace std;

mat weight_avg(mat I) {
  mat coeff = reshape(mat({
    1, 2, 1,
    2, 4, 2,
    1, 2, 1
  }), 3, 3).t();
  coeff /= 16;

  // put the image inside of a zeros matrix (pad it with 0's on the borders)
  mat G = I;
  I = zeros<mat>(G.n_rows + 2, G.n_cols + 2);
  I(span(1, G.n_rows), span(1, G.n_cols)) = G;

  mat H(G.n_rows, G.n_cols);
  for (int i = 0; i < G.n_rows; i++) {
    for (int j = 0; j < G.n_cols; j++) {
      mat block = I(span(i,i+2), span(j,j+2));
      double v = sum(sum(block % coeff)); // weighted average
      H(i, j) = v;
    }
  }
  return H;
}

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
