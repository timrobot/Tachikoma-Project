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
/*  cube I = load_image("butterfly.jpg");

  mat red = I.slice(0);
  mat green = I.slice(1);
  mat blue = I.slice(2);

  mat gray = 0.3 * red + 0.6 * green + 0.1 * blue; // industry standard
  mat blurred = weight_avg(gray);

  // (hand wavy) derivative
  mat e = gray - blurred;


  e /= e.max();
  disp_image("edges", e);
  disp_wait();*/
  cube image(320, 320, 3, fill::ones);
  image *= 0.5;
  cube red(80, 80, 3, fill::zeros);
  red.slice(0).ones();
  cube blue(80, 80, 3, fill::zeros);
  blue.slice(2).ones();
  //image(span(0, 79), span(80, 159), span::all) = red;
  //image(span(0, 79), span(320, 399), span::all) = red;
  //image(span(400, 479), span(80, 159), span::all) = blue;
  //image(span(400, 479), span(320, 399), span::all) = blue;
  image(span(0, 79), span(80, 159), span::all) = red;
  image(span(0, 79), span(160, 239), span::all) = red;
  image(span(240, 319), span(80, 159), span::all) = blue;
  image(span(240, 319), span(160, 239), span::all) = blue;

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      cube vline = zeros<cube>(80, 1, 3);
      cube hline = zeros<cube>(1, 80, 3);
      image(span(i * 80, i * 80 + 79), span(j * 80, j * 80), span::all) = vline;
      image(span(i * 80, i * 80 + 79), span(j * 80 + 79, j * 80 + 79), span::all) = vline;
      image(span(i * 80, i * 80), span(j * 80, j * 80 + 79), span::all) = hline;
      image(span(i * 80 + 79, i * 80 + 79), span(j * 80, j * 80 + 79), span::all) = hline;
    }
  }

//  cube greenline(2, 480, 3, fill::zeros);
//  greenline.slice(1).ones();
//  image(span(239, 240), span(0, 479), span::all) = greenline;

  disp_image("img", image);
  disp_wait();
  save_image("grid.png", image);

  return 0;
}
