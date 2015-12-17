#include "highgui.h"
#include "imgproc.h"
#include <cmath>

using namespace arma;

/* Rift dimensions are generally 2 screens, each 800x640 pixels
 * so in total 800x1280
 * the recommended parameters are 1.0, 0.22, 0.24, 0.0
 */

const double u_distortion[4] = { 1.0, 0.22, 0.24, 0 }; // distortion parameters

double distortionScale(const vec &offset) {
  assert(offset.n_elem == 2);
  vec offsetSquared = offset % offset;
  double radiusSquared = offsetSquared(0) + offsetSquared(1);
  double distortion =
    u_distortion[0] +
    u_distortion[1] * radiusSquared +
    u_distortion[2] * radiusSquared * radiusSquared +
    u_distortion[3] * radiusSquared * radiusSquared * radiusSquared;
  return distortion;
}

mat barrel_distort(const mat &F, double offset_x) {
  mat G(F.n_rows, F.n_cols, fill::zeros);
  // find the radius of the barrel distortion
  double r_y = F.n_rows / 2;
  double r_x = F.n_cols / 2;
  double r_max = sqrt((r_x * (1 + abs(offset_x))) * (r_x * (1 + abs(offset_x))) + r_y * r_y);
  // grab the distortionScale
  for (uword i = 0; i < F.n_rows; i++) {
    for (uword j = 0; j < F.n_cols; j++) {
      double x = (double)(j-r_x) / r_max + offset_x;
      double y = (double)(i-r_y) / r_max;
      double distortion = distortionScale(vec({ y, x }));
      int _i = (int)round(distortion*y*r_max+r_y);
      int _j = (int)round((distortion*x-offset_x)*r_max+r_x);
      if (_i >= 0 && _i < F.n_rows && _j >= 0 && _j < F.n_cols) {
        G(i, j) = F(_i, _j);
      }
    }
  }
  return G;
}

cube barrel_distort_rgb(const cube &F, double offset_x) {
  cube G(F.n_rows, F.n_cols, F.n_slices);
  G.slice(0) = barrel_distort(F.slice(0), offset_x);
  G.slice(1) = barrel_distort(F.slice(1), offset_x);
  G.slice(2) = barrel_distort(F.slice(2), offset_x);
  return G;
}

cube ovr_image(const cube &left, const cube &right, double offset_x) {
  cube l = barrel_distort_rgb(left, -offset_x);
  cube r = barrel_distort_rgb(right, offset_x);
  cube combined(l.n_rows, l.n_cols + r.n_cols, l.n_slices);
  combined(span::all, span(0, l.n_cols-1), span::all) = l;
  combined(span::all, span(l.n_cols, l.n_cols+r.n_cols-1), span::all) = r;
  return imresize2(combined, 800, 1200);
}
