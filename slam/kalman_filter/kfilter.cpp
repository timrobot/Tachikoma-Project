#include <armadillo>

mat motion_data;
mat sensor_data;

mat set_motion_data(mat d) {
  motion_data = d;
}

mat set_sensor_data(mat d) {
  sensor_data = d;
}

mat variance(data) {
  vec avg = sum(data, 1) / data.n_cols;
  vec diff = sum(data - repmat(avg, 1, data.n_cols), 1);
  mat var = diff * diff.t();
  return var;
}

mat F() { // no drift
  mat A = reshape(mat({
        1, 0, 0, 1, 0, 0,
        0, 1, 0, 0, 1, 0,
        0, 0, 1, 0, 0, 1,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1
      }), 6, 6).t();
  return A;
}

mat B(mat mu) {
  double theta = mu(2, 0);
  mat R = reshape(mat({
        cos(theta), -sin(theta), 0,
        sin(theta), cos(theta), 0,
        0, 0, 1,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0
      }), 3, 6).t();
  double v = sqrt(2) / 2;
  mat u2mu = reshape(mat({
        v, -v, -v, v,
        v, v, v, v,
        -1, 1, -1, 1,
      }), 4, 3).t();
  return 33.0 * M_PI / 32.0 * R * u2mu;
}

mat H() {
  mat C = reshape(mat({
        1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0
        }), 6, 3).t();
  return C;
}

mat R() {
  return variance(motion_data);
}

mat Q() {
  mat Q_ = eye(6) * 2;
  return Q_;
}

void kfilter(mat &mu, mat &sigma, mat u, mat z) {
  mat bel_mu = mu + B(mu) * u;
  mat bel_sigma = sigma + R();
  mat K = bel_sigma * C().t() * (C() * bel_sigma * C().t() + Q()).i();
  mu = bel_mu + K * (z - C() * bel_mu);
  sigma = (I - K * C()) * bel_sigma;
}
