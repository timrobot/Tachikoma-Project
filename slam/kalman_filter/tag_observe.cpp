#include <armadillo>

void filter_active(mat &m, ivec active) {
  mat _m(sum(active), m.n_cols);
  int ind = 0;
  for (int i = 0; i < (int)active.n_elem; i++) {
    if (active(i)) {
      _m.row(ind) = m.row(ind);
      ind++;
    }
  }
  m = _m;
}

void tag_observe_pos(vec obs, mat pos, vec &mu, mat &sigma) {
  int k = obs.n_elem;
  if (k < 2) {
    return vec(pos.n_cols, fill::zeros); // this will be the only strange position
  }

  // compute the pose from the linear regression solver
  mat obs_k = repmat(obs(span(k-1, k-1)), k-1, 1);
  mat obs = obs(span(0, k-2));
  mat pos_k = repmat(pos(span(k-1, k-1), span::all), k-1, 1);
  mat pos = pos_(span(0, k-2), span::all);

  mat A = 2 * pos - repmat(pos_k, pos.n_rows, 1);
  mat b = obs % obs - obs_k % obs_k - sum(pos % pos, 1) + sum(pos_k % pos_k, 1);
  
  mat x = pinv(A.t() * A) * A.t() * b;
  mu = x.col(0);

  mat diff = repmat(z.t(), pos.n_rows, 1) + pos;
  diff = sqrt(sum(diff % diff, 1));
  diff -= obs;
  sigma = diff.t() * diff / (double)diff.n_rows;
}

void tag_observe_theta(vec t, mat pos, vec z, vec &mu, mat &sigma) {
  pos -= z;
  vec theta = atan2(pos(1), pos(0));
  vec dt = theta - t;
  mu = mean(dt);
  vec diff = ones<vec>(dt.n_elem) * mu - dt;
  sigma = dot(diff, diff) / (double)diff.n_elem;
}

void tag_observe(mat obs, mat pos, ivec active, vec &mu, mat &sigma) {
  // filter both the observation and the position
  filter_active(obs, active);
  filter_active(pos, active);
  // get the obs and pos and put it into their respective functions
  vec posmu;
  mat possigma;
  vec thetamu;
  mat thetasigma;
  tag_observe_pos(obs.col(0), pos, posmu, possigma);
  tag_observe_theta(obs.col(1), pos, thetamu, thetasigma);
  // assign the mus and sigmas
  mu = vec({ posmu(0), posmu(1), thetamu });
  sigma = mat(3, 3, fill::zeros);
  sigma(span(0,1), span(0,1)) = possigma;
  sigma(2, 2) = thetasigma;
}

void tag_generate(mat 
