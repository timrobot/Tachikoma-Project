#include "highgui.h"
#include "imgproc.h"
#include "feature.h"
#include <armadillo>
#include <vector>
#include <cmath>
#include <cassert>

using namespace arma;
using namespace std;

static void mergesort(vec &keys, uvec &values);
static mat atan2(const mat &Y, const mat &X);

// try to get adaptive thresholding working
mat edge2(const mat &F, int op) {
  switch (op) {
    case EDGE_SOBEL:
      return sobel_edge2(F);
    case EDGE_CANNY:
      return canny2(F);
    case EDGE_DOG:
      return dog2(F);
    case EDGE_LOG:
      return log2(F);
    default:
      return mat();
  }
}

cube edge2(const cube &F, int op) {
  return gray2rgb(edge2(rgb2gray(F), op));
}

mat sobel_edge2(const mat &F, uword n, double sigma2) {
  // smooth first
  mat H = conv2(F, gauss2(n, sigma2));
  mat DX, DY;
  gradient2(DX, DY, H);
  mat G = sqrt(DX % DX + DY % DY);
  return G;
}

mat canny2(const mat &F, double low, double high, uword n, double sigma2) { // not the complete canny
  // filter
  mat H = conv2(F, gauss2(n, sigma2));
  // intensity gradient
  mat DX, DY;
  gradient2(DX, DY, H);
  // non-maximal suppression
  mat G = sqrt(DX % DX + DY % DY);
  mat T = atan2(DY, DX);
  H = nmm2(G, T);
  G /= G.max(); // normalise
  // double threshold
  mat strong = (G >= high) % H;
  mat weak = (low < G && G < high) % H;
  // hysteresis edge-tracking
//  mat K = strong + weak;
//  K /= K;
  return strong;
}

mat dog2(const mat &F, double alpha, uword n, double sigma2) {
  return F - alpha * conv2(F, gauss2(n, sigma2));
}

mat log2(const mat &F, uword n, double sigma2) {
  return conv2(F, laplace_gauss2(n, sigma2));
}

void blob2(const mat &F, vector<vec> &centroids) {
  mat visited(F.n_rows, F.n_cols, fill::zeros);
  vector<ivec> tovisit;
  size_t nvisited = 0;
  centroids.clear();
  ivec pt;
  vec mid(2);
  int npts;
  for (uword i = 0; i < F.n_rows; i++) {
    for (uword j = 0; j < F.n_cols; j++) {
      if (!visited(i, j)) {
        mid.zeros();
        npts = 0;
        tovisit.push_back(ivec({(sword)i,(sword)j}));
        visited(i,j) = 1;
        while (tovisit.size() > nvisited) {
          pt = tovisit[nvisited];
          if (pt(0)-1>=0 && !visited(pt(0)-1,pt(1)) && F(pt(0)-1,pt(1)) == F(pt(0),pt(1))) {
            visited(pt(0)-1,pt(1)) = 1;
            tovisit.push_back(ivec({pt(0)-1,pt(1)}));
          }
          if (pt(0)+1<F.n_rows && !visited(pt(0)+1,pt(1)) && F(pt(0)+1,pt(1)) == F(pt(0),pt(1))) {
            visited(pt(0)+1,pt(1)) = 1;
            tovisit.push_back(ivec({pt(0)+1,pt(1)}));
          }
          if (pt(1)-1>=0 && !visited(pt(0),pt(1)-1) && F(pt(0),pt(1)-1) == F(pt(0),pt(1))) {
            visited(pt(0),pt(1)-1) = 1;
            tovisit.push_back(ivec({pt(0),pt(1)-1}));
          }
          if (pt(1)+1<F.n_cols && !visited(pt(0),pt(1)+1) && F(pt(0),pt(1)+1) == F(pt(0),pt(1))) {
            visited(pt(0),pt(1)+1) = 1;
            tovisit.push_back(ivec({pt(0),pt(1)+1}));
          }
          mid += vec({(double)pt(0), (double)pt(1)});
          npts++;
          nvisited++;
        }
        if (npts > 24) {
          centroids.push_back(mid/npts);
        }
      }
    }
  }
}

mat corner2(const mat &I, int op) {
  switch (op) {
    case CORNER_SOBEL:
      return sobel_corner2(I);
    case CORNER_HARRIS:
      return harris2(I, reshape(mat({1,2,1,2,4,2,1,2,1}),3,3)/16.0);
    default:
      return mat();
  }
}

mat sobel_corner2(const mat &F, uword n, double sigma2) {
  mat H = reshape(mat({
    1, -2, 1,
    -2, 4, -2,
    1, -2, 1
  }), 3, 3).t();
  mat G = conv2(F, gauss2(n, sigma2));
  G = conv2(G, H);
}

mat harris2(const mat &I, const mat &W) {
  assert(W.n_rows == W.n_cols);
  mat DX, DY;
  gradient2(DX, DY, I); // grab the gradients
  // place gradients into padded matrix
  mat wIxx = conv2(DX % DX, W);
  mat wIxy = conv2(DX % DY, W);
  mat wIyy = conv2(DY % DY, W);
  // find the taylor expansion-based corner detector
  //double k = 0.07;
  //return ((wIxx % wIyy) - (wIxy % wIxy)) - (k * ((wIxx + wIyy) % (wIxx + wIyy)));
  double eps = 0.05;
  return 2.0 * ((wIxx % wIyy) - (wIxy % wIxy)) / (wIxx + wIyy + eps);
}

mat lines2(const mat &I, const vector<vec> &pts, int op) {
  assert(op == LINE_RANSAC || op == LINE_HOUGH);
  switch (op) {
    case LINE_RANSAC:
      return ransac(pts, 10.0, (int)pts.size());
    case LINE_HOUGH:
      return hough_line(pts, 0.5, I.n_rows, I.n_cols);
    default:
      return mat();
  }
}

mat ransac(const vector<vec> &pts, double sigma2, int k) {
  int consensus = 0;
  vec pt1, pt2;
  int i1, i2;
  int ind1 = 0, ind2 = 0;
  for (int i = 0; i < k; i++) {
    // choose two random points
    i1 = i;
    i2 = rand() % pts.size();
    while (i1 == i2) {
      i2 = rand() % pts.size();
    }

    // fit a particular model
    vec u_i2_i1 = normalise(pts[i2] - pts[i1]);

    // get the set of inliers that are close enough to the line
    int hyp_consensus = 0;
    for (int j = 0; j < k; j++) {
      // compute the distance from the point
      vec v_j_i1 = pts[j] - pts[i1];
      vec para = dot(v_j_i1.t(), u_i2_i1) * u_i2_i1;
      vec perp = v_j_i1 - para;
      double distance = sqrt(dot(perp, perp));
      if (distance < sigma2) {
        hyp_consensus++;
      }
    }

    // compare the consensus to the known consensus
    // if larger, then replace
    if (hyp_consensus > consensus) {
      pt1 = pts[i1];
      pt2 = pts[i2];
      consensus = hyp_consensus;
      ind1 = i1;
      ind2 = i2;
    } else if (consensus == 0) {
      i = 0; // keep repeating until a consensus is reached
    }
  }

  mat ans(2, 2);
  ans.col(0) = pt1;
  ans.col(1) = pt2;
  return ans;
}

mat hough_line(const vector<vec> &pts, double sigma2, size_t n_rows, size_t n_cols) {
  // create an accumulator
  mat accumulator(2 * (n_rows + n_cols), 180, fill::zeros); // radius, degrees
  for (const vec &pt : pts) {
    for (uword theta = 0; theta < accumulator.n_cols; theta++) {
      double rad = (double)theta * M_PI / 180.0;
      double y = pt(0);
      double x = pt(1);
      uword r = (uword)(y * sin(rad) + x * cos(rad)) + n_rows + n_cols;
      if (r < accumulator.n_rows && r != (n_rows + n_cols)) { // error otherwise
        accumulator(r, theta) += 1.0;
      }
    }
  }
  // find the approximate gradient of the accumulator
  mat DX, DY;
  gradient2(DX, DY, accumulator);
  mat theta = atan2(DY, DX);
  mat radius = sqrt(DX % DX + DY % DY);
  accumulator = nmm2(accumulator, theta, 2);
  accumulator /= accumulator.max(); // normalized

  mat II = imresize2(accumulator, 500, accumulator.n_cols);
  II /= II.max();
  disp_image("accumulator", II);
  disp_wait();

  // now grab all the local maximums
  uvec ind = find(accumulator > 0.0);
  vec votes(ind.n_elem);
  mat rt(3, ind.n_elem);
  for (uword i = 0; i < ind.n_elem; i++) {
    rt.col(i) = vec({ 0.0,
        (double)(ind(i) % accumulator.n_rows),
        (double)(ind(i) / accumulator.n_rows) });
    uword radius = rt(1, i);
    uword theta = rt(2, i);
    votes(i) = accumulator(radius, theta);
    rt(0, i) = votes(i);
  }

  // sort them to get the best matches
  uvec vv = cumsum(ones<uvec>(votes.n_elem)) - 1;
  mergesort(votes, vv);
  mat hlines(3, ind.n_elem);
  for (uword i = 0; i < vv.n_elem; i++) {
    uword j = vv(i);
    hlines.col(i) = rt.col(j);
  }
  hlines.row(1) -= (n_rows + n_cols);

  return hlines;
}

// TODO
mat hough_circle(const vector<vec> &pts, double sigma2, size_t n_rows, size_t n_cols) {
  cube accumulator(n_rows, n_cols, 2 * (n_rows + n_cols), fill::zeros);
  for (uword R = 0; R < accumulator.n_slices; R++) {
    for (vec pt : pts) {
      accumulator(pt(0), pt(1), R) += 1.0;
    }
  }
  return accumulator;
}

// STATIC

static void mergesort(vec &keys, uvec &values) {
  if (keys.size() == 1) {
    return;
  }
  uword mid = keys.n_elem/2;
  vec keys1 = keys(span(0,mid-1));
  vec keys2 = keys(span(mid,keys.n_elem-1));
  uvec values1 = values(span(0,mid-1));
  uvec values2 = values(span(mid,keys.n_elem-1));
  mergesort(keys1, values1);
  mergesort(keys2, values2);
  uword i = 0, j = 0, k = 0;
  while (k < keys.n_elem) {
    if (i == keys1.n_elem) {
      keys(k) = keys2(j);
      values(k) = values2(j);
      j++;
      k++;
    } else if (j == keys2.n_elem || keys1(i) > keys2(j)) {
      keys(k) = keys1(i);
      values(k) = values1(i);
      i++;
      k++;
    } else {
      keys(k) = keys2(j);
      values(k) = values2(j);
      j++;
      k++;
    }
  }
}

static mat atan2(const mat &Y, const mat &X) {
  assert(Y.n_rows == X.n_rows && Y.n_cols == X.n_cols);
  mat ans(Y.n_rows, Y.n_cols);
  for (uword i = 0; i < ans.n_rows; i++) {
    for (uword j = 0; j < ans.n_cols; j++) {
      ans(i, j) = atan2(Y(i, j), X(i, j));
    }
  }
  return ans;
}
