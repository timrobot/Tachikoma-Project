#include "highgui.h"
#include "imgproc.h"
#include <cstdio>
#include <iostream>

using namespace arma;
using namespace std;

/** Convolution operations **/

static mat flipmat(const mat &F) {
  mat G(F.n_rows, F.n_cols);
  for (uword i = 0; i < F.n_rows; i++) {
    for (uword j = 0; j < F.n_cols; j++) {
      uword r = F.n_rows - 1 - i;
      uword c = F.n_rows - 1 - j;
      G(r, c) = F(i, j);
    }
  }
  return G;
}

mat conv2(const mat &F, const mat &H) {
  uword my = H.n_rows / 2;
  uword mx = H.n_cols / 2;
  mat G(F.n_rows, F.n_cols);

  mat A(F.n_rows+H.n_rows-1, F.n_cols+H.n_cols-1, fill::zeros);
  A(span(my, F.n_rows+my-1), span(mx, F.n_cols+mx-1)) = F;

  if (arma::rank(H) == 1) {
    // linearly separable
    rowvec rk = H.row(0);
    rk /= sum(abs(rk.t())); // normalize
    rk = fliplr(rk);
    colvec ck = H.col(0);
    ck /= sum(abs(ck)); // normalize
    ck = flipud(ck);

    // first do the convolution across the rows
    mat B(F.n_rows+H.n_rows-1, F.n_cols, fill::zeros);
    for (uword i = 0; i < F.n_rows; i++) {
      for (uword j = 0; j < F.n_cols; j++) {
        rowvec S = A(i+my, span(j, j+H.n_cols-1));
        B(i+my, j) = sum(S % rk);
      }
    }

    // then do the convolution across the cols
    for (uword i = 0; i < F.n_rows; i++) {
      for (uword j = 0; j < F.n_cols; j++) {
        colvec S = B(span(i, i+H.n_rows-1), j);
        G(i, j) = sum(S % ck);
      }
    }

  } else {
    // regular convolution
    mat K = flipmat(H); // convolution kernel inverse
    for (uword i = 0; i < F.n_rows; i++) {
      for (uword j = 0; j < F.n_cols; j++) {
        // get a chunk of the A cube
        mat S = A(span(i, i+H.n_rows-1), span(j, j+H.n_cols-1));
        G(i, j) = accu(S % K);
      }
    }
  }
  return G;
}

cube conv2(const cube &F, const mat &H) {
  cube G(F.n_rows, F.n_cols, F.n_slices);
  for (uword i = 0; i < F.n_slices; i++) {
    G.slice(i) = conv2(F.slice(i), H);
  }
  return G;
}

mat gauss2(uword n, double sigma2) {
  double mu = (double)(n - 1) / 2.0;
  double c = 1.0 / (2.0 * M_PI * sigma2);
  double o2_2 = 2.0 * sigma2;
  mat H(n, n);
  for (uword i = 0; i < n; i++) {
    for (uword j = 0; j < n; j++) {
      double y = (double)i - mu;
      double x = (double)j - mu;
      H(i, j) = c * exp(-(x * x + y * y) / o2_2);
    }
  }
  return H / accu(H);
}

mat laplace_gauss2(uword n, double sigma2) {
  double mu = (double)(n - 1) / 2;
  double o2_2 = 2.0 * sigma2;
  double c = 1.0 / (M_PI * o2_2);
  mat H(n, n);
  for (uword i = 0; i < n; i++) {
    for (uword j = 0; j < n; j++) {
      double y = (double)i - mu;
      double x = (double)j - mu;
      y = y * y;
      x = x * x;
      double g = c * exp(-(x + y) / o2_2);
      H(i, j) = g * (x + y - o2_2) / (sigma2 * sigma2);
    }
  }
  return H;
}

void gradient2(mat &DX, mat &DY, const mat &F) {
  mat sobel = reshape(mat({
        -1.0, 0.0, 1.0 ,
        -2.0, 0.0, 2.0 ,
        -1.0, 0.0, 1.0  }), 3, 3).t();
  sobel /= accu(abs(sobel));
  DX = conv2(F, sobel);
  DY = conv2(F, sobel.t());
}

void gradient2(mat &DX, mat &DY, const cube &F) {
  mat sobel = reshape(mat({
        -1.0, 0.0, 1.0 ,
        -2.0, 0.0, 2.0 ,
        -1.0, 0.0, 1.0  }), 3, 3).t();
  sobel /= accu(abs(sobel));
  DX = conv2(F, sobel);
  DY = conv2(F, sobel.t());
}

mat nmm2(const mat &F, const mat &theta, uword nsize) {
  mat A(F.n_rows + (2 * nsize), F.n_cols + (2 * nsize), fill::zeros);
  A(span(nsize, F.n_rows+nsize-1), span(nsize, F.n_cols+nsize-1)) = F;
  mat G(F.n_rows, F.n_cols, fill::zeros);
  for (uword i = 0; i < F.n_rows; i++) {
    for (uword j = 0; j < F.n_cols; j++) {
      if (nsize == 0) {
        double t = theta(i, j) * 4 / M_PI;
        if (t < 0) {
          t += 8.0;
        }
        uword ind[2];
        // instead of rounding, use ceil and floor for more accuracy
        ind[0] = (int)round(t) % 8;
        ind[1] = (ind[0] + 4) % 8;
        double v[2];
        // choose a grid index
        for (int idx = 0; idx < 2; idx++) {
          uvec pt;
          switch (ind[idx]) {
            case 0: pt = { i    , j + 1 }; break;
            case 1: pt = { i + 1, j + 1 }; break;
            case 2: pt = { i + 1, j     }; break;
            case 3: pt = { i + 1, j - 1 }; break;
            case 4: pt = { i    , j - 1 }; break;
            case 5: pt = { i - 1, j - 1 }; break;
            case 6: pt = { i - 1, j     }; break;
            case 7: pt = { i - 1, j + 1 }; break;
            case 8: pt = { i, j }; break; // an error has occurred
          }
          if (pt(0) >= 0 && pt(0) < F.n_rows &&
              pt(1) >= 0 && pt(1) < F.n_cols) {
            v[idx] = A(pt(0), pt(1));
          } else {
            v[idx] = A(i, j);
          }
        }
        if (v[0] < A(i, j) && v[1] < A(i, j)) {
          G(i, j) = A(i, j);
        } else {
          G(i, j) = 0.0;
        }
      } else {
        mat sec = A(span(i,i+(2*nsize)),span(j,j+(2*nsize)));
        if (A(i+nsize,j+nsize) == sec.max()) {
          G(i, j) = A(i+nsize,j+nsize);
        }
      }
    }
  }
  return G;
}

mat k_cluster(const mat &S, uword k, int niter, vector<vec> &centroids, vector<vec> hyp, bool usehyp) {
  // do a check against the size of the image
  if (k >= S.n_cols-1 || k == 0) {
    fprintf(stderr, "Not valid cluster number!\n");
    return S;
  }
  // generate k randomized centers uniformly random
  centroids.clear();
  if (usehyp) {
    for (uword i = 0; i < hyp.size() && i < k; i++) {
      centroids.push_back(hyp[i]);
    }
  }
  for (uword i = hyp.size(); i < k; i++) {
    centroids.push_back(S.col(rand() % S.n_cols));
  }
  // try to get the cluster element by doing iterative matchups
  mat interim(3, k);
  int count[k];
  for (int iter = 0; iter < niter; iter++) {
    interim.zeros();
    memset(count, 0, sizeof(int) * k);
    // :3 GPU part please
    for (int i = 0; i < S.n_cols; i++) {
      double minerr = 0;
      bool errset = false;
      int index = 0;
      for (int kid = 0; kid < k; kid++) {
        vec diff = S.col(i) - centroids[kid];
        double interr = dot(diff, diff);
        if (!errset || minerr > interr) {
          errset = true;
          minerr = interr;
          index = kid;
        }
      }
      interim.col(index) += S.col(i);
      count[index]++;
    }
    for (int i = 0; i < (int)k; i++) {
      centroids[i] = interim.col(i) / (double)count[i];
    }
  }
  // generate the cluster from the partitions
  mat cluster(S.n_rows, k);
  for (int i = 0; i < k; i++) {
    cluster.col(i) = centroids[i];
  }
  return cluster;
}

mat hist_segment2(const mat &F, uword k, vector<vec> &centroids, int niter, vector<vec> hyp, bool usehyp) { // do mixture of gaussians later on?
  mat H = F;
  // cluster the color points based on the number of clusters
  mat S;
  if (usehyp) {
    k_cluster(reshape(H, 1, H.n_rows * H.n_cols), k, niter, centroids, hyp, true);
  } else {
    k_cluster(reshape(H, 1, H.n_rows * H.n_cols), k, niter, centroids);
  }
  // filter the image to use the closest cluster based on
  // the k-cluster algorithm given before
  for (uword i = 0; i < H.n_rows; i++) {
    for (uword j = 0; j < H.n_cols; j++) {
      // find the minimum
      uword min_ind = 0;
      vec diff = H(i, j) - S.col(0);
      double min_val = sqrt(dot(diff, diff));
      for (uword k = 0; k < S.n_cols; k++) {
        diff = H(i, j) - S.col(k);
        double interim = sqrt(dot(diff, diff));
        if (interim < min_val) {
          min_val = interim;
          min_ind = k;
        }
      }
      H(i, j) = S.col(min_ind)(0);
    }
  }
  return H;
}

cube hist_segment2(const cube &F, uword k, vector<vec> &centroids, int niter, vector<vec> hyp, bool usehyp) {
  cube C = F;
  mat Z(C.n_slices, C.n_rows * C.n_cols);
  for (uword i = 0; i < C.n_rows; i++) {
    for (uword j = 0; j < C.n_cols; j++) {
      vec RGB(C.n_slices);
      for (uword k = 0; k < C.n_slices; k++) {
        RGB(k) = C(i, j, k);
      }
      Z.col(j * C.n_rows + i) = RGB;
    }
  }
  mat S;
  if (usehyp) {
    S = k_cluster(Z, k, niter, centroids, hyp, true);
  } else {
    S = k_cluster(Z, k, niter, centroids);
  }
  for (uword i = 0; i < C.n_rows; i++) {
    for (uword j = 0; j < C.n_cols; j++) {
      vec RGB(C.n_slices);
      for (uword k = 0; k < C.n_slices; k++) {
        RGB(k) = C(i, j, k);
      }
      uword min_ind = 0;
      vec diff = RGB - S.col(0);
      double min_val = sqrt(dot(diff, diff));
      for (uword k = 0; k < S.n_cols; k++) {
        diff = RGB - S.col(k);
        double interim = sqrt(dot(diff, diff));
        if (interim < min_val) {
          min_val = interim;
          min_ind = k;
        }
      }
      for (uword k = 0; k < C.n_slices; k++) {
        C(i, j, k) = S.col(min_ind)(k);
      }
    }
  }
  return C;
}

double sad2(const mat &I1, const mat &I2) {
  return accu(abs(I1 - I2));
}

double ssd2(const mat &I1, const mat &I2) {
  mat C = I1 - I2;
  return accu(C % C);
}

double ncc2(const mat &I1, const mat &I2) {
  assert(I1.n_rows == I2.n_rows && I1.n_cols == I2.n_cols);
  double mu1 = accu(I1) / (double)(I1.n_rows * I1.n_cols);
  double mu2 = accu(I2) / (double)(I1.n_rows * I1.n_cols);
  mat F = I1 - mu1;
  mat G = I2 - mu2;
  return accu(F % G) / (accu(F) * accu(G));
}

static vec mergesort(const vec &nums) {
  if (nums.size() == 1) {
    return nums;
  }
  vec a = mergesort(nums(span(0, nums.n_elem/2-1)));
  vec b = mergesort(nums(span(nums.n_elem/2, nums.n_elem-1)));
  uword i = 0, j = 0, k = 0;
  vec c(nums.n_elem, fill::zeros);
  while (k < c.n_elem) {
    if (i == a.n_elem) {
      c(k) = b(j); j++; k++;
    } else if (j == b.n_elem || a(i) < b(j)) {
      c(k) = a(i); i++; k++;
    } else {
      c(k) = b(j); j++; k++;
    }
  }
  return c;
}

static double median_of_medians(const vec &nums) {
  vec tnums = nums;
  while (tnums.n_rows >= 15) {
    vec new_nums = vec((uword)ceil((double)tnums.n_elem / 5.0));
    for (uword i = 0; i < new_nums.n_elem; i++) {
      uword left = i * 5;
      uword right = (i + 1) * 5;
      if (right > tnums.n_elem) {
        right = tnums.n_elem;
      }
      vec S = mergesort(tnums(span(left, right - 1)));
      new_nums(i) = S(S.n_elem / 2);
    }
    printf("ayyyyy\n");
    tnums = new_nums;
  }
  vec S = mergesort(tnums);
  return S(S.n_elem / 2);
}

mat medfilt2(const mat &F, uword k) {
  uword mid = k / 2;
  mat G(F.n_rows, F.n_cols);

  mat A(F.n_rows+k-1, F.n_cols+k-1, fill::zeros);
  A(span(mid, F.n_rows+mid-1), span(mid, F.n_cols+mid-1)) = F;

  for (uword i = 0; i < F.n_rows; i++) {
    for (uword j = 0; j < F.n_cols; j++) {
      mat C = A(span(i, i+k-1), span(j, j+k-1));
      G(i, j) = median_of_medians(vectorise(C));
    }
  }
  return G;
}

mat imresize2(const mat &A, uword m, uword n) {
  mat G(m, n, fill::zeros);
  double kr = (double)A.n_rows / (double)m;
  double kc = (double)A.n_cols / (double)n;
  // bilinear interpolation
  for (uword i = 0; i < m; i++) {
    for (uword  j = 0; j < n; j++) {
      double y = (double)i * kr - 0.5;
      double x = (double)j * kc - 0.5;
      int i_ = (int)floor(y);
      int j_ = (int)floor(x);
      double dy = 1.0 - (y - floor(y));
      double dx = 1.0 - (x - floor(x));
      double total = 0.0;
      if (j_ >= 0 && j_ < (int)A.n_cols) {
        if (i_ >= 0 && i_ < (int)A.n_rows) {
          G(i, j) += dx * dy * A(i_,j_);
          total += dx * dy;
        }
        if (i_+1 >= 0 && i_+1 < (int)A.n_rows) {
          G(i, j) += dx * (1-dy) * A(i_+1,j_);
          total += dx * (1-dy);
        }
      }
      if (j_+1 >= 0 && j_+1 < (int)A.n_cols) {
        if (i_ >= 0 && i_ < (int)A.n_rows) {
          G(i, j) += (1-dx) * dy * A(i_,j_+1);
          total += (1-dx) * dy;
        }
        if (i_+1 >= 0 && i_+1 < (int)A.n_rows) {
          G(i, j) += (1-dx) * (1-dy) * A(i_+1,j_+1);
          total += (1-dx) * (1-dy);
        }
      }
      if (total != 0.0) {
        G(i, j) /= total;
      } else {
        G(i, j) = 0.0;
      }
    }
  }
  return G;
}

cube imresize2(const cube &C, uword m, uword n) {
  cube F(m, n, C.n_slices);
  for (uword k = 0; k < C.n_slices; k++) {
    F.slice(k) = imresize2(C.slice(k), m, n);
  }
  return F;
}

mat fast_imresize_half(const arma::mat &A) {
  assert(A.n_rows > 1 && A.n_cols > 1);
  mat I(A.n_rows/2, A.n_cols/2);
  for (uword i = 0; i < I.n_rows; i++) {
    for (uword j = 0; j < I.n_cols; j++) {
      uword i_ = i * 2;
      uword j_ = j * 2;
      I(i, j) = (A(i_, j_) + A(i_+1, j_+1) +
          A(i_+1, j_) + A(i_, j_+1))/4;
    }
  }
  return I;
}

vector< vector<mat> > gausspyr2(const mat &I, int noctaves, int nscales, double sigma2) {
  vector< vector<mat> > pyramid;
  uword kernel_size = noctaves * (int)(sigma2 * 2) + 1;
  for (int o = 0; o < noctaves; o++) {
    vector<mat> octave;
    for (int s = 0; s < nscales; s++) {
      double var = sigma2 * s;
      mat B;
      if (o == 0) {
        B = I;
      } else {
        B = pyramid[s - 1][o];
        B = imresize2(B, B.n_rows / 2, B.n_cols / 2);
      }
      octave.push_back(conv2(B, gauss2(kernel_size, var)));
    }
    pyramid.push_back(octave);
  }
  return pyramid;
}

void lappyr2(vector< vector<mat> > &blurred, vector< vector<mat> > &edges,
    const mat &I, int noctaves, int nscales, double sigma2) {
  // grab a gaussian pyramid
  blurred = gausspyr2(I, noctaves, nscales, sigma2);
  // create edges from blurred images
  edges = vector< vector<mat> >();
  for (int o = 0; o < noctaves; o++) {
    vector<mat> octave;
    for (int s = 0; s < nscales - 1; s++) {
      int s1 = s;
      int s2 = s + 1;
      octave.push_back(blurred[o][s2] - blurred[o][s1]);
    }
    edges.push_back(octave);
  }
}
