#include "highgui.h"
#include "imgproc.h"
#include <cmath>
#include "gpu_util.h"
#include <cassert>
#include <cstdio>

#define M_PI_8   0.39269908169872414
#define M_3_PI_8 1.1780972450961724
#define M_5_PI_8 1.9634954084936207
#define M_7_PI_8 2.748893571891069

__global__ void GPU_conv2_gen(float *G, float *F, float *H, int F_n_rows, int F_n_cols, int H_n_rows, int H_n_cols) {
  // assume that the matrix represents an image
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  int idy = blockIdx.y * blockDim.y + threadIdx.y;
  if (idx >= F_n_cols || idy >= F_n_rows) {
    return;
  }
  // stencil operation
  int my = H_n_rows / 2;
  int mx = H_n_cols / 2;

  // call kernel? for now just use a for loop
  float total = 0.0f;
  float weight = 0.0f;
  int _i, _j, i, j;
  for (i = 0; i < H_n_rows; i++) {
    for (j = 0; j < H_n_cols; j++) {
      _i = idy + my - i - 1;
      _j = idx + mx - j - 1;
      if (_i >= 0 && _i < F_n_rows && _j >= 0 && _j < F_n_cols) {
        total += H[IJ2C(i, j, H_n_rows)] * F[IJ2C(_i, _j, F_n_rows)];
        weight += H[IJ2C(i, j, H_n_rows)];
      }
    }
  }
  G[IJ2C(idy, idx, F_n_rows)] = total / weight;
}

gcube gpu_conv2(const gcube &F, const gcube &K) {
  gcube G(F.n_rows, F.n_cols, F.n_slices);
  for (int k = 0; k < F.n_slices; k++) {
    dim3 blockSize(16, 16, 1);
    dim3 gridSize((F.n_cols-1)/16+1, (F.n_rows-1)/16+1, 1);
    // call the GPU kernel
    GPU_conv2_gen<<<gridSize, blockSize>>>(
        &G.d_pixels[IJK2C(0, 0, k, F.n_rows, F.n_cols)],
        &F.d_pixels[IJK2C(0, 0, k, F.n_rows, F.n_cols)],
        K.d_pixels, F.n_rows, F.n_cols, K.n_rows, K.n_cols);
    checkCudaErrors(cudaGetLastError());
  }
  return G;
}

__global__ void GPU_conv2_sym(float *G, float *F, float *Hy, float *Hx, int F_n_rows, int F_n_cols, int H_n_rows, int H_n_cols) {
  // assume that the matrix represents an image
  int j = blockIdx.x * blockDim.x + threadIdx.x;
  int i = blockIdx.y * blockDim.y + threadIdx.y;
  if (j >= F_n_cols || i >= F_n_rows) {
    return;
  }
  // stencil operation
  int mi = H_n_rows / 2;
  int mj = H_n_cols / 2;

  // for now just use a for loop
  float total = 0.0f;
  float weight = 0.0f;
  for (int hi = 0; hi < H_n_rows; hi++) {
    int fi = i + mi - hi - 1;
    if (fi >= 0 && fi < F_n_rows) {
      total += Hy[hi] * F[IJ2C(fi, j, F_n_rows)];
      weight += Hy[hi];
    }
  }
  G[IJ2C(i, j, F_n_rows)] = total / weight;
  __syncthreads();
  total = 0.0f;
  weight = 0.0f;
  for (int hj = 0; hj < H_n_cols; hj++) {
    int fj = j + mj - hj - 1;
    if (fj >= 0 && fj < F_n_cols) {
      total += Hx[hj] * G[IJ2C(i, fj, F_n_rows)];
      weight += Hx[hj];
    }
  }
  G[IJ2C(i, j, F_n_rows)] = total / weight;
}

gcube gpu_conv2(const gcube &F, const gcube &V, const gcube &H) {
  gcube G(F.n_rows, F.n_cols, F.n_slices);
  for (int k = 0; k < F.n_slices; k++) {
    dim3 blockSize(16, 16, 1);
    dim3 gridSize((F.n_cols-1)/16+1, (F.n_rows-1)/16+1, 1);
    // call the GPU kernel
    GPU_conv2_sym<<<gridSize, blockSize>>>(
        &G.d_pixels[IJK2C(0, 0, k, G.n_rows, G.n_cols)],
        &F.d_pixels[IJK2C(0, 0, k, F.n_rows, F.n_cols)],
        V.d_pixels, H.d_pixels, F.n_rows, F.n_cols, V.n_rows, H.n_cols);
    checkCudaErrors(cudaGetLastError());
  }
  return G;
}

gcube gpu_gauss2(int n, double sigma2) {
  assert(n > 0);
  gcube H(n, n);
  float h_pixels[n * n];
  float total = 0.0;
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      double x = (double)i - ((double)(n-1)/2);
      double y = (double)j - ((double)(n-1)/2);
      float g = (float)exp(-(x * x + y * y) / (2 * sigma2));
      h_pixels[IJ2C(i, j, n)] = g;
      total += g;
    }
  }
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      h_pixels[IJ2C(i, j, n)] /= total;
    }
  }
  checkCudaErrors(cudaMemcpy(H.d_pixels, h_pixels, n * n * sizeof(float), cudaMemcpyHostToDevice));
  return H;
}

void gpu_gauss2(gcube &V, gcube &H, int n, double sigma2) {
  assert(n > 0);
  V.create(n);
  H.create(n);
  float h_pixels[n];
  float total = 0.0;
  for (int i = 0; i < n; i++) {
    double x = (double)i - ((double)(n-1)/2);
    float g = (float)exp(-(x * x) / (2 * sigma2));
    h_pixels[i] = g;
    total += g;
  }
  for (int i = 0; i < n; i++) {
    h_pixels[i] /= total;
  }
  checkCudaErrors(cudaMemcpy(V.d_pixels, h_pixels, n * sizeof(float), cudaMemcpyHostToDevice));
  checkCudaErrors(cudaMemcpy(H.d_pixels, h_pixels, n * sizeof(float), cudaMemcpyHostToDevice));
}

void gpu_edgesobel2(gcube &V, gcube &H, bool isVert) { // change to vector
  V.create(3);
  H.create(3);
  float V_h_pixels[3];
  float H_h_pixels[3];
  if (isVert) {
    V_h_pixels[0] = 0.25f; V_h_pixels[1] = 0.5f; V_h_pixels[2] = 0.25f;
    H_h_pixels[0] = 0.5f; H_h_pixels[1] = 0.0f; H_h_pixels[2] = -0.5f;
  } else {
    V_h_pixels[0] = 0.5f; V_h_pixels[1] = 0.0f; V_h_pixels[2] = -0.5f;
    H_h_pixels[0] = 0.25f; H_h_pixels[1] = 0.5f; H_h_pixels[2] = 0.25f;
  }
  // small memcpy
  checkCudaErrors(cudaMemcpy(V.d_pixels, V_h_pixels, 3 * sizeof(float), cudaMemcpyHostToDevice));
  checkCudaErrors(cudaMemcpy(H.d_pixels, H_h_pixels, 3 * sizeof(float), cudaMemcpyHostToDevice));
}

std::vector<gcube> gpu_gradient2(const gcube &F) {
  gcube sobel_v, sobel_h;
  std::vector<gcube> g;
  // vertical
  gpu_edgesobel2(sobel_v, sobel_h, true);
  g.push_back(gpu_conv2(F, sobel_v, sobel_h));
  // horizontal
  gpu_edgesobel2(sobel_v, sobel_h, false);
  g.push_back(gpu_conv2(F, sobel_v, sobel_h));
  return g;
}

__global__ void GPU_eucdist(float *C, float *A, float *B, int n_rows, int n_cols) {
  int j = blockIdx.x * blockDim.x + threadIdx.x;
  int i = blockIdx.y * blockDim.y + threadIdx.y;
  if (i >= n_rows || j >= n_cols) {
    return;
  }
  float dx = A[IJ2C(i, j, n_rows)];
  float dy = B[IJ2C(i, j, n_rows)];
  C[IJ2C(i, j, n_rows)] = sqrtf(dx * dx + dy * dy);
}

__global__ void GPU_minthresh2(float *G, float *F, int n_rows, int n_cols, float minthresh) {
  int j = blockIdx.x * blockDim.x + threadIdx.x;
  int i = blockIdx.y * blockDim.y + threadIdx.y;
  if (i >= n_rows || j >= n_cols) {
    return;
  }
  G[IJ2C(i, j, n_rows)] = F[IJ2C(i, j, n_rows)];
}

// By default uses the sobel operator
gcube gpu_edge2(const gcube &F, int n, double sigma2) {
  gcube V, H;
  // smooth first
  gpu_gauss2(V, H, n, sigma2);
  gcube G = gpu_conv2(F, V, H);
  // get gradients
  std::vector<gcube> dxdy = gpu_gradient2(G);
  // grab the eucdist
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((F.n_cols-1)/16+1, (F.n_rows-1)/16+1, 1);
  GPU_eucdist<<<gridSize, blockSize>>>(G.d_pixels, dxdy[0].d_pixels, dxdy[1].d_pixels, G.n_rows, G.n_cols);
  // do nonmaximal suppression, then thresholding
  gpu_nmm2(G, dxdy[0], dxdy[1]);
  // TODO: make adaptive thresholding
  GPU_minthresh2<<<gridSize, blockSize>>>(G.d_pixels, G.d_pixels, G.n_rows, G.n_cols, 0.4);
  return G;
}

void gpu_cornersobel2(gcube &V, gcube &H) { // change to vector
  V.create(3);
  H.create(3);
  float V_h_pixels[3];
  float H_h_pixels[3];
  V_h_pixels[0] = 0.25f; V_h_pixels[1] = -0.5f; V_h_pixels[2] = 0.25f;
  H_h_pixels[0] = 0.25f; H_h_pixels[1] = -0.5f; H_h_pixels[2] = 0.25f;
  // small memcpy
  checkCudaErrors(cudaMemcpy(V.d_pixels, V_h_pixels, 3 * sizeof(float), cudaMemcpyHostToDevice));
  checkCudaErrors(cudaMemcpy(H.d_pixels, H_h_pixels, 3 * sizeof(float), cudaMemcpyHostToDevice));
}

// By default uses the sobel operator
gcube gpu_corner2(const gcube &F, int n, double sigma2) {
  gcube sobel_v, sobel_h;
  gpu_cornersobel2(sobel_v, sobel_h);
  return gpu_conv2(F, sobel_v, sobel_h);
}

__global__ void GPU_nmm2(float *G, float *F, float *Fx, float *Fy, int n_rows, int n_cols) {
  int j = blockIdx.x * blockDim.x + threadIdx.x;
  int i = blockIdx.y * blockDim.y + threadIdx.y;
  if (i >= n_rows || j >= n_cols) {
    return;
  }
  // get the direction of the gradient...
  if (i == 0 || i == n_rows - 1 || j == 0 || j == n_cols - 1) {
    G[IJ2C(i, j, n_rows)] = 0;
    return;
  }
  float angle = atan2f(Fy[IJ2C(i, j, n_rows)], Fx[IJ2C(i, j, n_rows)]);
  int dx = (angle > -M_3_PI_8 && angle < M_3_PI_8) - (angle < -M_5_PI_8 || angle > M_5_PI_8);
  int dy = (angle > M_PI_8 && angle < M_7_PI_8) - (angle < -M_PI_8 && angle > -M_7_PI_8);
  int v = F[IJ2C(i, j, n_rows)];
  int v1 = v - F[IJ2C(dy, dx, n_rows)];
  int v2 = v - F[IJ2C(-dy, -dx, n_rows)];
  G[IJ2C(i, j, n_rows)] = (v1 + v2) * 0.5f * (v1 > 0 && v2 > 0);
}

gcube gpu_nmm2(const gcube &F, const gcube &Fx, const gcube &Fy) {
  gcube G(F.n_rows, F.n_cols);
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((F.n_cols-1)/16+1, (F.n_rows-1)/16+1, 1);
  GPU_nmm2<<<gridSize, blockSize>>>
    (G.d_pixels, F.d_pixels, Fx.d_pixels, Fy.d_pixels, F.n_rows, F.n_cols);
  checkCudaErrors(cudaGetLastError());
  return G;
}

__global__ void GPU_bilinear_filter2(float *G, float *F, int G_rows, int G_cols, int F_rows, int F_cols, int n_slices, float kr, float kc) {
  // gather
  int j = blockIdx.x * blockDim.x + threadIdx.x;
  int i = blockIdx.y * blockDim.y + threadIdx.y;
  if (i >= G_rows || j >= G_cols) {
    return;
  }
  float y = (float)i * kr - 0.5f;
  float x = (float)j * kc - 0.5f;
  int Fi = (int)floorf(y);
  int Fj = (int)floorf(x);
  float dy = 1.0f - (y - floorf(y));
  float dx = 1.0f - (x - floorf(x));
  float wsum = 0.0f;
  float total = 0.0f;
  for (int k = 0; k < n_slices; k++) {
    if (Fj >= 0 && Fj < F_cols) {
      if (Fi >= 0 && Fi < F_rows) {
        wsum += dx * dy * F[IJK2C(Fi, Fj, k, F_rows, F_cols)];
        total += dx * dy;
      }
      if (Fi+1 >= 0 && Fi+1 < F_rows) {
        wsum += dx * (1-dy) * F[IJK2C(Fi+1, Fj, k, F_rows, F_cols)];
        total += dx * (1-dy);
      }
    }
    if (Fj+1 >= 0 && Fj+1 < F_cols) {
      if (Fi >= 0 && Fi < F_rows) {
        wsum += (1-dx) * dy * F[IJK2C(Fi, Fj+1, k, F_rows, F_cols)];
        total += (1-dx) * dy;
      }
      if (Fi+1 >= 0 && Fi < F_cols) {
        wsum += (1-dx) * (1-dy) * F[IJK2C(Fi+1, Fj+1, k, F_rows, F_cols)];
        total += (1-dx) * (1-dy);
      }
    }
    if (total != 0.0f) {
      G[IJK2C(i, j, k, G_rows, G_cols)] = wsum / total; // normalize
    }
  }
}

gcube gpu_imresize2(const gcube &A, int m, int n) {
  gcube G(m, n, A.n_slices);
  double kr = (double)A.n_rows / (double)m;
  double kc = (double)A.n_cols / (double)n;
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((G.n_cols-1)/16+1, (G.n_rows-1)/16+1, 1);
  GPU_bilinear_filter2<<<gridSize, blockSize>>>(G.d_pixels, A.d_pixels,
      G.n_rows, G.n_cols, A.n_rows, A.n_cols, A.n_slices, (float)kr, (float)kc);
  checkCudaErrors(cudaGetLastError());
  return G;
}
