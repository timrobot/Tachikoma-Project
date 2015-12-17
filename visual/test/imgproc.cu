#include "imgfmt.h"
#include "imgproc.h"
#include <cmath>
#include "gpu_util.h"
#include <cassert>
#include <cstdio>

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
      _i = idy + my - i;
      _j = idx + mx - j;
      if (_i >= 0 && _i < F_n_rows && _j >= 0 && _j < F_n_cols) {
        total += H[IJ2C(i, j, H_n_rows)] * F[IJ2C(_i, _j, F_n_rows)];
        weight += H[IJ2C(i, j, H_n_rows)];
      }
    }
  }
  G[IJ2C(idy, idx, F_n_rows)] = total / weight;
}

gcube gpu_conv2(const gcube &F, const gcube &K) {
  gcube G(F.n_rows, F.n_cols);
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((F.n_cols-1)/16+1, (F.n_rows-1)/16+1, 1);
  // call the GPU kernel
  GPU_conv2_gen<<<gridSize, blockSize>>>(
      G.d_pixels, F.d_pixels, K.d_pixels,
      F.n_rows, F.n_cols, K.n_rows, K.n_cols);
  checkCudaErrors(cudaGetLastError());
  return G;
}

__global__ void GPU_conv2_sym(float *G, float *F, float *Hy, float *Hx, int F_n_rows, int F_n_cols, int H_n_rows, int H_n_cols) {
  // assume that the matrix represents an image
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  int idy = blockIdx.y * blockDim.y + threadIdx.y;
  if (idx >= F_n_cols || idy >= F_n_rows) {
    return;
  }
  // stencil operation
  int my = H_n_rows / 2;
  int mx = H_n_cols / 2;

  // for now just use a for loop
  float total = 0.0f;
  float weight = 0.0f;
  int _i, _j, i, j;
  for (i = 0; i < H_n_rows; i++) {
    _i = idy + my - i;
    if (_i >= 0 && _i < F_n_rows) {
      total += Hy[i] * F[IJ2C(_i, idx, F_n_rows)];
      weight += Hy[i];
    }
  }
  G[IJ2C(idy, idx, F_n_rows)] = total / weight;
  __syncthreads();
  total = 0.0f;
  weight = 0.0f;
  for (j = 0; j < H_n_cols; j++) {
    _j = idx + mx - j;
    if (_j >= 0 && _j < F_n_cols) {
      total += Hx[j] * G[IJ2C(idy, _j, F_n_rows)];
      weight += Hx[j];
    }
  }
  G[IJ2C(idy, idx, F_n_rows)] = total / weight;
}

gcube gpu_conv2(const gcube &F, const gcube &V, const gcube &H) {
  gcube G(F.n_rows, F.n_cols);
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((F.n_cols-1)/16+1, (F.n_rows-1)/16+1, 1);
  // call the GPU kernel
  GPU_conv2_sym<<<gridSize, blockSize>>>(
      G.d_pixels, F.d_pixels, V.d_pixels, H.d_pixels,
      F.n_rows, F.n_cols, V.n_rows, H.n_cols);
  checkCudaErrors(cudaGetLastError());
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
