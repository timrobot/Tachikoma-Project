#include "gpu_util.h"

__global__ void GPU_sum(float *G, float *F, int n) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) {
    return;
  }
  double sum = F[i];
  for (int j = 1; (i % (j >> 1)) == 0 && (i + j) < n; j >>= 1) {
    sum += F[i + j];
    __syncthreads();
  }
  if (i == 0) {
    *G = sum;
  }
}

__global__ void GPU_add(float *H, float *F, float *G, int n_rows, int n_cols) {
  int i = blockIdx.y * blockDim.y + threadIdx.y;
  int j = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n_rows || j >= n_cols) {
    return;
  }
  H[IJ2C(i, j, n_rows)] = F[IJ2C(i, j, n_rows)] + G[IJ2C(i, j, n_rows)];
}

__global__ void GPU_sub(float *H, float *F, float *G, int n_rows, int n_cols) {
  int i = blockIdx.y * blockDim.y + threadIdx.y;
  int j = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n_rows || j >= n_cols) {
    return;
  }
  H[IJ2C(i, j, n_rows)] = F[IJ2C(i, j, n_rows)] - G[IJ2C(i, j, n_rows)];
}

__global__ void GPU_mul(float *H, float *F, float *G, int n_rows, int n_cols) {
  int i = blockIdx.y * blockDim.y + threadIdx.y;
  int j = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n_rows || j >= n_cols) {
    return;
  }
  H[IJ2C(i, j, n_rows)] = F[IJ2C(i, j, n_rows)] * G[IJ2C(i, j, n_rows)];
}

__global__ void GPU_div(float *H, float *F, float *G, int n_rows, int n_cols) {
  int i = blockIdx.y * blockDim.y + threadIdx.y;
  int j = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n_rows || j >= n_cols) {
    return;
  }
  H[IJ2C(i, j, n_rows)] = F[IJ2C(i, j, n_rows)] / G[IJ2C(i, j, n_rows)];
}

__global__ void GPU_abs(float *H, float *F, int n_rows, int n_cols) {
  int i = blockIdx.y * blockDim.y + threadIdx.y;
  int j = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n_rows || j >= n_cols) {
    return;
  }
  int v = F[IJ2C(i, j, n_rows)];
  H[IJ2C(i, j, n_rows)] = v * ((v >= 0) - (v < 0));
}
