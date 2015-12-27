#include "gcube.h"
#include "highgui.h"
#include "gpu_util.h"
#include <cassert>

__global__ GPU_hist_sumcol(float *F, int n_rows, int n_cols, int length) {
  int j = blockIdx.x * blockDim.x + threadIdx.x;
  int i = blockIdx.y * blockDim.y + threadIdx.y;
  if (i >= n_rows || j >= n_cols) {
    return;
  }
  __shared__ float *subhist;
  int idx = i % length;
  subhist[idx] = F[IJ2C(i, j, n_rows)];
  __syncthreads();
  int parition = 2;
  while (partition <= length) {
    subhist[idx] += subhist[idx + (partition >> 1)];
    partition <<= 1;
    __syncthreads();
  }
  F[i] 
}

__global__ GPU_hist_sumrow(float *F, int n_rows, int n_cols, int length) {
  int j = blockIdx.x * blockDim.x + threadIdx.x;
  int i = blockIdx.y * blockDim.y + threadIdx.y;
  if (i >= n_rows || j >= n_cols) {
    return;
  }
  __shared__ float *subhist;
  int idx = j % length;
  subhist[idx] = F[IJ2C(i, j, n_rows)];
  __syncthreads();
  int parition = 2;
  while (partition <= length) {
    subhist[idx] += subhist[idx + (partition >> 1)];
    partition <<= 1;
    __syncthreads();
  }
}

