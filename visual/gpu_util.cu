#include "gpu_util.h"

__global__ void GPU_addI(float *H, float *F, float I, int n_rows, int n_cols) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  int j = blockIdx.y * blockDim.y + threadIdx.y;
  if (i >= n_rows || j >= n_cols) {
    return;
  }
  H[IJ2C(i, j, n_rows)] = F[IJ2C(i, j, n_rows)] + I;
}

__global__ void GPU_subI(float *H, float *F, float I, int n_rows, int n_cols) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  int j = blockIdx.y * blockDim.y + threadIdx.y;
  if (i >= n_rows || j >= n_cols) {
    return;
  }
  H[IJ2C(i, j, n_rows)] = F[IJ2C(i, j, n_rows)] - I;
}

__global__ void GPU_mulI(float *H, float *F, float I, int n_rows, int n_cols) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  int j = blockIdx.y * blockDim.y + threadIdx.y;
  if (i >= n_rows || j >= n_cols) {
    return;
  }
  H[IJ2C(i, j, n_rows)] = F[IJ2C(i, j, n_rows)] * I;
}

__global__ void GPU_divI(float *H, float *F, float I, int n_rows, int n_cols) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  int j = blockIdx.y * blockDim.y + threadIdx.y;
  if (i >= n_rows || j >= n_cols) {
    return;
  }
  H[IJ2C(i, j, n_rows)] = F[IJ2C(i, j, n_rows)] / I;
}

__global__ void GPU_add(float *H, float *F, float *G, int n_rows, int n_cols) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  int j = blockIdx.y * blockDim.y + threadIdx.y;
  if (i >= n_rows || j >= n_cols) {
    return;
  }
  H[IJ2C(i, j, n_rows)] = F[IJ2C(i, j, n_rows)] + G[IJ2C(i, j, n_rows)];
}

__global__ void GPU_sub(float *H, float *F, float *G, int n_rows, int n_cols) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  int j = blockIdx.y * blockDim.y + threadIdx.y;
  if (i >= n_rows || j >= n_cols) {
    return;
  }
  H[IJ2C(i, j, n_rows)] = F[IJ2C(i, j, n_rows)] - G[IJ2C(i, j, n_rows)];
}

__global__ void GPU_mul(float *H, float *F, float *G, int n_rows, int n_cols) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  int j = blockIdx.y * blockDim.y + threadIdx.y;
  if (i >= n_rows || j >= n_cols) {
    return;
  }
  H[IJ2C(i, j, n_rows)] = F[IJ2C(i, j, n_rows)] * G[IJ2C(i, j, n_rows)];
}

__global__ void GPU_div(float *H, float *F, float *G, int n_rows, int n_cols) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  int j = blockIdx.y * blockDim.y + threadIdx.y;
  if (i >= n_rows || j >= n_cols) {
    return;
  }
  H[IJ2C(i, j, n_rows)] = F[IJ2C(i, j, n_rows)] / G[IJ2C(i, j, n_rows)];
}

__global__ void GPU_trans(float *H, float *F, int n_rows, int n_cols) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  int j = blockIdx.y * blockDim.y + threadIdx.y;
  if (i >= n_rows || j >= n_cols) {
    return;
  }
  H[IJ2C(j, i, n_cols)] = F[IJ2C(i, j, n_rows)];
}

__global__ void GPU_mmul(float *H, float *F, float *G, int n_rows, int n_cols, int n_inner) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  int j = blockIdx.y * blockDim.y + threadIdx.y;
  int k = blockIdx.z * blockDim.z + threadIdx.z;
  if (i >= n_rows || j >= n_cols || k >= n_inner) {
    return;
  }
  H[IJK2C(k, j, i, n_inner, n_cols)] = F[IJ2C(i, k, n_rows)] * G[IJ2C(k, j, n_inner)];
}

__global__ void GPU_abs(float *H, float *F, int n_rows, int n_cols) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  int j = blockIdx.y * blockDim.y + threadIdx.y;
  if (i >= n_rows || j >= n_cols) {
    return;
  }
  int v = F[IJ2C(i, j, n_rows)];
  H[IJ2C(i, j, n_rows)] = v * ((v >= 0) - (v < 0));
}

__global__ void GPU_sum(float *H, float *F, int n_rows, int n_cols, int shmsize, int offset) {
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  int idy = blockIdx.y * blockDim.y + threadIdx.y;
  if ((idx << offset) >= n_rows || idy >= n_cols) {
    return;
  }
  extern __shared__ float arr[];
  int i = idx % shmsize;
  arr[i] = F[IJ2C((idx << offset), idy, n_rows)];
  __syncthreads();

  int j;
  int gap = (n_rows>>offset)-(idx-i);
  for (int ptn = 2; (j = i + (ptn >> 1)) < shmsize; ptn <<= 1) {
    if (i % ptn != 0 || j >= gap) {
      break;
    }
    arr[i] += arr[j];
    __syncthreads();
  }

  if (i == 0) {
    H[IJ2C(idx, idy, n_rows)] = arr[i];
  }
}

__global__ void GPU_copyRow(float *H, float *F, int n_rows, int rowid) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n_rows) {
    return;
  }
  H[i] = F[IJ2C(rowid, i, n_rows)];
}
