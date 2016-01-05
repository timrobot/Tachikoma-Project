#include "gpu_util.h"

__global__ void GPU_addI(float *H, float *F, float I, int n_rows, int n_cols) {
  int i = blockIdx.y * blockDim.y + threadIdx.y;
  int j = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n_rows || j >= n_cols) {
    return;
  }
  H[IJ2C(i, j, n_rows)] = F[IJ2C(i, j, n_rows)] + I;
}

__global__ void GPU_subI(float *H, float *F, float I, int n_rows, int n_cols) {
  int i = blockIdx.y * blockDim.y + threadIdx.y;
  int j = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n_rows || j >= n_cols) {
    return;
  }
  H[IJ2C(i, j, n_rows)] = F[IJ2C(i, j, n_rows)] - I;
}

__global__ void GPU_mulI(float *H, float *F, float I, int n_rows, int n_cols) {
  int i = blockIdx.y * blockDim.y + threadIdx.y;
  int j = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n_rows || j >= n_cols) {
    return;
  }
  H[IJ2C(i, j, n_rows)] = F[IJ2C(i, j, n_rows)] * I;
}

__global__ void GPU_divI(float *H, float *F, float I, int n_rows, int n_cols) {
  int i = blockIdx.y * blockDim.y + threadIdx.y;
  int j = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n_rows || j >= n_cols) {
    return;
  }
  H[IJ2C(i, j, n_rows)] = F[IJ2C(i, j, n_rows)] / I;
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

__global__ void GPU_trans(float *H, float *F, int n_rows, int n_cols) {
  int i = blockIdx.y * blockDim.y + threadIdx.y;
  int j = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n_rows || j >= n_cols) {
    return;
  }
  H[IJ2C(j, i, n_cols)] = F[IJ2C(i, j, n_rows)];
}

//__global__ void GPU_mmul(float *H, float *F, float *G, int n_rows, int n_cols, int n_inner) {
  
//}

__global__ void GPU_abs(float *H, float *F, int n_rows, int n_cols) {
  int i = blockIdx.y * blockDim.y + threadIdx.y;
  int j = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n_rows || j >= n_cols) {
    return;
  }
  int v = F[IJ2C(i, j, n_rows)];
  H[IJ2C(i, j, n_rows)] = v * ((v >= 0) - (v < 0));
}

__global__ void GPU_sum(float *G, float *F, int n) { // TODO: fix
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
