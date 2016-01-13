#include "gpu_util.h"
#include "centroid.h"
#include "highgui.h"
#include "gcube.h"
#include <armadillo>

__global__ void GPU_map_binary(size_t *B, float *F, int n_rows, int n_cols) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  int j = blockIdx.y * blockDim.y + threadIdx.y;
  if (i >= n_rows || j >= n_cols) {
    return;
  }
  B[IJ2C(i, j, n_rows)] = (F[IJ2C(i, j, n_rows)] > 0.0f);
}

/** First we want to calculate a histogram of columns for now
 *  After that, use the centroid algorithm (center of mass)
 *  Weighted sum / sum
 */

__global__ void GPU_colsum(size_t *F, int n_rows, int n_cols, int shmsize, int offset, bool weighted) {
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  int idy = blockIdx.y * blockDim.y + threadIdx.y;
  if ((idx << offset) >= n_rows || idy >= n_cols) {
    return;
  }
  extern __shared__ size_t arr[];
  int i = idx % shmsize;
  int coeff = weighted ? idx : 1;
  arr[i] = F[IJ2C((idx << offset), idy, n_rows)] * coeff;
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
    F[IJ2C(idx, idy, n_rows)] = arr[i];
  }
}

__global__ void GPU_rowsum(size_t *F, int n_rows, int n_cols, int shmsize, int offset, bool weighted) {
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  int idy = blockIdx.y * blockDim.y + threadIdx.y;
  if (idx >= n_rows || (idy << offset) >= n_cols) {
    return;
  }
  extern __shared__ size_t arr[];
  int i = idy % shmsize;
  int coeff = weighted ? idy : 1;
  arr[i] = F[IJ2C(idx, (idy << offset), n_rows)] * coeff;
  __syncthreads();

  int j;
  int gap = (n_cols>>offset)-(idy-i);
  for (int ptn = 2; (j = i + (ptn >> 1)) < shmsize; ptn <<= 1) {
    if (i % ptn != 0 || j >= gap) {
      break;
    }
    arr[i] += arr[j];
    __syncthreads();
  }

  if (i == 0) {
    F[IJ2C(idx, idy, n_rows)] = arr[i];
  }
}

__global__ void GPU_memcpy_strided(size_t *G, size_t *F, int length, int stride, int offset) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= length) {
    return;
  }
  G[i] = F[offset + i * stride];
}

void gpu_hist(size_t **H, size_t *F, size_t n_rows, size_t n_cols, bool normal, bool weighted) {
  // create a temporary memory space for the image

  if (normal) {
    dim3 blockSize(256, 1, 1);
    dim3 gridSize((n_rows-1)/256+1, n_cols, 1);
    for (int i = 0; (1 << i) < n_rows; i += 8) {
      GPU_colsum<<<gridSize, blockSize, sizeof(size_t) * blockSize.x>>>
        (F, n_rows, n_cols, blockSize.x, i, weighted ? 1 : 0);
      checkCudaErrors(cudaGetLastError());
      checkCudaErrors(cudaDeviceSynchronize());
      blockSize.x = MIN(gridSize.x, 256);
      gridSize.x = (blockSize.x-1)/256+1;
    }
    checkCudaErrors(cudaMalloc(H, sizeof(size_t) * n_cols));
    blockSize.x = 256;
    gridSize.x = (n_cols-1)/256+1;
    GPU_memcpy_strided<<<gridSize, blockSize>>>(*H, F, n_cols, n_rows, 0);
    checkCudaErrors(cudaGetLastError());
  } else {
    dim3 blockSize(1, 256, 1);
    dim3 gridSize(n_rows, (n_cols-1)/256+1, 1);
    for (int i = 0; (1 << i) < n_cols; i += 8) {
      GPU_rowsum<<<gridSize, blockSize, sizeof(size_t) * blockSize.y>>>
        (F, n_rows, n_cols, blockSize.y, i, weighted ? 1 : 0);
      checkCudaErrors(cudaGetLastError());
      blockSize.y = MIN(gridSize.y, 256);
      gridSize.y = (blockSize.y-1)/256+1;
    }
    checkCudaErrors(cudaMalloc(H, sizeof(size_t) * n_rows));
    checkCudaErrors(cudaMemcpy(*H, F, sizeof(size_t) * n_rows, cudaMemcpyDeviceToDevice));
  }
}

void gpu_centroid(const gcube &F, size_t &x, size_t &y) { // make more efficient later on by getting rid of the ENTIRE copies of F
  size_t *B, *C;
  size_t *V = NULL, *H = NULL;
  size_t *w = NULL, *t = NULL;
  size_t a, b;

  checkCudaErrors(cudaMalloc(&B, sizeof(size_t) * F.n_rows * F.n_cols));
  checkCudaErrors(cudaMalloc(&C, sizeof(size_t) * F.n_rows * F.n_cols));
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((F.n_rows-1)/16+1,(F.n_cols-1)/16+1,1);
  GPU_map_binary<<<gridSize, blockSize>>>(B, F.d_pixels, F.n_rows, F.n_cols);
  checkCudaErrors(cudaGetLastError());
  checkCudaErrors(cudaMemcpy(C, B, sizeof(size_t) * F.n_rows * F.n_cols, cudaMemcpyDeviceToDevice));

  // calculate horizontal
  gpu_hist(&H, B, F.n_rows, F.n_cols, true, false);
  gpu_hist(&t, H, F.n_cols, 1, true, false);
  gpu_hist(&w, H, F.n_cols, 1, true, true);

  checkCudaErrors(cudaMemcpy(&a, w, sizeof(size_t), cudaMemcpyDeviceToHost));
  checkCudaErrors(cudaMemcpy(&b, t, sizeof(size_t), cudaMemcpyDeviceToHost));
  if (b == 0) { // this exception will only occur once
    x = 0;
    y = 0;
    checkCudaErrors(cudaFree(H));
    checkCudaErrors(cudaFree(w));
    checkCudaErrors(cudaFree(t));
    checkCudaErrors(cudaFree(B));
    checkCudaErrors(cudaFree(C));
    return;
  }
  x = a / b;
  checkCudaErrors(cudaFree(H));
  checkCudaErrors(cudaFree(w));
  checkCudaErrors(cudaFree(t));
  checkCudaErrors(cudaFree(B));

  // calculate vertical
  gpu_hist(&V, C, F.n_rows, F.n_cols, false, false);
  gpu_hist(&t, V, F.n_rows, 1, true, false);
  gpu_hist(&w, V, F.n_rows, 1, true, true);

  checkCudaErrors(cudaMemcpy(&a, w, sizeof(size_t), cudaMemcpyDeviceToHost));
  checkCudaErrors(cudaMemcpy(&b, t, sizeof(size_t), cudaMemcpyDeviceToHost));
  y = a / b;
  checkCudaErrors(cudaFree(V));
  checkCudaErrors(cudaFree(w));
  checkCudaErrors(cudaFree(t));
  checkCudaErrors(cudaFree(C));
}
