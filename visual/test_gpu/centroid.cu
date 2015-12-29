#include "gpu_util.h"
#include "centroid.h"

/** First we want to calculate a histogram of columns for now
 *  After that, use the centroid algorithm (center of mass)
 *  Weighted sum / sum
 */

__global__ void GPU_hist_colsum(float *image,
    int n_rows, int n_cols, int offset, int shmemsize, int weighted) {
  int i = blockIdx.y * blockDim.y + threadIdx.y;
  int j = blockIdx.x * blockDim.x + threadIdx.x;
  if ((i << offset) >= n_rows || j >= n_cols) {
    return;
  }

  // store in shared memory due to speed
  extern __shared__ float interim[];
  int idx = i % shmemsize;
  interim[idx] = image[IJ2C(i << offset, j, n_rows)] * (idx * weighted + !weighted); // skip elements for coalescing
  __syncthreads();

  // summation happens // (for a power of 2)
  int length = MIN(shmemsize, n_rows - (i - idx));
  int idt = idx + 1;
  for (int partition = 2; (idt < length) & (idx % partition == 0); partition <<= 1) {
    interim[idx] += interim[idt];
    __syncthreads();
    idt = idx + partition;
  }

  if (idx == 0) {
    image[IJ2C(i, j, n_rows)] = interim[idx];
  }
}

__global__ void GPU_hist_rowsum(float *image,
    int n_rows, int n_cols, int offset, int shmemsize, int weighted) {
  int i = blockIdx.y * blockDim.y + threadIdx.y;
  int j = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n_rows || (j << offset) >= n_cols) {
    return;
  }

  // store in shared memory due to speed
  extern __shared__ float interim[];
  int idx = j % shmemsize;
  interim[idx] = image[IJ2C(i, j << offset, n_rows)] * (idx * weighted + !weighted); // skip elements for coalescing
  __syncthreads();

  // summation happens // (for a power of 2)
  int length = MIN(shmemsize, n_cols - (j - idx));
  int idt = idx + 1;
  for (int partition = 2; (idt < length) & (idx % partition == 0); partition <<= 1) {
    interim[idx] += interim[idt];
    __syncthreads();
    idt = idx + partition;
  }

  if (idx == 0) {
    image[IJ2C(i, j, n_rows)] = interim[idx];
  }
}

__global__ void GPU_memcpy_strided(float *G, float *F, int length, int stride, int offset) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= length) {
    return;
  }
  G[i] = F[offset + i * stride];
}

void gpu_hist(const gcube &F, gcube &H, bool rowhist, bool weighted) {
  // create a temporary memory space for the image
  gcube G;
  size_t glen;
  size_t blen;
  G.copy(F);

  if (!rowhist) {
    glen = G.n_rows;
    for (int i = 0; (1 << i) <= G.n_rows; i += 8) {
      blen = MIN(256, glen);
      glen = ((glen - 1) >> 8) + 1;
      dim3 blockSize(1, blen, 1);
      dim3 gridSize(G.n_cols, glen, 1);
      GPU_hist_colsum<<<gridSize, blockSize, sizeof(float) << 8>>>(G.d_pixels, G.n_rows, G.n_cols, i, blen, weighted ? 1 : 0);
      checkCudaErrors(cudaGetLastError());
    }
    H.create(G.n_cols);
    dim3 bs(256, 1, 1);
    dim3 gs((G.n_cols-1)/256+1, 1, 1);
    GPU_memcpy_strided<<<gs, bs>>>(H.d_pixels, G.d_pixels, G.n_cols, G.n_rows, 0);
    checkCudaErrors(cudaGetLastError());
  } else {
    glen = G.n_cols;
    for (int i = 0; (1 << i) <= G.n_cols; i += 8) {
      blen = MIN(256, glen);
      glen = ((glen - 1) >> 8) + 1;
      dim3 blockSize(blen, 1, 1);
      dim3 gridSize(glen, G.n_rows, 1);
      GPU_hist_rowsum<<<gridSize, blockSize, sizeof(float) << 8>>>(G.d_pixels, G.n_rows, G.n_cols, i, blen, weighted ? 1 : 0);
      checkCudaErrors(cudaGetLastError());
    }
    H.create(G.n_rows);
    checkCudaErrors(cudaMemcpy(H.d_pixels, G.d_pixels, sizeof(float) * G.n_rows, cudaMemcpyDeviceToDevice));
  }
}

void gpu_centroid(const gcube &F, double &x, double &y) { // make more efficient later on by getting rid of the ENTIRE copies of F
  gcube V, wV, H, wH;

  gpu_hist(F, V, true, false);
  gpu_hist(F, wV, true, false);
  gpu_hist(F, H, false, false);
  gpu_hist(F, wH, false, false);


  // calculate the centroids
  gcube t, w;
  gpu_hist(V, t, false, false);
  gpu_hist(wV, w, false, true);

  y = w.arma_cube()(0, 0, 0) / t.arma_cube()(0, 0, 0);
  
  gpu_hist(H, t, false, false);
  gpu_hist(wH, w, false, true);

  x = w.arma_cube()(0, 0, 0) / t.arma_cube()(0, 0, 0);
}
