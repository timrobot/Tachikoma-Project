#include "gpu_util.h"
#include "centroid.h"

/** First we want to calculate a histogram of columns for now
 *  After that, use the centroid algorithm (center of mass)
 *  Weighted sum / sum
 */

__global__ GPU_hist_colsum(float *image,
    int n_rows, int n_cols, int shmemsize) {
  int rowid = blockIdx.y * blockDim.y + threadIdx.y;
  int colid = blockIdx.x * blockDim.x + threadIdx.x;

  // if it goes out of bounds, then return
  if (rowid >= n_rows || colid >= n_cols) {
    return;
  }

  // store in shared memory due to speed
  __shared__ float *interim; // allocation done on the gpu side via CPU kernel call

  // copy over the element into the interim
  int idx = rowid % shmemsize;
  interim[idx] = image[IJ2C(rowid, colid, n_rows)];
  __syncthreads();

  // summation happens // (for a power of 2)
  for (int parition = 2; idx + partition <= shmemsize; partition <<= 1) {
    if (idx % partition != 0) {
      break;
    }
    int a = idx;
    int b = idx + (partition >> 1);
    interim[a] += interim[b];
    __syncthreads();
  }

  if (idx == 0) {
    image[IJ2C(rowid, colid, n_rows)] = interim[idx];
  }
}

__global__ GPU_coalesce_col(float *image,
    int n_rows, int n_cols, int shmemsize) {
  int rowid = blockIdx.y * blockDim.y + threadIdx.y;
  int colid = blockIdx.x * blockDim.x + threadIdx.x;

  // if it goes out of bounds, then return
  if (rowid >= n_rows || colid >= n_cols) {
    return;
  }

  // coalesce (shmemsize * shmemsize < n_rows) => __shared__ mem
  image[IJ2C(rowid, colid, n_rows)] = image[IJ2C(rowid * shmemsize, colid, n_rows)];
}

__global__ GPU_hist_rowsum(float *image,
    int n_rows, int n_cols, int shmemsize) {
  int rowid = blockIdx.y * blockDim.y + threadIdx.y;
  int colid = blockIdx.x * blockDim.x + threadIdx.x;

  // if it goes out of bounds, then return
  if (rowid >= n_rows || colid >= n_cols) {
    return;
  }

  // store in shared memory due to speed
  __shared__ float *interim; // allocation done on the gpu side via CPU kernel call

  // copy over the element into the interim
  int idx = colid % shmemsize;
  interim[idx] = image[IJ2C(rowid, colid, n_rows)];
  __syncthreads();

  // summation happens // (for a power of 2)
  for (int parition = 2; idx + partition <= shmemsize; partition <<= 1) {
    if (idx % partition != 0) {
      break;
    }
    int a = idx;
    int b = idx + (partition >> 1);
    interim[a] += interim[b];
    __syncthreads();
  }

  if (idx == 0) {
    image[IJ2C(rowid, colid, n_rows)] = interim[idx];
  }
}

__global__ GPU_coalesce_col(float *image,
    int n_rows, int n_cols, int shmemsize) {
  int rowid = blockIdx.y * blockDim.y + threadIdx.y;
  int colid = blockIdx.x * blockDim.x + threadIdx.x;

  // if it goes out of bounds, then return
  if (rowid >= n_rows || colid >= n_cols) {
    return;
  }

  // coalesce (shmemsize * shmemsize < n_rows) => __shared__ mem
  image[IJ2C(rowid, colid, n_rows)] = image[IJ2C(rowid * shmemsize, colid, n_rows)];
}

void gpu_hist(gcube *F, gcube *V, gcube *H) {
  // create a temporary memory space for the image
  gcube G(F);
  gcube H(G.n_cols, G.n_rows);

  dim3 bs1(1, 256, 1);
  dim3 gs1(n_cols, (n_rows-1)/256+1, 1);
  GPU_hist_colsum<<<gs1, bs1, sizeof(float) * 256>>>(G->d_pixels, G->n_rows, G->n_cols, 256);
  checkCudaErrors(cudaGetLastError());

  // coalesce the data once (this only works for images with a max n_rows <= 2^16)
  int new_rows = (gs1.y-1)/32+1;
  dim3 bs2(1, new_rows, 1); // warp granularity of 32
  dim3 gs2(n_cols, 1, 1);
  GPU_coalesce_col<<<gs2, bs2>>>(G->d_pixels, n_rows, n_cols, 256);
  checkCudaErrors(cudaGetLastError());

  GPU_hist_colsum<<<gs2, bs2, sizeof(float) * new_rows>>>(G->d_pixels, new_rows, n_cols, new_rows);
  checkCudaErrors(cudaGetLastError());

  // too many operations, convert later to use less
  dim3 bs3(16, 16, 1);
  dim3 gs3((n_cols-1)/16+1, (n_rows-1)/16+1, 1);
  GPU_trans<<<gs3, bs3>>>(H->d_pixels, G->d_pixels, G->n_rows, G->n_cols);
  checkCudaErrors(cudaGetLastError());

  checkCudaErrors(cudaMemcpy(V, H, sizeof(float) * n_cols, cudaMemcpyDeviceToDevice));
}
