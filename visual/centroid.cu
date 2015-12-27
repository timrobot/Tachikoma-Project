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
  int idx = colid % shmemsize;
  interim[idx] = image[IJ2C(rowid, colid, n_rows)];
  __syncthreads();

  // summation happens // (for a power of 2)
  for (int parition = 2; partition <= shmemsize; partition <<= 1) {
    if (idx % partition != 0) {
      break;
    }
    int a = idx;
    int b = idx + (partition >> 1);
    if (b < shmemsize
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

void gpu_hist(float *image, float *colhist, int n_rows, int n_cols) {
  // create a temporary memory space for the image
  float *temp_image;
  checkCudaErrors(cudaMalloc(&temp_image, sizeof(float) * n_rows * n_cols));
  checkCudaErrors(cudaMalloc(temp_image, image, sizeof(float) * n_rows * n_cols, cudaMemcpyDeviceToDevice));

  dim3 blockSize(1, 256, 1);
  dim3 gridSize(n_rows, (n_cols - 1) % 256 + 1, 1);
  GPU_hist_colsum<<<gridSize, blockSize, sizeof(float) * 256>>>(temp_image, n_rows, n_cols, 256);
  checkCudaErrors(cudaGetLastError());

  // coalesce the data
  GPU_coalesce_col<<<gridSize, blockSize>>>(temp_image, n_rows, n_cols, 256);
  checkCudaErrors(cudaGetLastError());

  checkCudaErrors(cudaMemcpy(colhist, temp_image, sizeof(float) * n_cols, cudaMemcpyDeviceToDevice));
  checkCudaErrors(cudaFree(temp_image));
}
