#include "gpu_util.h"
#include "centroid.h"

/** First we want to calculate a histogram of columns for now
 *  After that, use the centroid algorithm (center of mass)
 *  Weighted sum / sum
 */

__global__ GPU_hist_colsum(float *image,
    int n_rows, int n_cols, int shared_mem_size) {
  int rowid = blockIdx.y * blockDim.y + threadIdx.y;
  int colid = blockIdx.x * blockDim.x + threadIdx.x;
  
}
