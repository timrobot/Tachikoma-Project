#include "highgui.h"
#include "imgproc.h"
#include "gpu_util.h"

//const float u_distortion[4] = { 1.0, -0.22, -0.24, 0 };
// preprogrammed constants
#define UD0 1.0
#define UD1 0.22
#define UD2 0.24

__global__ void barrel_distort_ovr(float *G, float *F,
    int n_rows, int n_cols, int n_slices,
    float r_x, float r_y, float r_max, float offset_x, int right_image) {
  int j = blockIdx.x * blockDim.x + threadIdx.x;
  int i = blockIdx.y * blockDim.y + threadIdx.y;
  if (i >= n_rows || j >= n_cols) {
    return;
  }
  float x = (float)(j-r_x) / r_max + offset_x;
  float y = (float)(i-r_y) / r_max;
  // distortionScale
  float rr = x*x+y*y;
  float distortion = UD0+UD1*rr+UD2*rr*rr;
  int _i = (int)roundf(distortion*y*r_max+r_y);
  int _j = (int)roundf((distortion*x-offset_x)*r_max+r_x);
  for (int k = 0; k < n_slices; k++) {
    // since they are stored right next to each other, the slices are the separating factor (k*2+right_image)
    if (_i >= 0 && _i < n_rows && _j >= 0 && _j < n_cols) {
      G[IJK2C(i, j, k*2+right_image, n_rows, n_cols)] = F[IJK2C(_i, _j, k, n_rows, n_cols)];
    }
  }
}

gcube ovr_image(const gcube &left, const gcube &right, double offset_x) {
  // assume that the left and right images have equal dimensions
  assert(left.n_rows == right.n_rows && left.n_rows == right.n_rows);
  double mrx = left.n_cols / 2.0 * (1 + abs(offset_x));
  double mry = right.n_cols / 2.0;
  float r_max = (float)sqrt(mrx * mrx + mry * mry);
  float r_x = left.n_cols / 2.0f;
  float r_y = left.n_rows / 2.0f;
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((left.n_cols-1)/16+1, (left.n_rows-1)/16+1, 1);
  gcube C(left.n_rows, left.n_cols + right.n_cols, left.n_slices, gfill::zeros);
  barrel_distort_ovr<<<gridSize, blockSize>>>(C.d_pixels, left.d_pixels,
      left.n_rows, left.n_cols, left.n_slices,
      r_x, r_y, r_max, -offset_x, 0);
  checkCudaErrors(cudaGetLastError());
  barrel_distort_ovr<<<gridSize, blockSize>>>(C.d_pixels, right.d_pixels,
      right.n_rows, right.n_cols, right.n_slices,
      r_x, r_y, r_max, offset_x, 1);
  checkCudaErrors(cudaGetLastError());
  return gpu_imresize2(C, 800, 1280);
}
