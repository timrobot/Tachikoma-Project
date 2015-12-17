#include <opencv2/highgui/highgui.hpp>
#include <cstdio>
#include <cassert>
#include "imgfmt.h"
#include "gpu_util.h"

void disp_gcube(const std::string &window_name, gcube &image) {
  cv::namedWindow(window_name);
  cv::imshow(window_name, image.cv_mat());
}

void disp_wait(void) {
  cv::waitKey(0);
}

__global__ void GPU_rgb2gray(float *G, float *F, int n_rows, int n_cols) {
  int j = blockIdx.x * blockDim.x + threadIdx.x;
  int i = blockIdx.y * blockDim.y + threadIdx.y;
  if (i >= n_rows || j >= n_cols) {
    return;
  }
  float red = F[IJK2C(i, j, 0, n_rows, n_cols)];
  float green = F[IJK2C(i, j, 1, n_rows, n_cols)];
  float blue = F[IJK2C(i, j, 2, n_rows, n_cols)];
  G[IJ2C(i, j, n_rows)] = red * 0.3f + green * 0.6f + blue * 0.1f;
}

gcube gpu_rgb2gray(const gcube &image) {
  assert(image.n_slices == 3);
  gcube G(image.n_rows, image.n_cols, 1);
  dim3 gridSize((image.n_cols-1)/16+1, (image.n_rows-1)/16+1, 1);
  dim3 blockSize(16, 16, 1);
  GPU_rgb2gray<<<gridSize, blockSize>>>(
        G.d_pixels, image.d_pixels,
        image.n_rows, image.n_cols);
  checkCudaErrors(cudaGetLastError());
  return G;
}
