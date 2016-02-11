#include <opencv2/highgui/highgui.hpp>
#include <cstdio>
#include <cassert>
#include "highgui.h"
#include "gpu_util.h"

gcube load_gcube(const std::string &image_name) {
  gcube G;
  G.load(image_name);
  return G;
}

void save_gcube(const std::string &image_name, gcube &image) {
  image.save(image_name);
}

void print_gcube(gcube &image) {
  float *mem = new float[image.n_elem];
  checkCudaErrors(cudaMemcpy(mem, image.d_pixels, sizeof(float) * image.n_elem, cudaMemcpyDeviceToHost));
  for (size_t k = 0; k < image.n_slices; k++) {
    printf("slice %zu\n", k);
    for (size_t i = 0; i < image.n_rows; i++) {
      for (size_t j = 0; j < image.n_cols; j++) {
        printf("%f, ", mem[IJK2C(i, j, k, image.n_rows, image.n_cols)]);
      }
      printf("\n");
    }
    printf("\n");
  }
  delete mem;
}

void disp_gcube(const std::string &window_name, gcube &image) {
  cv::namedWindow(window_name);
  cv::Mat I = image.cv_img();
  cv::imshow(window_name, I);
}

void disp_wait(void) {
  cv::waitKey(0);
}

int disp_keyPressed(void) {
  return cv::waitKey(30);
}

__global__ void GPU_rgb2gray(float *G, float *F, int n_rows, int n_cols) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  int j = blockIdx.y * blockDim.y + threadIdx.y;
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
  dim3 gridSize((image.n_rows-1)/16+1, (image.n_cols-1)/16+1, 1);
  dim3 blockSize(16, 16, 1);
  GPU_rgb2gray<<<gridSize, blockSize>>>(
        G.d_pixels, image.d_pixels,
        image.n_rows, image.n_cols);
  checkCudaErrors(cudaGetLastError());
  return G;
}

gcube gpu_gray2rgb(const gcube &image) {
  assert(image.n_slices == 1);
  gcube G(image.n_rows, image.n_cols, 3);
  checkCudaErrors(cudaMemcpy(&G.d_pixels[IJK2C(0, 0, 0, G.n_rows, G.n_cols)],
        image.d_pixels, sizeof(float) * image.n_elem, cudaMemcpyDeviceToDevice));
  checkCudaErrors(cudaMemcpy(&G.d_pixels[IJK2C(0, 0, 1, G.n_rows, G.n_cols)],
        image.d_pixels, sizeof(float) * image.n_elem, cudaMemcpyDeviceToDevice));
  checkCudaErrors(cudaMemcpy(&G.d_pixels[IJK2C(0, 0, 2, G.n_rows, G.n_cols)],
        image.d_pixels, sizeof(float) * image.n_elem, cudaMemcpyDeviceToDevice));
  return G;
}
