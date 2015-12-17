#include <opencv2/highgui/highgui.hpp>
#include "gcube.h"
#include "gpu_util.h"

gcube::gcube(void) {
  this->d_pixels = NULL;
  this->create(0, 0, 0, gfill::none);
}

gcube::gcube(size_t n_rows, size_t n_cols, size_t n_slices, uint8_t fill_type) {
  this->d_pixels = NULL;
  this->create(n_rows, n_cols, n_slices, fill_type);
}

gcube::gcube(const gcube &gpucube) {
  this->d_pixels = NULL;
  this->create(gpucube.n_rows, gpucube.n_cols, gpucube.n_slices, gfill::none);
  checkCudaErrors(cudaMemcpy(this->d_pixels, gpucube.d_pixels, this->n_elem * sizeof(float), cudaMemcpyDeviceToDevice));
}

gcube::gcube(const std::string &fname) {
  this->d_pixels = NULL;
  this->load(fname);
}

gcube::~gcube(void) {
  if (this->d_pixels) {
    checkCudaErrors(cudaFree(this->d_pixels));
  }
}

__global__ void GPU_map_assign(float *F, float val, size_t n_elems) {
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= n_elems) {
    return;
  }
  F[idx] = val;
}

void gcube::create(size_t n_rows, size_t n_cols, size_t n_slices, uint8_t fill_type) {
  if (this->d_pixels) {
    checkCudaErrors(cudaFree(d_pixels));
  }
  this->n_rows = n_rows;
  this->n_cols = n_cols;
  this->n_slices = n_slices;
  this->n_elem = n_rows * n_cols * n_slices;
  if (this->n_elem == 0) {
    this->d_pixels = NULL;
  } else {
    checkCudaErrors(cudaMalloc(&this->d_pixels, this->n_elem * sizeof(float)));
    switch (fill_type) {
      case gfill::none:
        break;
      case gfill::zeros:
        checkCudaErrors(cudaMemset(&this->d_pixels, 0, this->n_elem * sizeof(float)));
        break;
      case gfill::ones:
        GPU_map_assign<<<(this->n_elem-1) / 128 + 1, 128>>>(this->d_pixels, 1, this->n_elem);
        checkCudaErrors(cudaGetLastError());
        break;
      default:
        break;
    }
  }
}

gcube &gcube::operator=(const gcube &gpucube) {
  this->create(gpucube.n_rows, gpucube.n_cols, gpucube.n_slices, gfill::none);
  checkCudaErrors(cudaMemcpy(this->d_pixels, gpucube.d_pixels, this->n_elem * sizeof(float), cudaMemcpyDeviceToDevice));
  return *this;
}

void gcube::load(const std::string &fname) {
  this->create(cv::imread(fname));
}

void gcube::save(const std::string &fname) {
  cv::imwrite(fname, this->cv_mat());
}

// Specific OpenCV interaction (to make sure that they are backwards compatible)

gcube::gcube(cv::Mat &cvMat) {
  this->d_pixels = NULL;
  this->create(cvMat);
}

void gcube::create(const cv::Mat &cvMat) {
  this->create(cvMat.rows, cvMat.cols, cvMat.channels(), gfill::none);
  float *h_pixels = new float[this->n_elem];
  for (int i = 0; i < this->n_rows; i++) {
    for (int j = 0; j < this->n_cols; j++) {
      cv::Vec3b color = cvMat.at<cv::Vec3b>(i, j);
      for (int k = 0; k < this->n_slices; k++) {
        h_pixels[IJK2C(i, j, k, this->n_rows, this->n_cols)] = (float)color[k] / 255.0f;
      }
    }
  }
  checkCudaErrors(cudaMemcpy(this->d_pixels, h_pixels, this->n_elem * sizeof(float), cudaMemcpyHostToDevice));
  free(h_pixels);
}

cv::Mat gcube::cv_mat(void) {
  cv::Mat cv_image(this->n_rows, this->n_cols, CV_8UC3);
  float *h_pixels = new float[this->n_elem];
  checkCudaErrors(cudaMemcpy(h_pixels, this->d_pixels, this->n_elem * sizeof(float), cudaMemcpyDeviceToHost));
  for (int i = 0; i < this->n_rows; i++) {
    for (int j = 0; j < this->n_cols; j++) {
      if (this->n_slices == 1) {
        cv_image.at<cv::Vec3b>(i, j) =
          cv::Vec3b((int)(h_pixels[IJ2C(i, j, this->n_rows)] * 255.0f),
                    (int)(h_pixels[IJ2C(i, j, this->n_rows)] * 255.0f),
                    (int)(h_pixels[IJ2C(i, j, this->n_rows)] * 255.0f));
      } else if (this->n_slices == 3) {
        cv_image.at<cv::Vec3b>(i, j) =
          cv::Vec3b((int)(h_pixels[IJK2C(i, j, 0, this->n_rows, this->n_cols)] * 255.0f),
                    (int)(h_pixels[IJK2C(i, j, 1, this->n_rows, this->n_cols)] * 255.0f),
                    (int)(h_pixels[IJK2C(i, j, 2, this->n_rows, this->n_cols)] * 255.0f));
      }
    }
  }
  free(h_pixels);
  return cv_image;
}

gcube &gcube::operator=(const cv::Mat &cvMat) {
  this->create(cvMat);
  return *this;
}
