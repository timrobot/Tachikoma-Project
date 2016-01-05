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

/*gcube::gcube(const gcube &gpucube) {
  this->d_pixels = NULL;
  this->create(gpucube.n_rows, gpucube.n_cols, gpucube.n_slices, gfill::none);
  checkCudaErrors(cudaMemcpy(this->d_pixels, gpucube.d_pixels, this->n_elem * sizeof(float), cudaMemcpyDeviceToDevice));
}

gcube::gcube(const gcube *gpucube) {
  this->d_pixels = NULL;
  this->create(gpucube->n_rows, gpucube->n_cols, gpucube->n_slices, gfill::none);
  checkCudaErrors(cudaMemcpy(this->d_pixels, gpucube->d_pixels, this->n_elem * sizeof(float), cudaMemcpyDeviceToDevice));
}*/

gcube::gcube(const std::initializer_list<float> &list) {
  this->d_pixels = NULL;
  this->create(list);
}

gcube::gcube(const std::initializer_list< std::initializer_list<float> > &list) {
  this->d_pixels = NULL;
  this->create(list);
}

gcube::gcube(const std::initializer_list< std::initializer_list< std::initializer_list<float> > > &list) {
  this->d_pixels = NULL;
  this->create(list);
}

gcube::~gcube(void) {
  this->destroy();
}

__global__ void GPU_map_id(float *F, size_t n_elems) {
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= n_elems) {
    return;
  }
  F[idx] = idx;
}

__global__ void GPU_map_assign(float *F, float val, size_t n_elems) {
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= n_elems) {
    return;
  }
  F[idx] = val;
}

void gcube::create(size_t n_rows, size_t n_cols, size_t n_slices, uint8_t fill_type) {
  this->destroy();
  this->n_rows = n_rows;
  this->n_cols = n_cols;
  this->n_slices = n_slices;
  this->n_elem = n_rows * n_cols * n_slices;
  if (this->n_elem != 0) {
    checkCudaErrors(cudaMalloc(&this->d_pixels, this->n_elem * sizeof(float)));
    switch (fill_type) {
      case gfill::none:
        break;
      case gfill::zeros:
        checkCudaErrors(cudaMemset(this->d_pixels, 0, this->n_elem * sizeof(float)));
        break;
      case gfill::ones:
        GPU_map_assign<<<(this->n_elem-1) / 128 + 1, 128>>>(this->d_pixels, 1, this->n_elem);
        checkCudaErrors(cudaGetLastError());
        break;
      case gfill::linspace:
        GPU_map_id<<<(this->n_elem-1) / 128 + 1, 128>>>(this->d_pixels, this->n_elem);
        checkCudaErrors(cudaGetLastError());
      default:
        break;
    }
  }
}

void gcube::create(const std::initializer_list<float> &list) {
  int n_rows = list.size();
  this->create(n_rows, 1, 1, gfill::none);
  if (this->n_elem == 0) {
    return;
  }
  float *data = new float[this->n_elem];
  int i = 0;
  for (const float &f : list) {
    data[i] = f;
    i++;
  }
  checkCudaErrors(cudaMemcpy(this->d_pixels, data,
        this->n_elem * sizeof(float), cudaMemcpyHostToDevice));
  delete data;
}

void gcube::create(const std::initializer_list< std::initializer_list<float> > &list) {
  int n_rows = list.size();
  int n_cols = (n_rows != 0) ? list.begin()->size() : 0;
  this->create(n_rows, n_cols, 1, gfill::none);
  if (this->n_elem == 0) {
    return;
  }
  float *data = new float[this->n_elem];
  int i = 0;
  for (const std::initializer_list<float> &fl : list) {
    int j = 0;
    for (const float &f : fl) {
      data[IJ2C(i, j, n_rows)] = f;
      j++;
    }
    i++;
  }
  checkCudaErrors(cudaMemcpy(this->d_pixels, data,
        this->n_elem * sizeof(float), cudaMemcpyHostToDevice));
  delete data;
}

void gcube::create(const std::initializer_list< std::initializer_list< std::initializer_list<float> > > &list) {
  int n_rows = list.size();
  int n_cols = (n_rows != 0) ? list.begin()->size() : 0;
  int n_slices = (n_cols != 0) ? list.begin()->begin()->size() : 0;
  this->create(n_rows, n_cols, n_slices, gfill::none);
  if (this->n_elem == 0) {
    return;
  }
  float *data = new float[this->n_elem];
  int i = 0;
  for (const std::initializer_list< std::initializer_list<float> > &fll : list) {
    int j = 0;
    for (const std::initializer_list<float> &fl : fll) {
      int k = 0;
      for (const float &f : fl) {
        data[IJK2C(i, j, k, n_rows, n_cols)] = f;
        k++;
      }
      j++;
    }
    i++;
  }
  checkCudaErrors(cudaMemcpy(this->d_pixels, data,
        this->n_elem * sizeof(float), cudaMemcpyHostToDevice));
  delete data;
}

void gcube::destroy(void) {
  if (this->d_pixels) {
    checkCudaErrors(cudaFree(this->d_pixels));
    this->d_pixels = NULL;
  }
}

// OPERATORS

gcube &gcube::operator=(const gcube &gpucube) { // do not use: broken
  this->destroy();
  this->n_rows = gpucube.n_rows;
  this->n_cols = gpucube.n_cols;
  this->n_slices = gpucube.n_slices;
  this->n_elem = gpucube.n_elem;
  this->d_pixels = gpucube.d_pixels;
  return *this;
}

void gcube::operator+=(const float &f) {
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_cols-1) / 16 + 1, (this->n_rows-1) / 16 + 1, 1);
  GPU_addI<<<gridSize, blockSize>>>(this->d_pixels, this->d_pixels, f, this->n_rows, this->n_cols);
  checkCudaErrors(cudaGetLastError());
}

void gcube::operator-=(const float &f) {
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_cols-1) / 16 + 1, (this->n_rows-1) / 16 + 1, 1);
  GPU_subI<<<gridSize, blockSize>>>(this->d_pixels, this->d_pixels, f, this->n_rows, this->n_cols);
  checkCudaErrors(cudaGetLastError());
}

void gcube::operator*=(const float &f) {
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_cols-1) / 16 + 1, (this->n_rows-1) / 16 + 1, 1);
  GPU_mulI<<<gridSize, blockSize>>>(this->d_pixels, this->d_pixels, f, this->n_rows, this->n_cols);
  checkCudaErrors(cudaGetLastError());
}

void gcube::operator/=(const float &f) {
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_cols-1) / 16 + 1, (this->n_rows-1) / 16 + 1, 1);
  GPU_divI<<<gridSize, blockSize>>>(this->d_pixels, this->d_pixels, f, this->n_rows, this->n_cols);
  checkCudaErrors(cudaGetLastError());
}

void gcube::operator+=(const gcube &other) {
  assert(this->n_rows == other.n_rows && this->n_cols == other.n_cols);
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_cols-1) / 16 + 1, (this->n_rows-1) / 16 + 1, 1);
  GPU_add<<<gridSize, blockSize>>>(this->d_pixels, this->d_pixels, other.d_pixels, this->n_rows, this->n_cols);
  checkCudaErrors(cudaGetLastError());
}

void gcube::operator-=(const gcube &other) {
  assert(this->n_rows == other.n_rows && this->n_cols == other.n_cols);
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_cols-1) / 16 + 1, (this->n_rows-1) / 16 + 1, 1);
  GPU_sub<<<gridSize, blockSize>>>(this->d_pixels, this->d_pixels, other.d_pixels, this->n_rows, this->n_cols);
  checkCudaErrors(cudaGetLastError());
}

void gcube::operator%=(const gcube &other) {
  assert(this->n_rows == other.n_rows && this->n_cols == other.n_cols);
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_cols-1) / 16 + 1, (this->n_rows-1) / 16 + 1, 1);
  GPU_mul<<<gridSize, blockSize>>>(this->d_pixels, this->d_pixels, other.d_pixels, this->n_rows, this->n_cols);
  checkCudaErrors(cudaGetLastError());
}

/*void gcube::operator/=(const gcube &other) {
  assert(this->n_rows == other.n_rows && this->n_cols == other.n_cols);
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_cols-1) / 16 + 1, (this->n_rows-1) / 16 + 1, 1);
  GPU_div<<<gridSize, blockSize>>>(this->d_pixels, this->d_pixels, other.d_pixels, this->n_rows, this->n_cols);
  checkCudaErrors(cudaGetLastError());
}*/

gcube gcube::operator+(const float &f) {
  gcube G(this->n_rows, this->n_cols, 1, gfill::none);
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_cols-1) / 16 + 1, (this->n_rows-1) / 16 + 1, 1);
  GPU_addI<<<gridSize, blockSize>>>(G.d_pixels, this->d_pixels, f, this->n_rows, this->n_cols);
  checkCudaErrors(cudaGetLastError());
  return G;
}

gcube gcube::operator-(const float &f) {
  gcube G(this->n_rows, this->n_cols, 1, gfill::none);
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_cols-1) / 16 + 1, (this->n_rows-1) / 16 + 1, 1);
  GPU_subI<<<gridSize, blockSize>>>(G.d_pixels, this->d_pixels, f, this->n_rows, this->n_cols);
  checkCudaErrors(cudaGetLastError());
  return G;
}

gcube gcube::operator*(const float &f) {
  gcube G(this->n_rows, this->n_cols, 1, gfill::none);
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_cols-1) / 16 + 1, (this->n_rows-1) / 16 + 1, 1);
  GPU_mulI<<<gridSize, blockSize>>>(G.d_pixels, this->d_pixels, f, this->n_rows, this->n_cols);
  checkCudaErrors(cudaGetLastError());
  return G;
}

gcube gcube::operator/(const float &f) {
  gcube G(this->n_rows, this->n_cols, 1, gfill::none);
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_cols-1) / 16 + 1, (this->n_rows-1) / 16 + 1, 1);
  GPU_divI<<<gridSize, blockSize>>>(G.d_pixels, this->d_pixels, f, this->n_rows, this->n_cols);
  checkCudaErrors(cudaGetLastError());
  return G;
}

gcube gcube::operator+(const gcube &other) {
  gcube G(this->n_rows, this->n_cols, 1, gfill::none);
  assert(this->n_rows == other.n_rows && this->n_cols == other.n_cols);
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_cols-1) / 16 + 1, (this->n_rows-1) / 16 + 1, 1);
  GPU_add<<<gridSize, blockSize>>>(G.d_pixels, this->d_pixels, other.d_pixels, this->n_rows, this->n_cols);
  checkCudaErrors(cudaGetLastError());
  return G;
}

gcube gcube::operator-(const gcube &other) {
  gcube G(this->n_rows, this->n_cols, 1, gfill::none);
  assert(this->n_rows == other.n_rows && this->n_cols == other.n_cols);
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_cols-1) / 16 + 1, (this->n_rows-1) / 16 + 1, 1);
  GPU_sub<<<gridSize, blockSize>>>(G.d_pixels, this->d_pixels, other.d_pixels, this->n_rows, this->n_cols);
  checkCudaErrors(cudaGetLastError());
  return G;
}

gcube gcube::operator%(const gcube &other) {
  gcube G(this->n_rows, this->n_cols, 1, gfill::none);
  assert(this->n_rows == other.n_rows && this->n_cols == other.n_cols);
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_cols-1) / 16 + 1, (this->n_rows-1) / 16 + 1, 1);
  GPU_mul<<<gridSize, blockSize>>>(G.d_pixels, this->d_pixels, other.d_pixels, this->n_rows, this->n_cols);
  checkCudaErrors(cudaGetLastError());
  return G;
}

/*gcube gcube::operator/(const gcube &other) {
  assert(this->n_rows == other.n_rows && this->n_cols == other.n_cols);
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_cols-1) / 16 + 1, (this->n_rows-1) / 16 + 1, 1);
  GPU_div<<<gridSize, blockSize>>>(this->d_pixels, this->d_pixels, other.d_pixels, this->n_rows, this->n_cols);
  checkCudaErrors(cudaGetLastError());
}*/

// MEMORY

void gcube::copy(const gcube &gpucube) {
  this->create(gpucube.n_rows, gpucube.n_cols, gpucube.n_slices, gfill::none);
  checkCudaErrors(cudaMemcpy(this->d_pixels, gpucube.d_pixels,
        this->n_elem * sizeof(float), cudaMemcpyDeviceToDevice));
}

void gcube::submatCopy(const gcube &gpucube, int x1, int x2, int y1, int y2) {
  this->
}

void gcube::load(const std::string &fname) { // change
  this->create(cv::imread(fname));
}

void gcube::save(const std::string &fname) { // change
  cv::imwrite(fname, this->cv_mat());
}

// Specific OpenCV interaction (to make sure that they are backwards compatible)

gcube::gcube(cv::Mat &cvMat) {
  this->d_pixels = NULL;
  this->create(cvMat);
}

void gcube::create(const cv::Mat &cvMat) {
  this->create(cvMat.rows, cvMat.cols, cvMat.channels(), gfill::none);
  if (this->n_elem == 0) {
    return;
  }
  float *h_pixels = new float[this->n_elem];
  for (int i = 0; i < this->n_rows; i++) {
    for (int j = 0; j < this->n_cols; j++) {
      cv::Vec3b color = cvMat.at<cv::Vec3b>(i, j);
      for (int k = 0; k < this->n_slices; k++) {
        h_pixels[IJK2C(i, j, k, this->n_rows, this->n_cols)] = (float)color[k] / 255.0f;
      }
    }
  }
  checkCudaErrors(cudaMemcpy(this->d_pixels, h_pixels,
        this->n_elem * sizeof(float), cudaMemcpyHostToDevice));
  free(h_pixels);
}

void gcube::create(const cv::Mat &cvMat, int x1, int x2, int y1, int y2) {
  assert(x1 <= x2 && y1 <= y2 && x2 <= cvMat.cols && y2 <= cvMat.rows);
  this->create(y2 - y1, x2 - x1, cvMat.channels(), gfill::none);
  float *h_pixels = new float[this->n_elem];
  for (int i = y1; i < y2; i++) {
    for (int j = x1; j < x2; j++) {
      cv::Vec3b color = cvMat.at<cv::Vec3b>(i, j);
      for (int k = 0; k < this->n_slices; k++) {
        h_pixels[IJK2C(i-y1, j-x1, k, this->n_rows, this->n_cols)] = (float)color[k] / 255.0f;
      }
    }
  }
  checkCudaErrors(cudaMemcpy(this->d_pixels, h_pixels,
        this->n_elem * sizeof(float), cudaMemcpyHostToDevice));
  free(h_pixels);
}

cv::Mat gcube::cv_img(void) {
  cv::Mat cv_image(this->n_rows, this->n_cols, CV_8UC3);
  float *h_pixels = new float[this->n_elem];
  checkCudaErrors(cudaMemcpy(h_pixels, this->d_pixels,
        this->n_elem * sizeof(float), cudaMemcpyDeviceToHost));
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

cv::Mat gcube::cv_mat(void) {
  cv::Mat cv_image(this->n_rows, this->n_cols, (this->n_slices == 1 ? CV_32F : CV_32FC3));
  float *h_pixels = new float[this->n_elem];
  checkCudaErrors(cudaMemcpy(h_pixels, this->d_pixels,
        this->n_elem * sizeof(float), cudaMemcpyDeviceToHost));
  for (int i = 0; i < this->n_rows; i++) {
    for (int j = 0; j < this->n_cols; j++) {
      if (this->n_slices == 1) {
        cv_image.at<float>(i, j) = h_pixels[IJ2C(i, j, this->n_rows)];
      } else if (this->n_slices == 3) {
        cv_image.at<cv::Vec3b>(i, j) = cv::Vec3f(
              h_pixels[IJK2C(i, j, 0, this->n_rows, this->n_cols)],
              h_pixels[IJK2C(i, j, 1, this->n_rows, this->n_cols)],
              h_pixels[IJK2C(i, j, 2, this->n_rows, this->n_cols)]);
      }
    }
  }
  free(h_pixels);
  return cv_image;
}

// specific armadillo compatibility

arma::cube gcube::arma_cube(void) {
  arma::cube ac(this->n_rows, this->n_cols, this->n_slices);
  float *h_pixels = new float[this->n_elem];
  checkCudaErrors(cudaMemcpy(h_pixels, this->d_pixels,
        this->n_elem * sizeof(float), cudaMemcpyDeviceToHost));
  for (int i = 0; i < this->n_rows; i++) {
    for (int j = 0; j < this->n_cols; j++) {
      for (int k = 0; k < this->n_slices; k++) {
        ac(i, j, k) = h_pixels[IJK2C(i, j, k, this->n_rows, this->n_cols)];
      }
    }
  }
  free(h_pixels);
  return ac;
}

gcube &gcube::operator=(const cv::Mat &cvMat) {
  this->create(cvMat);
  return *this;
}
