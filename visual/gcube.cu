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
  this->copy(gpucube);
}

gcube::gcube(const gcube *gpucube) {
  this->d_pixels = NULL;
  this->copy(*gpucube);
}

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

void gcube::set(float v, size_t i, size_t j, size_t k) {
  checkCudaErrors(cudaMemcpy(&this->d_pixels[IJK2C(i, j, k, this->n_rows, this->n_cols)],
        &v, sizeof(float), cudaMemcpyHostToDevice));
}

float gcube::get(size_t i, size_t j, size_t k) {
  float v;
  checkCudaErrors(cudaMemcpy(&v, &this->d_pixels[IJK2C(i, j, k, this->n_rows, this->n_cols)],
        sizeof(float), cudaMemcpyDeviceToHost));
  return v;
}

gcube &gcube::operator=(const gcube &gpucube) {
  this->copy(gpucube);
  return *this;
}

gcube &gcube::operator+=(const float &f) {
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_rows-1) / 16 + 1, (this->n_cols-1) / 16 + 1, 1);
  GPU_addI<<<gridSize, blockSize>>>(this->d_pixels, this->d_pixels, f, this->n_rows, this->n_cols);
  checkCudaErrors(cudaGetLastError());
  return *this;
}

gcube &gcube::operator-=(const float &f) {
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_rows-1) / 16 + 1, (this->n_cols-1) / 16 + 1, 1);
  GPU_subI<<<gridSize, blockSize>>>(this->d_pixels, this->d_pixels, f, this->n_rows, this->n_cols);
  checkCudaErrors(cudaGetLastError());
  return *this;
}

gcube &gcube::operator*=(const float &f) {
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_rows-1) / 16 + 1, (this->n_cols-1) / 16 + 1, 1);
  GPU_mulI<<<gridSize, blockSize>>>(this->d_pixels, this->d_pixels, f, this->n_rows, this->n_cols);
  checkCudaErrors(cudaGetLastError());
  return *this;
}

gcube &gcube::operator/=(const float &f) {
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_rows-1) / 16 + 1, (this->n_cols-1) / 16 + 1, 1);
  GPU_divI<<<gridSize, blockSize>>>(this->d_pixels, this->d_pixels, f, this->n_rows, this->n_cols);
  checkCudaErrors(cudaGetLastError());
  return *this;
}

gcube &gcube::operator+=(const gcube &other) {
  assert(this->n_rows == other.n_rows && this->n_cols == other.n_cols);
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_rows-1) / 16 + 1, (this->n_cols-1) / 16 + 1, 1);
  GPU_add<<<gridSize, blockSize>>>(this->d_pixels, this->d_pixels, other.d_pixels, this->n_rows, this->n_cols);
  checkCudaErrors(cudaGetLastError());
  return *this;
}

gcube &gcube::operator-=(const gcube &other) {
  assert(this->n_rows == other.n_rows && this->n_cols == other.n_cols);
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_rows-1) / 16 + 1, (this->n_cols-1) / 16 + 1, 1);
  GPU_sub<<<gridSize, blockSize>>>(this->d_pixels, this->d_pixels, other.d_pixels, this->n_rows, this->n_cols);
  checkCudaErrors(cudaGetLastError());
  return *this;
}

gcube &gcube::operator%=(const gcube &other) { // schur product
  assert(this->n_rows == other.n_rows && this->n_cols == other.n_cols);
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_rows-1) / 16 + 1, (this->n_cols-1) / 16 + 1, 1);
  GPU_mul<<<gridSize, blockSize>>>(this->d_pixels, this->d_pixels, other.d_pixels, this->n_rows, this->n_cols);
  checkCudaErrors(cudaGetLastError());
  return *this;
}

gcube &gcube::operator/=(const gcube &other) {
  assert(this->n_rows == other.n_rows && this->n_cols == other.n_cols);
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_rows-1) / 16 + 1, (this->n_cols-1) / 16 + 1, 1);
  GPU_div<<<gridSize, blockSize>>>(this->d_pixels, this->d_pixels, other.d_pixels, this->n_rows, this->n_cols);
  checkCudaErrors(cudaGetLastError());
  return *this;
}

gcube &gcube::operator*=(const gcube &other) {
  gcube G = (*this) * other;
  this->destroy();
  this->d_pixels = G.d_pixels;
  this->n_rows = G.n_rows;
  this->n_cols = G.n_cols;
  this->n_slices = G.n_slices;
  this->n_elem = G.n_elem;
  return *this;
}

gcube gcube::operator+(const float &f) {
  gcube G(this->n_rows, this->n_cols, 1, gfill::none);
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_rows-1) / 16 + 1, (this->n_cols-1) / 16 + 1, 1);
  GPU_addI<<<gridSize, blockSize>>>(G.d_pixels, this->d_pixels, f, this->n_rows, this->n_cols);
  checkCudaErrors(cudaGetLastError());
  return G;
}

gcube gcube::operator-(const float &f) {
  gcube G(this->n_rows, this->n_cols, 1, gfill::none);
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_rows-1) / 16 + 1, (this->n_cols-1) / 16 + 1, 1);
  GPU_subI<<<gridSize, blockSize>>>(G.d_pixels, this->d_pixels, f, this->n_rows, this->n_cols);
  checkCudaErrors(cudaGetLastError());
  return G;
}

gcube gcube::operator*(const float &f) {
  gcube G(this->n_rows, this->n_cols, 1, gfill::none);
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_rows-1) / 16 + 1, (this->n_cols-1) / 16 + 1, 1);
  GPU_mulI<<<gridSize, blockSize>>>(G.d_pixels, this->d_pixels, f, this->n_rows, this->n_cols);
  checkCudaErrors(cudaGetLastError());
  return G;
}

gcube gcube::operator/(const float &f) {
  gcube G(this->n_rows, this->n_cols, 1, gfill::none);
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_rows-1) / 16 + 1, (this->n_cols-1) / 16 + 1, 1);
  GPU_divI<<<gridSize, blockSize>>>(G.d_pixels, this->d_pixels, f, this->n_rows, this->n_cols);
  checkCudaErrors(cudaGetLastError());
  return G;
}

gcube gcube::operator+(const gcube &other) {
  assert(this->n_rows == other.n_rows && this->n_cols == other.n_cols);
  gcube G(this->n_rows, this->n_cols, 1, gfill::none);
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_rows-1) / 16 + 1, (this->n_cols-1) / 16 + 1, 1);
  GPU_add<<<gridSize, blockSize>>>(G.d_pixels, this->d_pixels, other.d_pixels, this->n_rows, this->n_cols);
  checkCudaErrors(cudaGetLastError());
  return G;
}

gcube gcube::operator-(const gcube &other) {
  assert(this->n_rows == other.n_rows && this->n_cols == other.n_cols);
  gcube G(this->n_rows, this->n_cols, 1, gfill::none);
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_rows-1) / 16 + 1, (this->n_cols-1) / 16 + 1, 1);
  GPU_sub<<<gridSize, blockSize>>>(G.d_pixels, this->d_pixels, other.d_pixels, this->n_rows, this->n_cols);
  checkCudaErrors(cudaGetLastError());
  return G;
}

gcube gcube::operator%(const gcube &other) {
  assert(this->n_rows == other.n_rows && this->n_cols == other.n_cols);
  gcube G(this->n_rows, this->n_cols, 1, gfill::none);
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_rows-1)/16+1, (this->n_cols-1)/16+1, 1);
  GPU_mul<<<gridSize, blockSize>>>(G.d_pixels, this->d_pixels, other.d_pixels, this->n_rows, this->n_cols);
  checkCudaErrors(cudaGetLastError());
  return G;
}

gcube gcube::operator/(const gcube &other) {
  assert(this->n_rows == other.n_rows && this->n_cols == other.n_cols);
  gcube G(this->n_rows, this->n_cols, 1, gfill::none);
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_rows-1)/16+1, (this->n_cols-1)/16 + 1, 1);
  GPU_div<<<gridSize, blockSize>>>(G.d_pixels, this->d_pixels, other.d_pixels, this->n_rows, this->n_cols);
  checkCudaErrors(cudaGetLastError());
  return G;
}

gcube gcube::operator*(const gcube &other) {
  assert(this->n_cols == other.n_rows);
  gcube G(this->n_cols, this->n_rows * other.n_cols, 1, gfill::none);
  dim3 blockSize(8, 8, 8);
  dim3 gridSize((this->n_rows-1)/8+1, (other.n_cols-1)/8+1, (this->n_cols-1)/8+1);
  // set up the matrices (map mult)
  GPU_mmul<<<gridSize, blockSize>>>(G.d_pixels, this->d_pixels, other.d_pixels, this->n_rows, other.n_cols, this->n_cols);
  checkCudaErrors(cudaGetLastError());
  // sum up each column
  blockSize.x = 128;
  blockSize.y = 1;
  blockSize.z = 1;
  gridSize.x = (G.n_rows-1)/128+1;
  gridSize.y = G.n_cols;
  gridSize.z = 1;
  for (int i = 0; (size_t)(1 << i) < G.n_rows; i += 8) {
    GPU_sum<<<gridSize, blockSize, sizeof(float) * 128>>>(G.d_pixels, G.d_pixels, G.n_rows, G.n_cols, 128, i);
    checkCudaErrors(cudaGetLastError());
    blockSize.x = MIN(gridSize.x, 128);
    gridSize.x = (blockSize.x-1)/128+1;
  }
  blockSize.x = 128;
  gridSize.x = (G.n_rows-1)/128+1;
  gridSize.y = 1;
  gcube F(this->n_rows * other.n_cols, 1, 1, gfill::none);
  GPU_copyRow<<<gridSize, blockSize>>>(F.d_pixels, G.d_pixels, F.n_rows, 0);
  checkCudaErrors(cudaGetLastError());
  return F;
}

// MEMORY

void gcube::copy(const gcube &gpucube) {
  this->create(gpucube.n_rows, gpucube.n_cols, gpucube.n_slices, gfill::none);
  checkCudaErrors(cudaMemcpy(this->d_pixels, gpucube.d_pixels,
        this->n_elem * sizeof(float), cudaMemcpyDeviceToDevice));
}

/*void gcube::submatCopy(const gcube &gpucube, int x1, int x2, int y1, int y2) {
  this->
}*/

void gcube::load(const std::string &fname) { // change
  this->create(cv::imread(fname));
}

void gcube::save(const std::string &fname) { // change
  cv::imwrite(fname, this->cv_img());
}

// Specific OpenCV interaction (to make sure that they are backwards compatible)

gcube::gcube(cv::Mat &cvMat) {
  this->d_pixels = NULL;
  this->create(cvMat);
}

__global__ void GPU_cv_img2gcube(float *dst, unsigned char *src, int dst_n_rows, int dst_n_cols, int src_n_rows, int src_n_cols, int n_slices, int ioffset, int joffset) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  int j = blockIdx.y * blockDim.y + threadIdx.y;
  int k = blockIdx.z * blockDim.z + threadIdx.z;
  if (i >= dst_n_rows || j >= dst_n_cols || k >= n_slices) {
    return;
  }
  dst[IJK2C(i, j, n_slices-k-1, dst_n_rows, dst_n_cols)] = ((float)src[IJK2C(k, j+joffset, i+ioffset, n_slices, src_n_cols)]) / 255.0;
}

void gcube::create(const cv::Mat &cvMat, bool remalloc) {
  if (remalloc) {
    this->create(cvMat.rows, cvMat.cols, cvMat.channels(), gfill::none);
  } else {
    assert(cvMat.rows * cvMat.cols * cvMat.channels() == this->n_elem && this->d_pixels != NULL);
  }
  if (this->n_elem == 0) {
    return;
  }
  // copy to memory
  unsigned char *dimg;
  checkCudaErrors(cudaMalloc(&dimg, sizeof(unsigned char) * this->n_elem));
  checkCudaErrors(cudaMemcpy(dimg, cvMat.data, sizeof(unsigned char) * this->n_elem, cudaMemcpyHostToDevice));

  // reformat
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_rows-1)/16+1, (this->n_cols-1)/16+1, this->n_slices);
  GPU_cv_img2gcube<<<gridSize, blockSize>>>(this->d_pixels, dimg, this->n_rows, this->n_cols, this->n_rows, this->n_cols, this->n_slices, 0, 0);
  checkCudaErrors(cudaGetLastError());
  checkCudaErrors(cudaFree(dimg));
}

void gcube::create(const cv::Mat &cvMat, int i1, int i2, int j1, int j2, bool remalloc) {
  assert(i1 <= i2 && j1 <= j2 && j2 <= cvMat.cols && i2 <= cvMat.rows);
  int di = i2 - i1;
  int dj = j2 - j1;
  if (remalloc) {
    this->create(di, dj, cvMat.channels(), gfill::none);
  } else {
    assert(di * dj * cvMat.channels() == this->n_elem && this->d_pixels != NULL);
  }
  if (this->n_elem == 0) {
    return;
  }
  // copy to memory
  size_t n_elem = cvMat.rows * cvMat.cols * cvMat.channels();
  unsigned char *dimg;
  checkCudaErrors(cudaMalloc(&dimg, sizeof(unsigned char) * n_elem));
  checkCudaErrors(cudaMemcpy(dimg, cvMat.data, sizeof(unsigned char) * n_elem, cudaMemcpyHostToDevice));

  // reformat
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((di-1)/16+1, (dj-1)/16+1, this->n_slices);
  GPU_cv_img2gcube<<<gridSize, blockSize>>>(this->d_pixels, dimg, di, dj, cvMat.rows, cvMat.cols, this->n_slices, i1, j1);
  checkCudaErrors(cudaGetLastError());
  checkCudaErrors(cudaFree(dimg));
}

/*static int limit(int x, int a, int b) {
  if (x < a) {
    return a;
  } else if (x > b) {
    return b;
  } else {
    return x;
  }
}*/

__global__ void GPU_gcube2cv_img(unsigned char *dst, float *src, int n_rows, int n_cols, int n_slices) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  int j = blockIdx.y * blockDim.y + threadIdx.y;
  int k = blockIdx.z * blockDim.z + threadIdx.z;
  if (i >= n_rows || j >= n_cols || k >= n_slices) {
    return;
  }
  dst[IJK2C(k, j, i, n_slices, n_cols)] = (unsigned char)(src[IJK2C(i, j, n_slices-k-1, n_rows, n_cols)] * 255.0);
}

cv::Mat gcube::cv_img(void) {
  if (this->n_elem == 0) {
    return cv::Mat(0, 0, CV_8UC1);
  }
  cv::Mat cv_image(this->n_rows, this->n_cols, (this->n_slices == 3) ? CV_8UC3 : CV_8UC1);
  // reformat
  unsigned char *dimg;
  checkCudaErrors(cudaMalloc(&dimg, sizeof(unsigned char) * this->n_elem));
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_rows-1)/16+1, (this->n_cols-1)/16+1, this->n_slices);
  GPU_gcube2cv_img<<<gridSize, blockSize>>>(dimg, this->d_pixels, this->n_rows, this->n_cols, this->n_slices);
  checkCudaErrors(cudaGetLastError());

  // place the matrix into the image
  checkCudaErrors(cudaMemcpy(cv_image.data, dimg, sizeof(unsigned char) * this->n_elem, cudaMemcpyDeviceToHost));
  checkCudaErrors(cudaFree(dimg));
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
        cv_image.at<cv::Vec3f>(i, j) = cv::Vec3f(
              h_pixels[IJK2C(i, j, 0, this->n_rows, this->n_cols)],
              h_pixels[IJK2C(i, j, 1, this->n_rows, this->n_cols)],
              h_pixels[IJK2C(i, j, 2, this->n_rows, this->n_cols)]);
      }
    }
  }
  delete h_pixels;
  return cv_image;
}

// specific armadillo compatibility

gcube::gcube(arma::vec &armaCube) {
  this->d_pixels = NULL;
  this->create(armaCube);
}

gcube::gcube(arma::mat &armaCube) {
  this->d_pixels = NULL;
  this->create(armaCube);
}

gcube::gcube(arma::cube &armaCube) {
  this->d_pixels = NULL;
  this->create(armaCube);
}

void gcube::create(const arma::vec &armaCube) {
  this->create(armaCube.n_rows, 1, 1, gfill::none);
  if (this->n_elem == 0) {
    return;
  }
  float *h_pixels = new float[this->n_elem];
  for (int i = 0; i < this->n_rows; i++) {
    h_pixels[i] = (float)armaCube(i);
  }
  checkCudaErrors(cudaMemcpy(this->d_pixels, h_pixels,
        this->n_elem * sizeof(float), cudaMemcpyHostToDevice));
  delete h_pixels;
}

void gcube::create(const arma::mat &armaCube) {
  this->create(armaCube.n_rows, armaCube.n_cols, 1, gfill::none);
  if (this->n_elem == 0) {
    return;
  }
  float *h_pixels = new float[this->n_elem];
  for (int i = 0; i < this->n_rows; i++) {
    for (int j = 0; j < this->n_cols; j++) {
      h_pixels[IJ2C(i, j, this->n_rows)] = (float)armaCube(i, j);
    }
  }
  checkCudaErrors(cudaMemcpy(this->d_pixels, h_pixels,
        this->n_elem * sizeof(float), cudaMemcpyHostToDevice));
  delete h_pixels;
}

void gcube::create(const arma::cube &armaCube) {
  this->create(armaCube.n_rows, armaCube.n_cols, armaCube.n_slices, gfill::none);
  if (this->n_elem == 0) {
    return;
  }
  float *h_pixels = new float[this->n_elem];
  for (int i = 0; i < this->n_rows; i++) {
    for (int j = 0; j < this->n_cols; j++) {
      for (int k = 0; k < this->n_slices; k++) {
        h_pixels[IJK2C(i, j, k, this->n_rows, this->n_cols)] = (float)armaCube(i, j, k);
      }
    }
  }
  checkCudaErrors(cudaMemcpy(this->d_pixels, h_pixels,
        this->n_elem * sizeof(float), cudaMemcpyHostToDevice));
  delete h_pixels;
}

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
  delete h_pixels;
  return ac;
}

/*gcube &gcube::operator=(const cv::Mat &cvMat) {
  this->create(cvMat);
  return *this;
}*/
