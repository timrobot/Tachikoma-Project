#ifndef __TENSOR_H__
#define __TENSOR_H__

#ifdef __NVCC__

#include <opencv2/core/core.hpp>
#include <armadillo>
#include <string>
#include <initializer_list>

// TODO: allow compatibility with arma commands
// TODO: fix some lin alg stuff

namespace tfill {
  const uint8_t none = 0;
  const uint8_t zeros = 1;
  const uint8_t ones = 2;
  const uint8_t linspace = 3; // not implemented yet
};

enum Context { CPU, GPU };

class Tensor {
  public:
    Tensor(void);
    Tensor(const std::initializer_list<size_t> &shape,
           uint8_t fill_type = tfill::none);
    Tensor(const Tensor &tensor);
    Tensor(const Tensor *tensor);
    Tensor(float *data, Context context);
    ~Tensor(void);

    void create(const std::initializer_list<uint32_t> &shape,
                uint8_t fill_type=tfill::none);
    void destroy(void);

    // OPERATORS
    float &(
    void set(float v, size_t i, size_t j = 0, size_t k = 0);
    float get(size_t i, size_t j = 0, size_t k = 0);

    Tensor &operator=(const Tensor &tensor);
    Tensor &operator+=(const float &f);
    Tensor &operator-=(const float &f);
    Tensor &operator*=(const float &f);
    Tensor &operator/=(const float &f);

    Tensor &operator+=(const Tensor &other);
    Tensor &operator-=(const Tensor &other);
    Tensor &operator%=(const Tensor &other);
    Tensor &operator/=(const Tensor &other);
    Tensor &operator*=(const Tensor &other); // TODO: test

    // WARNING: the following create NEW Tensors
    Tensor operator+(const float &f);
    Tensor operator-(const float &f);
    Tensor operator*(const float &f);
    Tensor operator/(const float &f);
    
    Tensor operator+(const Tensor &other);
    Tensor operator-(const Tensor &other);
    Tensor operator%(const Tensor &other);
    Tensor operator/(const Tensor &other);
    Tensor operator*(const Tensor &other);

    // MEMORY
    void copy(const Tensor &tensor);
    void load(const std::string &fname); // TODO: load from HDF5 format
    void save(const std::string &fname); // TODO: save to HDF5 format

#ifdef USE_OPENCV
    // opencv compatibility
    Tensor(cv::Mat &cvMat);
    void create(const cv::Mat &cvMat, bool remalloc = true);
    void create(const cv::Mat &cvMat, int i1, int i2, int j1, int j2, bool remalloc = true);
    cv::Mat cv_img(void);
    //Tensor &operator=(const cv::Mat &cvMat);
    cv::Mat cv_mat(void);
#endif

#ifdef USE_ARMA
    // armadillo compatibility
    Tensor(arma::vec &armaCube);
    Tensor(arma::mat &armaCube);
    Tensor(arma::cube &armaCube);
    void create(const arma::vec &armaCube);
    void create(const arma::mat &armaCube);
    void create(const arma::cube &armaCube);
    //Tensor &operator=(const arma::cube &armaCube);
    arma::cube arma_cube(void);
#endif

    float *data;
    size_t n_rows;
    size_t n_cols;
    size_t n_slices;
    size_t n_elem;
    Context context;
};

#endif
#endif
