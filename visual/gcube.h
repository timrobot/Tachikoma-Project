#ifndef __TK_GCUBE_H__
#define __TK_GCUBE_H__

#ifdef __NVCC__

#include <opencv2/core/core.hpp>
#include <armadillo>
#include <string>
#include <initializer_list>

// TODO: allow compatibility with arma commands
// TODO: fix some lin alg stuff

namespace gfill {
  const uint8_t none = 0;
  const uint8_t zeros = 1;
  const uint8_t ones = 2;
  const uint8_t linspace = 3; // not implemented yet
};

class gcube {
  public:
    gcube(void);
    gcube(size_t n_rows,
          size_t n_cols = 1,
          size_t n_slices = 1,
          uint8_t fill_type = gfill::none);
    gcube(const gcube &gpucube);
    gcube(const gcube *gpucube);
    gcube(const std::initializer_list<float> &list);
    gcube(const std::initializer_list< std::initializer_list<float> > &list);
    gcube(const std::initializer_list< std::initializer_list< std::initializer_list<float> > > &list);
    ~gcube(void);

    void create(size_t n_rows,
                size_t n_cols = 1,
                size_t n_slices = 1,
                uint8_t fill_type = gfill::none);
    void create(const std::initializer_list<float> &list);
    void create(const std::initializer_list< std::initializer_list<float> > &list);
    void create(const std::initializer_list< std::initializer_list< std::initializer_list<float> > > &list);
    void destroy(void);

    // OPERATORS
    void set(float v, size_t i, size_t j = 0, size_t k = 0);
    float get(size_t i, size_t j = 0, size_t k = 0);

    gcube &operator=(const gcube &gpucube);
    gcube &operator+=(const float &f);
    gcube &operator-=(const float &f);
    gcube &operator*=(const float &f);
    gcube &operator/=(const float &f);

    gcube &operator+=(const gcube &other);
    gcube &operator-=(const gcube &other);
    gcube &operator%=(const gcube &other);
    gcube &operator/=(const gcube &other);
    gcube &operator*=(const gcube &other); // TODO: test

    // WARNING: the following create NEW gcubes
    gcube operator+(const float &f);
    gcube operator-(const float &f);
    gcube operator*(const float &f);
    gcube operator/(const float &f);
    
    gcube operator+(const gcube &other);
    gcube operator-(const gcube &other);
    gcube operator%(const gcube &other);
    gcube operator/(const gcube &other);
    gcube operator*(const gcube &other);

    // MEMORY
    void copy(const gcube &gpucube);
//    void submatCopy(const gcube &gpucube, int x1, int x2, int y1, int y2);
    void load(const std::string &fname);
    void save(const std::string &fname);

    // opencv compatibility
    gcube(cv::Mat &cvMat);
    void create(const cv::Mat &cvMat, bool remalloc = true);
    void create(const cv::Mat &cvMat, int i1, int i2, int j1, int j2, bool remalloc = true);
    cv::Mat cv_img(void);
    //gcube &operator=(const cv::Mat &cvMat);
    cv::Mat cv_mat(void);

    // armadillo compatibility
    gcube(arma::vec &armaCube);
    gcube(arma::mat &armaCube);
    gcube(arma::cube &armaCube);
    void create(const arma::vec &armaCube);
    void create(const arma::mat &armaCube);
    void create(const arma::cube &armaCube);
    //gcube &operator=(const arma::cube &armaCube);
    arma::cube arma_cube(void);

    float *d_pixels;
    size_t n_rows;
    size_t n_cols;
    size_t n_slices;
    size_t n_elem;
};

#endif
#endif
