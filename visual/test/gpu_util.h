#ifndef __TK_GPU_UTIL_H__
#define __TK_GPU_UTIL_H__

#ifdef __NVCC__

#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#include <iostream>
#include <cstdlib>

#define IJ2C(i,j,ld)(((j)*(ld))+(i))
#define IJK2C(i,j,k,ld,rd) (((k)*(ld)*(rd))+((j)*(ld))+(i)) // column wise ordering
#define GPU_submat(i,j,height,width,from,to) \
  { \
    int _i, _j; \
    for (_i = 0; _i < height; _i++) { \
      for (_j = 0; _j < width; _j++) { \
        to[IJK2C(i,j,1)] = from[IJK2C(i+_i,j+_j,1)]; \
      } \
    } \
  }

#define checkCudaErrors(val) __check( (val), #val, __FILE__, __LINE__)

template<typename T>
void __check(T err, const char* const func, const char* const file, const int line) {
  if (err != cudaSuccess) {
    std::cerr << "CUDA error at: " << file << ":" << line << std::endl;
    std::cerr << cudaGetErrorString(err) << " " << func << std::endl;
    exit(1);
  }
}

#endif
#endif
