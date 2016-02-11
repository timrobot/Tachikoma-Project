#ifndef __TK_GPU_UTIL_H__
#define __TK_GPU_UTIL_H__

#ifdef __NVCC__

#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#include <iostream>
#include <cstdlib>

#ifndef MIN
#define MIN(a, b) (((a)<(b))?(a):(b))
#endif

#ifndef MAX
#define MAX(a, b) (((a)>(b))?(a):(b))
#endif

#define IJ2C(i,j,ld)(((j)*(ld))+(i))
#define IJK2C(i,j,k,ld,rd) (((k)*(ld)*(rd))+((j)*(ld))+(i)) // column wise ordering

#define checkCudaErrors(val) __check( (val), #val, __FILE__, __LINE__)

template<typename T>
void __check(T err, const char* const func, const char* const file, const int line) {
  if (err != cudaSuccess) {
    std::cerr << "CUDA error at: " << file << ":" << line << std::endl;
    std::cerr << cudaGetErrorString(err) << " " << func << std::endl;
    exit(1);
  }
}

//typedef float (*SFPfunc)(float, float); // must be of type __global__ or __device__

__global__ void GPU_addI(float *H, float *F, float I, int n_rows, int n_cols);
__global__ void GPU_subI(float *H, float *F, float I, int n_rows, int n_cols);
__global__ void GPU_mulI(float *H, float *F, float I, int n_rows, int n_cols);
__global__ void GPU_divI(float *H, float *F, float I, int n_rows, int n_cols);

__global__ void GPU_add(float *H, float *F, float *G, int n_rows, int n_cols);
__global__ void GPU_sub(float *H, float *F, float *G, int n_rows, int n_cols);
__global__ void GPU_mul(float *H, float *F, float *G, int n_rows, int n_cols);
__global__ void GPU_div(float *H, float *F, float *G, int n_rows, int n_cols);

__global__ void GPU_mmul(float *H, float *F, float *G, int n_rows, int n_cols, int n_inner);
__global__ void GPU_trans(float *H, float *F, int n_rows, int n_cols);
//__global__ void GPU_inv(float *H, float *F, int n_rows, int n_cols);
//__global__ void GPU_svd(float *U, float *S, float *V, float *A, int n_rows, int n_cols);

__global__ void GPU_abs(float *H, float *F, int n_rows, int n_cols);

__global__ void GPU_sum(float *H, float *F, int n_rows, int n_cols, int shmsize, int offset);

__global__ void GPU_copyRow(float *H, float *F, int n_rows, int rowid);

//__global__ void GPU_map(float *H, float *F, int n_rows, int n_cols, SFPfunc op);
//__global__ void GPU_merge(float *H, float *F, float *G, int n_rows, int n_cols, SFPfunc op);

#endif
#endif
