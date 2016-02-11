#include "gcube.h"
#include "highgui.h"
#include "imgproc.h"
#include <cmath>
#include "gpu_util.h"
#include <cassert>
#include <cstdio>

#define M_PI_8   0.39269908169872414
#define M_3_PI_8 1.1780972450961724
#define M_5_PI_8 1.9634954084936207
#define M_7_PI_8 2.748893571891069

__global__ void GPU_conv2_gen(float *G, float *F, float *H, int F_n_rows, int F_n_cols, int H_n_rows, int H_n_cols) {
  // assume that the matrix represents an image
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  int j = blockIdx.y * blockDim.y + threadIdx.y;
  if (i >= F_n_rows || j >= F_n_cols) {
    return;
  }
  // stencil operation
  int mi = H_n_rows / 2;
  int mj = H_n_cols / 2;

  // call kernel? for now just use a for loop
  float total = 0.0f;
  int _i, _j, i_, j_;
  for (i_ = 0; i_ < H_n_rows; i_++) {
    for (j_ = 0; j_ < H_n_cols; j_++) {
      _i = i + mi - i_;
      _j = j + mj - j_;
      if (_i >= 0 && _i < F_n_rows && _j >= 0 && _j < F_n_cols) {
        total += H[IJ2C(i_, j_, H_n_rows)] * F[IJ2C(_i, _j, F_n_rows)];
      }
    }
  }
  G[IJ2C(i, j, F_n_rows)] = total;
}

gcube gpu_conv2(const gcube &F, const gcube &K) {
  gcube G(F.n_rows, F.n_cols, F.n_slices);
  for (int k = 0; k < F.n_slices; k++) {
    dim3 blockSize(16, 16, 1);
    dim3 gridSize((F.n_rows-1)/16+1, (F.n_cols-1)/16+1, 1);
    // call the GPU kernel
    GPU_conv2_gen<<<gridSize, blockSize>>>(
        &G.d_pixels[IJK2C(0, 0, k, F.n_rows, F.n_cols)],
        &F.d_pixels[IJK2C(0, 0, k, F.n_rows, F.n_cols)],
        K.d_pixels, F.n_rows, F.n_cols, K.n_rows, K.n_cols);
    checkCudaErrors(cudaGetLastError());
  }
  return G;
}

__global__ void GPU_conv2_Hx(float *G, float *F, float *Hx, int F_n_rows, int F_n_cols, int H_n_cols) {
  // assume that the matrix represents an image
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  int j = blockIdx.y * blockDim.y + threadIdx.y;
  if (i >= F_n_rows || j >= F_n_cols) {
    return;
  }
  // stencil operation
  int mj = H_n_cols / 2;

  // for now just use a for loop
  float total = 0.0f;
  int _j, j_;
  for (j_ = 0; j_ < H_n_cols; j_++) {
    _j = j + mj - j_;
    if (_j >= 0 && _j < F_n_cols) {
      total += Hx[j_] * F[IJ2C(i, _j, F_n_rows)];
    }
  }
  G[IJ2C(i, j, F_n_rows)] = total;
}

__global__ void GPU_conv2_Hy(float *G, float *F, float *Hy, int F_n_rows, int F_n_cols, int H_n_rows) {
  // assume that the matrix represents an image
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  int j = blockIdx.y * blockDim.y + threadIdx.y;
  if (i >= F_n_rows || j >= F_n_cols) {
    return;
  }
  // stencil operation
  int mi = H_n_rows / 2;

  // for now just use a for loop
  float total = 0.0f;
  int _i, i_;
  for (i_ = 0; i_ < H_n_rows; i_++) {
    _i = i + mi - i_;
    if (_i >= 0 && _i < F_n_rows) {
      total += Hy[i_] * F[IJ2C(_i, j, F_n_rows)];
    }
  }
  G[IJ2C(i, j, F_n_rows)] = total;
}

gcube gpu_conv2(const gcube &F, const gcube &DX, const gcube &DY) {
  gcube G(F.n_rows, F.n_cols, F.n_slices);
  gcube H(F.n_rows, F.n_cols, F.n_slices);
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((F.n_rows-1)/16+1, (F.n_cols-1)/16+1, 1);
  for (int k = 0; k < F.n_slices; k++) {
    // call the GPU kernel
    GPU_conv2_Hx<<<gridSize, blockSize>>>(
        &G.d_pixels[IJK2C(0, 0, k, G.n_rows, G.n_cols)],
        &F.d_pixels[IJK2C(0, 0, k, F.n_rows, F.n_cols)],
        DX.d_pixels, F.n_rows, F.n_cols, DX.n_cols);
    checkCudaErrors(cudaGetLastError());
    GPU_conv2_Hy<<<gridSize, blockSize>>>(
        &H.d_pixels[IJK2C(0, 0, k, H.n_rows, H.n_cols)],
        &G.d_pixels[IJK2C(0, 0, k, G.n_rows, G.n_cols)],
        DY.d_pixels, G.n_rows, G.n_cols, DY.n_rows);
    checkCudaErrors(cudaGetLastError());
  }
  return H;
}

gcube gpu_gauss2(int n, double sigma2) {
  assert(n > 0);
  gcube H(n, n);
  float h_pixels[n * n];
  float total = 0.0f;
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      double x = (double)i - ((double)(n-1)/2);
      double y = (double)j - ((double)(n-1)/2);
      float g = (float)exp(-(x * x + y * y) / (2 * sigma2));
      h_pixels[IJ2C(i, j, n)] = g;
      total += g;
    }
  }
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      h_pixels[IJ2C(i, j, n)] /= total;
    }
  }
  checkCudaErrors(cudaMemcpy(H.d_pixels, h_pixels, n * n * sizeof(float), cudaMemcpyHostToDevice));
  return H;
}

void gpu_gauss2(gcube &DX, gcube &DY, int n, double sigma2) {
  assert(n > 0);
  DX.create(1, n);
  DY.create(n, 1);
  float h_pixels[n];
  float total = 0.0f;
  for (int i = 0; i < n; i++) {
    double x = (double)i - ((double)(n-1)/2);
    float g = (float)exp(-(x * x) / (2 * sigma2));
    h_pixels[i] = g;
    total += g;
  }
  for (int i = 0; i < n; i++) {
    h_pixels[i] /= total;
  }
  checkCudaErrors(cudaMemcpy(DX.d_pixels, h_pixels, n * sizeof(float), cudaMemcpyHostToDevice));
  checkCudaErrors(cudaMemcpy(DY.d_pixels, h_pixels, n * sizeof(float), cudaMemcpyHostToDevice));
}

void gpu_gradient2(const gcube &F, gcube &DX, gcube &DY) {
  DX = gpu_edgeSobel2(F, false);
  DY = gpu_edgeSobel2(F, true);
}

__global__ void GPU_nmm2(float *G, float *F, float *Fx, float *Fy, int n_rows, int n_cols) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  int j = blockIdx.y * blockDim.y + threadIdx.y;
  if (i >= n_rows || j >= n_cols) {
    return;
  }
  // get the direction of the gradient...
  if (i == 0 || i == n_rows - 1 || j == 0 || j == n_cols - 1) {
    G[IJ2C(i, j, n_rows)] = 0;
    return;
  }
  float angle = atan2f(Fy[IJ2C(i, j, n_rows)], Fx[IJ2C(i, j, n_rows)]);
  int dx = (angle > -M_3_PI_8 && angle < M_3_PI_8) - (angle < -M_5_PI_8 || angle > M_5_PI_8);
  int dy = (angle > M_PI_8 && angle < M_7_PI_8) - (angle < -M_PI_8 && angle > -M_7_PI_8);
  int v = F[IJ2C(i, j, n_rows)];
  int v1 = v - F[IJ2C(dy, dx, n_rows)];
  int v2 = v - F[IJ2C(-dy, -dx, n_rows)];
  G[IJ2C(i, j, n_rows)] = (v1 + v2) * 0.5f * (v1 > 0 && v2 > 0);
}

gcube gpu_nmm2(const gcube &F, const gcube &Fx, const gcube &Fy) {
  gcube G(F.n_rows, F.n_cols);
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((F.n_rows-1)/16+1, (F.n_cols-1)/16+1, 1);
  GPU_nmm2<<<gridSize, blockSize>>>
    (G.d_pixels, F.d_pixels, Fx.d_pixels, Fy.d_pixels, F.n_rows, F.n_cols);
  checkCudaErrors(cudaGetLastError());
  return G;
}

__global__ void GPU_bilinear_interpolate2(float *G, float *F, int G_rows, int G_cols, int F_rows, int F_cols, int n_slices, float kr, float kc) {
  // gather
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  int j = blockIdx.y * blockDim.y + threadIdx.y;
  if (i >= G_rows || j >= G_cols) {
    return;
  }
  float y = (float)i * kr - 0.5f;
  float x = (float)j * kc - 0.5f;
  int Fi = (int)floorf(y);
  int Fj = (int)floorf(x);
  float dy = 1.0f - (y - floorf(y));
  float dx = 1.0f - (x - floorf(x));
  for (int k = 0; k < n_slices; k++) {
    float wsum = 0.0f;
    float total = 0.0f;
    if (Fj >= 0 && Fj < F_cols) {
      if (Fi >= 0 && Fi < F_rows) {
        wsum += dx * dy * F[IJK2C(Fi, Fj, k, F_rows, F_cols)];
        total += dx * dy;
      }
      if (Fi+1 >= 0 && Fi+1 < F_rows) {
        wsum += dx * (1-dy) * F[IJK2C(Fi+1, Fj, k, F_rows, F_cols)];
        total += dx * (1-dy);
      }
    }
    if (Fj+1 >= 0 && Fj+1 < F_cols) {
      if (Fi >= 0 && Fi < F_rows) {
        wsum += (1-dx) * dy * F[IJK2C(Fi, Fj+1, k, F_rows, F_cols)];
        total += (1-dx) * dy;
      }
      if (Fi+1 >= 0 && Fi < F_cols) {
        wsum += (1-dx) * (1-dy) * F[IJK2C(Fi+1, Fj+1, k, F_rows, F_cols)];
        total += (1-dx) * (1-dy);
      }
    }
    if (total != 0.0f) {
      G[IJK2C(i, j, k, G_rows, G_cols)] = wsum / total; // normalize
    }
  }
}

gcube gpu_imresize2(const gcube &A, int m, int n) {
  gcube G(m, n, A.n_slices);
  double kr = (double)A.n_rows / (double)m;
  double kc = (double)A.n_cols / (double)n;
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((G.n_rows-1)/16+1, (G.n_cols-1)/16+1, 1);
  GPU_bilinear_interpolate2<<<gridSize, blockSize>>>(G.d_pixels, A.d_pixels,
      G.n_rows, G.n_cols, A.n_rows, A.n_cols, A.n_slices, (float)kr, (float)kc);
  checkCudaErrors(cudaGetLastError());
  return G;
}

__global__ void GPU_k_cluster_mapcentroid(int *id, float *pts, float *centroids,
    int n_dim, int n_pts, int k) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n_pts) {
    return;
  }
  // 1) look at all the hypothesis centroids, choose the closest centroid
  float minerr = 0;
  int errset = 0;
  int index = 0;
  float diff;
  for (int kid = 0; kid < k; kid++) {
    float interr = 0;
    for (int dim = 0; dim < n_dim; dim++) {
      diff = pts[IJ2C(dim, i, n_dim)] - centroids[IJ2C(dim, kid, n_dim)];
      interr += diff * diff;
    }
    if (!errset || minerr > interr) {
      errset = 1;
      minerr = interr;
      index = kid;
    }
  }
  // 2) place the id in
  id[i] = index;
}

__global__ void GPU_k_cluster_mapid(float *outpts, float *outcount, int *id, float *pts, int n_dim, int n_pts, int cid) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n_pts) {
    return;
  }
  if (id[i] == cid) {
    for (int j = 0; j < n_dim; j++) {
      outpts[IJ2C(i, j, n_pts)] = pts[IJ2C(j, i, n_dim)];
    }
    outcount[i] = 1.0f;
  } else {
    for (int j = 0; j < n_dim; j++) {
      outpts[IJ2C(i, j, n_pts)] = 0.0f;
    }
    outcount[i] = 0.0f;
  }
}

__global__ void GPU_k_cluster_commit(float *centroids, float *outpts, float *outcount, int n_dim, int n_pts, int k) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n_dim) {
    return;
  }
  centroids[IJ2C(i, k, n_dim)] = outpts[IJ2C(0, i, n_pts)] / outcount[0];
}


// TODO
void gpu_k_cluster(const gcube &S, int k, int niter, gcube &centroids, const gcube &hyp, bool usehyp) {
  // do a check against the size of the image
  assert(S.n_rows == 3); // only work on RGB for now
  assert(k <= 256 && k > 0); // only work with k <= 256 and k > 0

  // create centroids
  centroids.create(S.n_rows, k);

  // insert hypotheses into the centroids
  if (usehyp) {
    checkCudaErrors(cudaMemcpy(centroids.d_pixels, hyp.d_pixels,
          hyp.n_rows * hyp.n_cols * sizeof(float), cudaMemcpyDeviceToDevice));
  }

  // insert random columns into the centroids
  for (int j = hyp.n_cols; j < k; j++) {
    int cind = rand() % S.n_cols; // do something better than random
    checkCudaErrors(cudaMemcpy(&centroids.d_pixels[IJ2C(0, j, S.n_rows)],
          &S.d_pixels[IJ2C(0, cind, S.n_rows)],
          S.n_rows * sizeof(float), cudaMemcpyDeviceToDevice));
  }

  // call the gpu calls for clustering, in the following order:
  // 1) create a bunch of centroid associations
  // 2) once the centroids associations have been made, map the id
  // 3) do steele-sum scan
  // 4) remap the sum-scan to the centroid format
  int *id;
  float *outpts;
  float *outcount;
  checkCudaErrors(cudaMalloc(&id, sizeof(int) * S.n_cols));
  checkCudaErrors(cudaMalloc(&outpts, sizeof(float) * S.n_cols * S.n_rows));
  checkCudaErrors(cudaMalloc(&outcount, sizeof(float) * S.n_cols));

  for (int iter = 0; iter < niter; iter++) {
    // 1) create a bunch of centroid associations
    dim3 blockSize(256, 1, 1); // the max number of threads is 1024
    dim3 gridSize((S.n_cols-1)/256+1, 1, 1);
    GPU_k_cluster_mapcentroid<<<gridSize, blockSize>>>(id, S.d_pixels, centroids.d_pixels,
        S.n_rows, S.n_cols, k);
    checkCudaErrors(cudaGetLastError());

    for (int i = 0; i < k; i++) {
      // 2) once the centroids have been made, map the id
      blockSize.x = 256;
      gridSize.x = (S.n_cols-1)/256+1;
      GPU_k_cluster_mapid<<<gridSize, blockSize>>>(outpts, outcount, id, S.d_pixels,
          S.n_rows, S.n_cols, i);
      checkCudaErrors(cudaGetLastError());

      // 3) do steele-sum scan
      blockSize.x = 256;
      gridSize.x = (S.n_cols-1)/256+1;
      gridSize.y = S.n_rows;
      for (size_t j = 0; (1 << j) < S.n_cols; j += 8) { // this is where it is possibly failing
        // sum for the points
        GPU_sum<<<gridSize, blockSize, sizeof(float) * blockSize.x>>>(outpts, outpts,
            S.n_cols, S.n_rows, blockSize.x, j);
        checkCudaErrors(cudaGetLastError());
        // sum for the count
        GPU_sum<<<gridSize, blockSize, sizeof(float) * blockSize.x>>>(outcount, outcount,
            S.n_cols, 1, blockSize.x, j);
        checkCudaErrors(cudaGetLastError());

        blockSize.x = MIN(gridSize.x, 256);
        gridSize.x = (blockSize.x-1)/256+1;
      }

      // 4) commit the results to memory (waste, but we gotta do it)
      blockSize.x = S.n_rows;
      gridSize.y = 1;
      GPU_k_cluster_commit<<<gridSize, blockSize>>>(centroids.d_pixels, outpts, outcount, S.n_rows, S.n_cols, i);
      checkCudaErrors(cudaGetLastError());
    }
  }

  checkCudaErrors(cudaFree(id));
  checkCudaErrors(cudaFree(outpts));
  checkCudaErrors(cudaFree(outcount));
}

__global__ void GPU_remap_img2mat(float *G, float *F, int n_rows, int n_cols, int n_slices) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  int j = blockIdx.y * blockDim.y + threadIdx.y;
  int k = blockIdx.z * blockDim.z + threadIdx.z;
  if (i >= n_rows || j >= n_cols || k >= n_slices) {
    return;
  }
  G[IJ2C(k, IJ2C(i, j, n_rows), n_slices)] = F[IJK2C(i, j, k, n_rows, n_cols)];
}

gcube gpu_hist_segment2(const gcube &I, int n_centroids, int n_iter, const gcube &hyp, bool usehyp) {
  // return the number of centroids
  gcube S(I.n_slices, I.n_rows * I.n_cols);

  // remap the image
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((I.n_rows-1)/16+1, (I.n_cols-1)/16+1, I.n_slices);
  GPU_remap_img2mat<<<gridSize, blockSize>>>(S.d_pixels, I.d_pixels, I.n_rows, I.n_cols, I.n_slices);
  checkCudaErrors(cudaGetLastError());

  // cluster the image
  gcube centroids;
  gpu_k_cluster(S, n_centroids, n_iter, centroids, hyp, usehyp);

  return centroids;
}

__global__ void GPU_filter_colors(float *F, float *M, int F_n_rows, int F_n_cols, int F_n_slices, int n_matches, int target) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  int j = blockIdx.y * blockDim.y + threadIdx.y;
  if (i >= F_n_rows || j >= F_n_cols) {
    return;
  }
  // find the closest match
  int closest = 0;
  float total_diff = 0.0f;
  for (int k = 0; k < F_n_slices; k++) {
    float diff = F[IJK2C(i, j, k, F_n_rows, F_n_cols)] - M[IJ2C(k, 0, F_n_slices)];
    total_diff += diff * diff;
  }
  for (int m = 1; m < n_matches; m++) {
    float interim_diff = 0.0f;
    for (int k = 0; k < F_n_slices; k++) {
      float diff = F[IJK2C(i, j, k, F_n_rows, F_n_cols)] - M[IJ2C(k, m, F_n_slices)];
      interim_diff += diff * diff;
    }
    if (interim_diff < total_diff) {
      closest = m;
      total_diff = interim_diff;
    }
  }
  // set color if within set, else filter to black
  if (closest < target) {
    for (int k = 0; k < F_n_slices; k++) {
      F[IJK2C(i, j, k, F_n_rows, F_n_cols)] = M[IJ2C(k, closest, F_n_slices)];
    }
  } else {
    for (int k = 0; k < F_n_slices; k++) {
      F[IJK2C(i, j, k, F_n_rows, F_n_cols)] = 0.0f;
    }
  }
}

gcube gpu_filter_colors(const gcube &I, const gcube &C, size_t target) {
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((I.n_rows-1)/16+1, (I.n_cols-1)/16+1, 1);
  gcube F = I;
  GPU_filter_colors<<<gridSize, blockSize>>>(F.d_pixels, C.d_pixels, F.n_rows, F.n_cols, F.n_slices, C.n_cols, target);
  checkCudaErrors(cudaGetLastError());
  return F;
}

/*__global__ void GPU_sad2(float *S, float *I1, float *I2, int n_rows, int n_cols) {
 
}

float sad2(const gcube &I1, const gcube &I2) {
  assert(I1.n_rows == I2.n_rows && I1.n_cols == I2.n_cols);
  float h_G;
  float *d_G;
  checkCudaErrors(cudaMalloc(&d_G, sizeof(float)));
  gcube Temp(I1.n_rows, I1.n_cols);
  GPU_sub<<<(Temp.n_elem-1)/256+1, 256>>>(Temp, I1, I2);
  checkCudaErrors(cudaGetLastError());
  GPU_abs<<<(Temp.n_elem-1)/256+1, 256>>>(Temp, Temp);
  checkCudaErrors(cudaGetLastError());
  GPU_sum<<<(Temp.n_elem-1)/256+1, 256>>>(d_G, Temp);
  checkCudaErrors(cudaGetLastError());
  checkCudaErrors(cudaMemcpy(&h_G, d_G, sizeof(float), cudaMemcpyDeviceToHost));
  checkCudaErrors(cudaFree(d_G));
  return h_G;
}

float ssd2(const gcube &I1, const gcube &I2) {
  assert(I1.n_rows == I2.n_rows && I1.n_cols == I2.n_cols);
  float h_G;
  float *d_G;
  checkCudaErrors(cudaMalloc(&d_G, sizeof(float)));
  gcube Temp(I1.n_rows, I1.n_cols);
  GPU_sub<<<(T.n_elem-1)/256+1, 256>>>(Temp, I1, I2);
  checkCudaErrors(cudaGetLastError());
  GPU_eemult<<<(Temp.n_elem-1)/256+1, 256>>>(Temp, Temp, Temp);
  checkCudaErrors(cudaGetLastError());
  GPU_sum<<<(Temp.n_elem-1)/256+1, 256>>>(d_G, Temp);
  checkCudaErrors(cudaGetLastError());
  checkCudaErrors(cudaMemcpy(&h_G, d_G, sizeof(float), cudaMemcpyDeviceToHost));
  checkCudaErrors(cudaFree(d_G));
  return h_G;
}

float ncc2(const gcube &I1, const gcube &I2) {
  assert(I1.n_rows == I2.n_rows && I1.n_cols == I2.n_cols);
  float h_G;
  float *d_G; // size 2 for the mus
  checkCudaErrors(cudaMalloc(&d_G,  2 * sizeof(float)));
  double mu[2];
  GPU_sum<<<(Temp.n_elem-1)/256+1, 256>>>(d_G, Temp);
}*/
