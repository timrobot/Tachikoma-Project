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
  float weight = 0.0f;
  int _i, _j, i_, j_;
  for (i_ = 0; i_ < H_n_rows; i_++) {
    for (j_ = 0; j_ < H_n_cols; j_++) {
      _i = i + mi - i_;
      _j = j + mj - j_;
      if (_i >= 0 && _i < F_n_rows && _j >= 0 && _j < F_n_cols) {
        total += H[IJ2C(i_, j_, H_n_rows)] * F[IJ2C(_i, _j, F_n_rows)];
        weight += H[IJ2C(i_, j_, H_n_rows)];
      }
    }
  }
  G[IJ2C(i, j, F_n_rows)] = total; // / weight;
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

/*__global__ void GPU_conv2_Hy(float *G, float *F, float *Hy, int F_n_rows, int F_n_cols, int H_n_rows) {
  // assume that the matrix represents an image
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  int j = blockIdx.y * blockDim.y + threadIdx.y;
  if (j >= F_n_cols || i >= F_n_rows) {
    return;
  }
  // stencil operation
  int mi = H_n_rows / 2;
  //  int mj = H_n_cols / 2;

  // for now just use a for loop
  float total = 0.0f;
  float weight = 0.0f;
  for (int i_ = 0; i_ < H_n_rows; i_++) {
    int _i = i + mi - i_;
    if (fi >= 0 && fi < F_n_rows) {
      total += Hy[hi] * F[IJ2C(fi, j, F_n_rows)];
      weight += Hy[hi];
    }
  }
  G[IJ2C(i, j, F_n_rows)] = total / weight;
} 

__global__ void GPU_conv2_Hx(float *G, float *F, float *Hx, int F_n_rows, int F_n_cols, int H_n_cols) {
  // assume that the matrix represents an image
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  int j = blockIdx.y * blockDim.y + threadIdx.y;
  if (j >= F_n_cols || i >= F_n_rows) {
    return;
  }
  // stencil operation
  //  int mi = H_n_rows / 2;
  int mj = H_n_cols / 2;

  // forst now just use a for loop
  float total = 0.0f;
  float weight = 0.0f;
  for (int hj = 0; hj < H_n_cols; hj++) {
    int fj = j + mj - hj;
    if (fj >= 0 && fj < F_n_cols) {
      total += Hx[hj] * F[IJ2C(i, fj, F_n_rows)];
      weight += Hx[hj];
    }
  }
  G[IJ2C(i, j, F_n_rows)] = total / weight;
}*/

gcube gpu_conv2(const gcube &F, const gcube &V, const gcube &H) {
  gcube G(F.n_rows, F.n_cols, F.n_slices);
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((F.n_rows-1)/16+1, (F.n_cols-1)/16+1, 1);
/*  for (int k = 0; k < F.n_slices; k++) {
    // call the GPU kernel
    GPU_conv2_Hy<<<gridSize, blockSize>>>(
        &G.d_pixels[IJK2C(0, 0, k, G.n_rows, G.n_cols)],
        &F.d_pixels[IJK2C(0, 0, k, F.n_rows, F.n_cols)],
        V.d_pixels, F.n_rows, F.n_cols, V.n_rows);
    checkCudaErrors(cudaGetLastError());
    GPU_conv2_Hx<<<gridSize, blockSize>>>(
        &G.d_pixels[IJK2C(0, 0, k, G.n_rows, G.n_cols)],
        &F.d_pixels[IJK2C(0, 0, k, F.n_rows, F.n_cols)],
        H.d_pixels, F.n_rows, F.n_cols, H.n_rows);
    checkCudaErrors(cudaGetLastError());
  }*/
  return G;
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

void gpu_gauss2(gcube &V, gcube &H, int n, double sigma2) {
  assert(n > 0);
  V.create(n);
  H.create(n);
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
  checkCudaErrors(cudaMemcpy(V.d_pixels, h_pixels, n * sizeof(float), cudaMemcpyHostToDevice));
  checkCudaErrors(cudaMemcpy(H.d_pixels, h_pixels, n * sizeof(float), cudaMemcpyHostToDevice));
}

void gpu_edgesobel2(gcube &V, gcube &H, bool isVert) { // change to vector
  V.create(3);
  H.create(3);
  float V_h_pixels[3];
  float H_h_pixels[3];
  if (isVert) {
    V_h_pixels[0] = 0.25f; V_h_pixels[1] = 0.5f; V_h_pixels[2] = 0.25f;
    H_h_pixels[0] = 0.5f; H_h_pixels[1] = 0.0f; H_h_pixels[2] = -0.5f;
  } else {
    V_h_pixels[0] = 0.5f; V_h_pixels[1] = 0.0f; V_h_pixels[2] = -0.5f;
    H_h_pixels[0] = 0.25f; H_h_pixels[1] = 0.5f; H_h_pixels[2] = 0.25f;
  }
  // small memcpy
  checkCudaErrors(cudaMemcpy(V.d_pixels, V_h_pixels, 3 * sizeof(float), cudaMemcpyHostToDevice));
  checkCudaErrors(cudaMemcpy(H.d_pixels, H_h_pixels, 3 * sizeof(float), cudaMemcpyHostToDevice));
}

void gpu_gradient2(const gcube &F, gcube &V, gcube &H) {
  gcube sobel_v, sobel_h;
  // vertical
  gpu_edgesobel2(sobel_v, sobel_h, true);
  V = gpu_conv2(F, sobel_v, sobel_h);
  // horizontal
  gpu_edgesobel2(sobel_v, sobel_h, false);
  H = gpu_conv2(F, sobel_v, sobel_h);
}

__global__ void GPU_eucdist(float *C, float *A, float *B, int n_rows, int n_cols) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  int j = blockIdx.y * blockDim.y + threadIdx.y;
  if (i >= n_rows || j >= n_cols) {
    return;
  }
  float dx = A[IJ2C(i, j, n_rows)];
  float dy = B[IJ2C(i, j, n_rows)];
  C[IJ2C(i, j, n_rows)] = sqrtf(dx * dx + dy * dy);
}

__global__ void GPU_minthresh2(float *G, float *F, int n_rows, int n_cols, float minthresh) { // TODO
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  int j = blockIdx.y * blockDim.y + threadIdx.y;
  if (i >= n_rows || j >= n_cols) {
    return;
  }
  G[IJ2C(i, j, n_rows)] = F[IJ2C(i, j, n_rows)];
}

// By default uses the sobel operator
gcube gpu_edge2(const gcube &F, int n, double sigma2) {
  gcube V, H;
  // smooth first
  gpu_gauss2(V, H, n, sigma2);
  gcube G = gpu_conv2(F, V, H);
  // get gradients
  gpu_gradient2(G, V, H);
  // grab the eucdist
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((F.n_rows-1)/16+1, (F.n_cols-1)/16+1, 1);
  GPU_eucdist<<<gridSize, blockSize>>>(G.d_pixels, H.d_pixels, V.d_pixels, G.n_rows, G.n_cols);
  // do nonmaximal suppression, then thresholding
  gpu_nmm2(G, V, H);
  // TODO: make adaptive thresholding
  GPU_minthresh2<<<gridSize, blockSize>>>(G.d_pixels, G.d_pixels, G.n_rows, G.n_cols, 0.4);
  return G;
}

gcube gpu_LoG2(const gcube &F, int n, double sigma2) {
  gcube V, H;
  // smooth first
  gpu_gauss2(V, H, n, sigma2);
  gcube G = gpu_conv2(F, V, H);  // TODO: correct conv_sym
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((F.n_rows-1)/16+1, (F.n_cols-1)/16+1, 1);
  GPU_sub<<<gridSize, blockSize>>>(G.d_pixels, F.d_pixels, G.d_pixels, G.n_rows, G.n_cols);
  return G;
}

/*__global__ void GPU_blob2(float *centroid, float *F, int iteration, int n_rows, int n_cols) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  int j = blockIdx.y * blockDim.y + threadIdx.y;
  if (i >= n_rows || j >= n_cols) {
  return;
  }
// 
}

gcube gpu_blob2(const gcube &F, double sigma2, const gcube &color) {
gcube visited(F.n_rows, F.n_cols, 1, gfill::zeros);

}*/

void gpu_cornersobel2(gcube &V, gcube &H) { // change to vector
  V.create(3);
  H.create(3);
  float V_h_pixels[3];
  float H_h_pixels[3];
  V_h_pixels[0] = 0.25f; V_h_pixels[1] = -0.5f; V_h_pixels[2] = 0.25f;
  H_h_pixels[0] = 0.25f; H_h_pixels[1] = -0.5f; H_h_pixels[2] = 0.25f;
  // small memcpy
  checkCudaErrors(cudaMemcpy(V.d_pixels, V_h_pixels, 3 * sizeof(float), cudaMemcpyHostToDevice));
  checkCudaErrors(cudaMemcpy(H.d_pixels, H_h_pixels, 3 * sizeof(float), cudaMemcpyHostToDevice));
}

// By default uses the sobel operator
gcube gpu_corner2(const gcube &F, int n, double sigma2) {
  gcube sobel_v, sobel_h;
  gpu_cornersobel2(sobel_v, sobel_h);
  return gpu_conv2(F, sobel_v, sobel_h);
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
  float wsum = 0.0f;
  float total = 0.0f;
  for (int k = 0; k < n_slices; k++) {
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

__global__ void GPU_k_cluster_setup(float *pts, float *centroids, float *interim,
    int *count, int n_dim, int n_pts, int k) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n_pts) {
    return;
  }
  // once the function is entered, assume that
  // we can function the clustering to behave on the following manner:
  // 1) look at all the hypothesis centroids
  // 2) a) place the corresponding matching centroid into the interim and count
  float minerr = 0;
  int errset = 0;
  int index = 0;
  float diff;
  for (int kid = 0; kid < k; kid++) { // careful! branch
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
  // chunk of memory suppressed (bottleneck?)
  for (int dim = 0; dim < n_dim; dim++) {
    atomicAdd(&interim[IJ2C(dim, index, n_dim)], pts[IJ2C(dim, i, n_dim)]);
  }
  atomicAdd(&count[index], 1);
}

__global__ void GPU_k_cluster_commit(float *centroids, float *interim, int *count, int n_dim, int k) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= k) {
    return;
  }
  // 2) b) commit the results into centroids
  for (int dim = 0; dim < n_dim; dim++) {
    centroids[IJ2C(dim, i, n_dim)] = interim[IJ2C(dim, i, n_dim)] / count[i];
  }
}

void gpu_k_cluster(const gcube &S, int k, int niter, gcube &centroids, gcube &hyp, bool usehyp) {
  // do a check against the size of the image
  assert(S.n_rows == 3); // only work on RGB for now
  assert(k <= 256 && k > 0); // only work with k <= 256 and k > 0
  // generate randomized centers, hypotheses first
  centroids.create(3, k);
  if (usehyp) {
    checkCudaErrors(cudaMemcpy(centroids.d_pixels, hyp.d_pixels,
          3 * hyp.n_cols * sizeof(float), cudaMemcpyDeviceToDevice));
  }
  for (int j = 0; j + hyp.n_cols < k; j++) {
    checkCudaErrors(cudaMemcpy(&centroids.d_pixels[IJ2C(0, j, 3)], &S.d_pixels[IJ2C(0, j, 3)],
          3 * sizeof(float), cudaMemcpyDeviceToDevice));
  }
  // once the hypotheses have been generated
  // call the gpu call for clustering, then redecide the cluster
  gcube interim(3, k);
  int *pcount;
  checkCudaErrors(cudaMalloc(&pcount, sizeof(int) * k));
  for (int iter = 0; iter < niter; iter++) {
    checkCudaErrors(cudaMemset(interim.d_pixels, 0, sizeof(float) * 3 * k));
    checkCudaErrors(cudaMemset(pcount, 0, sizeof(int) * k));
    dim3 blockSize(256, 1, 1);
    dim3 gridSize((S.n_cols-1)/256+1, 1, 1);
    GPU_k_cluster_setup<<<gridSize, blockSize>>>(S.d_pixels, centroids.d_pixels, interim.d_pixels,
        pcount, 3, S.n_cols, k);
    checkCudaErrors(cudaGetLastError());
    gridSize.x = 1;
    GPU_k_cluster_commit<<<gridSize, blockSize>>>(centroids.d_pixels, interim.d_pixels, pcount, 3, k);
    checkCudaErrors(cudaGetLastError());
  }
  checkCudaErrors(cudaFree(pcount));
}
