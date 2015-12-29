#ifndef __TK_IMGPROC_H__
#define __TK_IMGPROC_H__

#ifndef __NVCC__

#include <vector>
#include <armadillo>

/** Convolve an image with some kernel.
 *  @param F the image to convolve
 *  @param H the convolution kernel
 *  @return the convolved image
 */
arma::mat conv2(const arma::mat &F, const arma::mat &H);
arma::cube conv2(const arma::cube &F, const arma::mat &H);

/** Generate a gaussian square kernel.
 *  @param n the size of the kernel in pixels
 *  @param sigma2 the gaussian covariance
 *  @return the gaussian square kernel
 */
arma::mat gauss2(arma::uword n, double sigma2);

/** Generate a laplacian square kernel.
 *  @param n the size of the kernel in pixels
 *  @param sigma2 the gaussian covariance
 *  @return the laplace of gauss square kernel
 */
arma::mat laplace_gauss2(arma::uword n, double sigma2);

/** Create a matrix representing the edges.
 *  @param F the image to extract the edges from
 *  @param n the size of the convolution window
 *  @param isSobel (optional) use the sobel operator
 *  @param isDoG (optional) use the difference of gauss operator
 *  @return the edge matrix
 */
arma::mat edge2(const arma::mat &F, arma::uword n, double sigma2,
    bool isSobel = false, bool isDoG = true);
arma::mat edge2(const arma::cube &F, arma::uword n, double sigma2,
    bool isSobel = false, bool isDoG = true);

/** Generate a gradient matrix of a grayscale image.
 *  @param F the image to find the gradient of
 *  @return a pair of gradient matrices { X, Y }
 */
void gradient2(arma::mat &DX, arma::mat &DY, const arma::mat &F);
void gradient2(arma::cube &DX, arma::cube &DY, const arma::cube &F);

/** Apply non-min/maximal suppression on the image.
 *  @param F the image to apply nmm on
 *  @return a non-maximally suppressed matrix
 */
arma::mat nmm2(const arma::mat &F, arma::uword nsize = 1, bool min_en = false, bool max_en = true);

/** Cluster the matrix using the distance vectors in the matrix.
 *  @param S a matrix of data points, where each column is one datapt
 *  @param k the number of clusters
 *  @return a set of cluster centers, each column being a center
 */
arma::mat k_cluster(const arma::mat &S, arma::uword k);

/** Generate a segmented picture based on the k clustered histogram.
 *  @param F the image to segment
 *  @param k the number of color values
 *  @return the clustered image
 */
arma::mat hist_segment2(const arma::mat &F, arma::uword k);
arma::cube hist_segment2(const arma::cube &F, arma::uword k);

/** Get the sum of absolute differences of two patches.
 *  @param I1 the first patch
 *  @param I2 the second patch
 *  @return the sum of the absolute differences of the patches
 */
double sad2(const arma::mat &I1, const arma::mat &I2);

/** Get the sum of square differences of two patches.
 *  @param I1 the first patch
 *  @param I2 the second patch
 *  @return the sum of the square differences of the patches
 */
double ssd2(const arma::mat &I1, const arma::mat &I2);

/** Get the normalized cross-correlation of two patches.
 *  @param I1 the first patch
 *  @param I2 the second patch
 *  @return the normalized cross correlation of two patches
 */
double ncc2(const arma::mat &I1, const arma::mat &I2);

/** Get the corners of an image using the Harris feature detector.
 *  @param I the image
 *  @param W the weights of importance of a patch
 *  @return the image gradient
 */
arma::mat harris2(const arma::mat &I, const arma::mat &W);

/** Resize an image using bilinear interpolation
 *  @param A the image to resize
 *  @param m the number of rows of the new size
 *  @param n the number of cols of the new size
 *  @return the interpolated image
 */
arma::mat imresize2(const arma::mat &A, arma::uword m, arma::uword n);
arma::cube imresize2(const arma::cube &C, arma::uword m, arma::uword n);

/** Quickly resize an image into half the rows and cols
 *  @param A the image to resize
 *  @return the interpolated image
 */
arma::mat fast_imresize_half(const arma::mat &A);

typedef std::vector< std::vector<arma::mat> > imgpyr2;

/** Create a laplacian pyramid of an image
 *  @param H the image to create a pyramid from
 *  @param levels (optional) number of levels in the pyramid, default 0 for infinite
 */
void lappyr2(imgpyr2 &blurred, imgpyr2 &edges, const arma::mat &I,
    int iter = 8, int levels = 4, double sigma2 = 0.5);

#else

/** GPU Convolve an image with a kernel
 *  @param F the image
 *  @param K the kernel
 *  @param H optional second kernel, vector (V replaces K)
 *  @return the convolved image
 */
gcube gpu_conv2(const gcube &F, const gcube &K);
gcube gpu_conv2(const gcube &F, const gcube &V, const gcube &H);

/** GPU Create a gaussian kernel
 *  @param v the vertical gaussian vector
 *  @param h the horizontal gaussian vector
 *  @param n the size of the dimensions of the kernel
 *  @param sigma2 the kernel's covariance
 */
gcube gpu_gauss2(int n, double sigma2);
void gpu_gauss2(gcube &V, gcube &H, int n, double sigma2);

void gpu_edgesobel2(gcube &V, gcube &H, bool isVert);

std::vector<gcube> gpu_gradient2(const gcube &F);

gcube gpu_edge2(const gcube &F, int n, double sigma2);

void gpu_cornersobel2(gcube &V, gcube &H);

gcube gpu_corner2(const gcube &F, int n, double sigma2);

gcube gpu_nmm2(const gcube &F, const gcube &Fx, const gcube &Fy);

gcube gpu_imresize2(const gcube &A, int m, int n);

#endif

#endif
