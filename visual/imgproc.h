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

/** Convolve an image with some kernel.
 *  @param F the image to convolve
 *  @param H the convolution kernel
 *  @return the convolved image
 */
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

/** Generate a gradient matrix of a grayscale image.
 *  @param F the image to find the gradient of
 *  @return a pair of gradient matrices { X, Y }
 */
void gradient2(arma::mat &DX, arma::mat &DY, const arma::mat &F);

/** Generate a gradient matrix of a grayscale image.
 *  @param F the image to find the gradient of
 *  @return a pair of gradient matrices { X, Y }
 */
void gradient2(arma::cube &DX, arma::cube &DY, const arma::cube &F);

/** Apply non-min/maximal suppression on the image.
 *  @param F the intensity matrix
 *  @param theta the angle matrix
 *  @param (optional) nsize the width of the neighbor kernel
 *  @return a non-maximally suppressed matrix
 */
arma::mat nmm2(const arma::mat &F, const arma::mat &theta, arma::uword nsize = 0);

/** Cluster the matrix using the distance vectors in the matrix.
 *  @param S a matrix of data points, where each column is one datapt
 *  @param k the number of clusters
 *  @param niter the number of iterations
 *  @param centroids (output) the centroids vector
 *  @param hyp (input) the hypothesis vector for the centroids
 *  @param usehyp (optional) choose whether or not to use hypotheses
 *  @return a set of cluster centers, each column being a center
 */
arma::mat k_cluster(const arma::mat &S, arma::uword k, int niter,
    std::vector<arma::vec> &centroids,
    std::vector<arma::vec> hyp = std::vector<arma::vec>(),
    bool usehyp = false);

/** Generate a segmented picture based on the k clustered histogram.
 *  @param F the image to segment
 *  @param k the number of color values
 *  @param centroids (output) the centroids vector
 *  @param niter the number of iterations
 *  @param hyp (input) the hypothesis vector for the centroids
 *  @param usehyp (optional) choose whether or not to use hypotheses
 *  @return the clustered image
 */
arma::mat hist_segment2(const arma::mat &F, arma::uword k,
    std::vector<arma::vec> &centroids,
    int niter = 10,
    std::vector<arma::vec> hyp = std::vector<arma::vec>(),
    bool usehyp = false);

/** Generate a segmented picture based on the k clustered histogram.
 *  @param F the image to segment
 *  @param k the number of color values
 *  @param centroids (output) the centroids vector
 *  @param niter the number of iterations
 *  @param hyp (input) the hypothesis vector for the centroids
 *  @param usehyp (optional) choose whether or not to use hypotheses
 *  @return the clustered image
 */
arma::cube hist_segment2(const arma::cube &F, arma::uword k,
    std::vector<arma::vec> &centroids,
    int niter = 10,
    std::vector<arma::vec> hyp = std::vector<arma::vec>(),
    bool usehyp = false);

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

/** Resize an image using bilinear interpolation
 *  @param A the image to resize
 *  @param m the number of rows of the new size
 *  @param n the number of cols of the new size
 *  @return the interpolated image
 */
arma::mat imresize2(const arma::mat &A, arma::uword m, arma::uword n);

/** Resize an image using bilinear interpolation
 *  @param A the image to resize
 *  @param m the number of rows of the new size
 *  @param n the number of cols of the new size
 *  @return the interpolated image
 */
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
    int noctaves = 8, int nscales = 4, double sigma2 = 0.5);

#else

/** GPU Convolve an image with a kernel
 *  @param F the image
 *  @param K/DX the kernel or x-dim gaussian vector
 *  @param DY the y-dim gaussian vector
 *  @return the convolved image
 */
gcube gpu_conv2(const gcube &F, const gcube &K);
gcube gpu_conv2(const gcube &F, const gcube &DX, const gcube &DY);

/** GPU Create a gaussian kernel
 *  @param n the size of the dimensions of the kernel
 *  @param sigma2 the kernel's covariance
 *  @return the full gaussian kernel
 */
gcube gpu_gauss2(int n, double sigma2);

/** GPU Create a gaussian kernel
 *  @param DX (output) the x-dim gaussian vector
 *  @param DY (output) the y-dim gaussian vector
 *  @param n the size of the dimensions of the kernel
 *  @param sigma2 the kernel's covariance
 */
void gpu_gauss2(gcube &DX, gcube &DY, int n, double sigma2);
void gpu_gradient2(const gcube &F, gcube &DX, gcube &DY);

gcube gpu_nmm2(const gcube &F, const gcube &Fx, const gcube &Fy);

gcube gpu_imresize2(const gcube &A, int m, int n);

gcube gpu_hist_segment2(const gcube &I, int n_centroids, int n_iter, const gcube &hyp, bool usehyp);

gcube gpu_filter_colors(const gcube &I, const gcube &C, size_t n_matches);

#endif

#endif
