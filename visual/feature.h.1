#ifndef __TK_FEATURE_H__
#define __TK_FEATURE_H__

#include <armadillo>
#include <vector>

#ifndef __NVCC__

enum FEATURE_DEFS {
  EDGE_SOBEL,
  EDGE_CANNY,
  EDGE_DOG,
  EDGE_LOG,
  CORNER_SOBEL,
  CORNER_HARRIS,
  LINE_RANSAC,
  LINE_HOUGH
};

/** Create a matrix representing the edges.
 *  @param F the image to extract the edges from
 *  @param n the size of the convolution window
 *  @param op choose the edge parameter
 *  @return the edge matrix
 */
arma::mat edge2(const arma::mat &F, int op = EDGE_CANNY);
arma::cube edge2(const arma::cube &F, int op = EDGE_CANNY);

arma::mat sobel_edge2(const arma::mat &F, arma::uword n = 7, double sigma2 = 3.0);
arma::mat canny2(const arma::mat &F, double low = 0.4, double high = 0.8, arma::uword n = 7, double sigma2 = 3.0);
arma::mat dog2(const arma::mat &F, double alpha = 1.0, arma::uword n = 7, double sigma2 = 3.0);
arma::mat log2(const arma::mat &F, arma::uword n = 7, double sigma2 = 3.0);

void blob2(const arma::mat &F, std::vector<arma::vec> &centroids);

arma::mat corner2(const arma::mat &I, int op);
arma::mat sobel_corner2(const arma::mat &I, arma::uword n = 7, double sigma2 = 3.0);

/** Get the corners of an image using the Harris feature detector.
 *  @param I the image
 *  @param W the weights of importance of a patch
 *  @return the image gradient
 */
arma::mat harris2(const arma::mat &I, const arma::mat &W);

arma::mat lines2(const arma::mat &I, const std::vector<arma::vec> &pts, int op = LINE_HOUGH);
arma::mat ransac(const std::vector<arma::vec> &pts, double sigma2, int k);
arma::mat hough_line(const std::vector<arma::vec> &pts, double sigma2, size_t n_rows, size_t n_cols);

#else

enum FEATURE_DEFS {
  EDGE_SOBEL, EDGE_CANNY, EDGE_LOG,
  CORNER_HARRIS,
  BLOB_GRAY
};

gcube gpu_edge2(const gcube &F, int op = EDGE_LOG);
gcube gpu_corner2(const gcube &F, int op = CORNER_HARRIS);
gcube gpu_blob2(const gcube &F, int op = BLOB_GRAY);

/** GPU Find the edges using the sobel operator
 *  @param F the image
 *  @param isVert (optional) the enable parameter if this is finding the vertical sobel
 *  @return the edge image
 */
gcube gpu_edgeSobel2(const gcube &F, bool isVert = true);

gcube gpu_edgeCanny2(const gcube &F, int n = 7, double sigma2 = 1.2);

gcube gpu_edgeLoG2(const gcube &F, int n = 7, double sigma2 = 1.2, float alpha = 1.0f);

void gpu_cornerSobel2(gcube &DX, gcube &DY);

gcube gpu_corner2(const gcube &F, int n, double sigma2);

#endif

#endif
