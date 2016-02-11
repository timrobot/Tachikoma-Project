#include "gpu_util.h"
#include "gcube.h"
#include "highgui.h"
#include "imgproc.h"
#include "centroid.h"
#include <iostream>
#include <cstdio>
#include <unistd.h>

using namespace std;

int main() {
  srand(271828183);
  gcube img1 = load_gcube("img01.png"), img2;

  // resize for speed
  size_t newsize = 240;
  img1 = gpu_imresize2(img1, newsize, newsize);
  //  img1 = gpu_conv2(img1, gpu_gauss2(5, 1.2f));

//  disp_gcube("train image", img1);
//  disp_gcube("test image", img2);
//  disp_wait();

  // after choosing the centroids, claim the first to be the background, and the rest to be the foreground
  gcube centroids1, centroids2;

  size_t mx = img1.n_cols/2;
  size_t my = img1.n_rows/2;

  gcube hyp1(img1.n_slices, 2);
  hyp1.set(img1.get(0, 0, 0), 0, 0);
  hyp1.set(img1.get(0, 0, 1), 1, 0);
  hyp1.set(img1.get(0, 0, 2), 2, 0);
  hyp1.set(img1.get(my, mx, 0), 0, 1);
  hyp1.set(img1.get(my, mx, 1), 1, 1);
  hyp1.set(img1.get(my, mx, 2), 2, 1);
  centroids1 = gpu_hist_segment2(img1, 2, 5, hyp1, true);

  // filter out background by selecting only the object's colors
  gcube hyp2(centroids1.n_rows, centroids1.n_cols - 1);
  checkCudaErrors(cudaMemcpy(hyp2.d_pixels, &centroids1.d_pixels[IJ2C(0, 1, centroids1.n_rows)],
        sizeof(float) * hyp2.n_elem, cudaMemcpyDeviceToDevice));

  printf("hyp2:\n");
  print_gcube(hyp2);

  gcube I, E;

  // once the object's color is filtered, use that to get the objects
  for (;;) {
    img2 = load_gcube("test00.png");
    img2 = gpu_imresize2(img2, newsize, newsize);
    centroids2 = gpu_hist_segment2(img2, 5, 5, hyp2, true);

    // the best matching colors are the hypotheses
    I = gpu_filter_colors(img2, centroids2, 1);

    // show the resulting filter
    //  disp_gcube("filtered", I);
    //  disp_wait();

    // get the centroid of the image
    I = gpu_rgb2gray(I);
    double cx, cy;
    gpu_centroid(I, cx, cy);
    printf("c: %lf %lf\n", cx, cy);
    E = gpu_gray2rgb(gpu_edge2(I));
    size_t i = (size_t)round(cy);
    size_t j = (size_t)round(cx);
    printf("%zu %zu\n", i, j);
    E.set(0.0f, i, j, 0);
    E.set(1.0f, i, j, 1);
    E.set(1.0f, i, j, 2);

    disp_gcube("centroids", E);
    disp_keyPressed();

    img2.destroy();
    centroids2.destroy();
    I.destroy();
    E.destroy();
  }

  return 0;
}
