#include "gpu_util.h"
#include "gcube.h"
#include "highgui.h"
#include "imgproc.h"
#include <iostream>

using namespace std;

int main() {
  gcube G = gpu_rgb2gray(load_gcube("butterfly.jpg"));

  gcube K = gpu_gauss2(25, 9.0);
  gcube C1 = gpu_conv2(G, K);

//  disp_gcube("general convolution", C1);
//  disp_wait();
//  save_gcube("test_gpu_blur_gen_25_9.png", C1);

  gcube DX, DY;
  gpu_gauss2(DX, DY, 25, 9.0);
  gcube C2 = gpu_conv2(G, DX, DY);

//  disp_gcube("symmetric convolution", C2);
//  disp_wait();
//  save_gcube("test_gpu_blur_sym_25_9.png", C2);

  gcube S = gpu_imresize2(G, 240, 320);
  S = gpu_edge2(S, EDGE_LOG);
  gcube T = gpu_imresize2(S, 480, 640);
  disp_gcube("smaller", S);
  disp_wait();

  return 0;
}
