#include "imgproc.h"
#include "highgui.h"
#include "feature.h"
#include "draw.h"
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <njson/json>

using namespace arma;
using namespace std;
using namespace json = nlohmann::json;

vector<mat> blur_multiple(mat &image, vector<double> &convsigmas) {
  vector<mat> ans;
  for (double &d : convsigmas) {
    ans.push_back(conv2(image, gauss2(11, d)));
  }
  return ans;
}

mat cvhack_hough_circle(mat &m) {
  cube I(m.n_rows, m.n_cols, 1);
  I.slice(0) = m * 255;
  cv::Mat inputImage = cvt_arma2opencv(I);
  vector<cv::Vec3f> circles;
  printf("getting circles\n");
  HoughCircles(inputImage, circles, CV_HOUGH_GRADIENT, 1, 40, 100, 50, 10, 0);
  printf("got circles\n");
  mat pts(3, circles.size());
  for (int j = 0; j < circles.size(); j++) {
    pts(0, j) = circles[j][0];
    pts(1, j) = circles[j][1];
    pts(2, j) = circles[j][2];
  }
  return pts;
}

vector<mat> hough_multiple(vector<mat> &blurred) {
  vector<mat> ans;
  for (mat &m : blurred) {
    ans.push_back(cvhack_hough_circle(m));
  }
  return ans;
}

vector<mat> edge_multiple(mat &orig_image, vector<double> &blurred) {
  vector<mat> edges;
  for (double &d : blurred) {
    edges.push_back((canny2(orig_image, 0.1, 0.2, 11, d) > 0) % ones<mat>(orig_image.n_rows, orig_image.n_cols));
  }
  return edges;
}

vector<cube> draw_circles(vector<mat> &blurred, vector<mat> &ht) {
  vector<cube> drawings;
  for (int i = 0; i < blurred.size(); i++) {
    cube img = gray2rgb(blurred[i]);
    mat &h = ht[i];
    for (int j = 0; j < (int)h.n_cols; j++) {
      draw_circle(img, {1,0,0}, vec({ h(1, j), h(0, j) }), h(2, j));
    }
    drawings.push_back(img);
  }
  return drawings;
}

vector<cube> gray2rgb_multiple(vector<mat> &images) {
  vector<cube> rgb;
  for (mat &m : images) {
    rgb.push_back(gray2rgb(m));
  }
  return rgb;
}

void disp_everything(vector<cube> images, string prefix) {
  for (int i = 0; i < images.size(); i++) {
    string window_name = prefix + to_string(i);
    disp_image(window_name, images[i]);
  }
}

vector<cube> crop_circles(cube image, mat circles) {
  vector<cube> cropped;
  for (int j = 0; j < (int)circles.n_cols; j++) {
    printf("drawing %d\n", j);
    vec circp = circles.col(j);
    cout << circp << endl;
    cv::Mat circle_template(ceil(circp(2)) * 2, ceil(circp(2)) * 2, CV_8UC3);
    circle_template = cv::Scalar(0, 0, 0);
    circle(circle_template, cv::Point2f(ceil(circp(2)), ceil(circp(2))), ceil(circp(2)), cv::Scalar(255, 255, 255), -1);
    cube mask = cvt_opencv2arma(circle_template) / 255;
    int minx = max(vec({floor(circp(0)) - ceil(mask.n_rows/2), 0}));
    int maxx = min(vec({minx + (double)mask.n_cols, (double)image.n_cols}));
    int miny = max(vec({floor(circp(1)) - ceil(mask.n_cols/2), 0}));
    int maxy = min(vec({miny + (double)mask.n_rows, (double)image.n_rows}));
    cube partial_cube = image(span(miny, maxy-1), span(minx, maxx-1), span::all);
    partial_cube = partial_cube % mask(span(0,maxy-miny-1), span(0,maxx-minx-1), span::all);
    cropped.push_back(partial_cube);
  }
  return cropped;
}

Ptr<SVM> load_classifier(string s, string config_file) {
  FileStorage fs(s, FileStorage::READ);
  string params, temp;
  ifstream params_file(config_file);
  while (getline(params_file, temp)) {
    params += temp;
  }
  params_file.close();
  json config = json::parse(params);
  
  Ptr<SVM> classifier = SVM::create();
}

int main(int argc, char *argv[]) {
  // read in the input
  if (argc != 2) {
    printf("usage error\n");
    return 1;
  }
  printf("rgb2gray\n");
  cube rgbimage = load_image(argv[1]);
  mat image = rgb2gray(rgbimage);

  // try to get the handicap signs using multiplle convolution stages
  vector<double> blvls = { 0.3, 0.5, 0.7, 0.9, 1.1, 1.3, 1.5, 1.7, 1.9 };
  printf("blurred\n");
  vector<mat> blurred = blur_multiple(image, blvls);
  printf("images: %d\n", (int)blurred.size());

  // once blurred, pass the blurred images into a hough transform stage
  printf("ht\n");
  vector<mat> ht = hough_multiple(blurred);
  for (mat &m : ht) {
    printf("%d circles found\n", (int)m.n_cols);
  }
  vector<mat> largestht;
  for (mat &m : ht) {
    largestht.push_back(m(span::all, span(0,0)));
  }
  
  // once the hough transform has been passed, then attempt to get
  // the max pooling of the hough transformed images
  printf("edges\n");
  vector<mat> edges = edge_multiple(image, blvls);
  printf("circles\n");
  vector<cube> circles = draw_circles(blurred, largestht);
  printf("rgb conv\n");
  vector<cube> rgbedges = gray2rgb_multiple(edges);

  // draw them out
  /*printf("disp\n");
  disp_everything(rgbedges, "edges");
  disp_wait();
  printf("disp\n");
  disp_everything(circles, "circles");
  disp_wait();*/

  mat circleht(3, 0);
  for (mat &m : ht) {
    circleht = join_rows(circleht, m);
  }
  vector<cube> cropped = crop_circles(rgbimage, circleht);
  for (int i = 0; i < cropped.size(); i++) {
    disp_image("cd " + to_string(i), cropped[i]);
  }
  disp_wait();

  Ptr<SVM> classifier = load_classifier("desc.yml");

  return 0;
}
