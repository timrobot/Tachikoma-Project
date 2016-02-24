#include "map.h"

static icube cvt_opencv2arma(cv::Mat &cv_image) {
  assert(cv_image.channels() == 3);
  icube image(cv_image.rows, cv_image.cols, 3);
  for (uword i = 0; i < image.n_rows; i++) {
    for (uword j = 0; j < image.n_cols; j++) {
      cv::Vec3b color = cv_image.at<cv::Vec3b>(i, j);
      image(i, j, 0) = color[2];
      image(i, j, 1) = color[1];
      image(i, j, 2) = color[0];
    }
  }
  return image;
}

static imat load_maze(string &maze_name) {
  cv::Mat img = cv::imread(maze_name);
  icube image = cvt_opencv2arma(img);
  imat maze = image.slice(0) + image.slice(1) + image.slice(2);
  return ones<imat>(maze.n_rows, maze.n_cols) % (maze < 128);
}
