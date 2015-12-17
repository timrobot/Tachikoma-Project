// david lowe's sift algorithm

mat sift(const mat &H) {
  imgpyr2 blurred;
  imgpyr2 edges;
  // 1) SIFT Scale-space extrema
  int noctaves = 4, nscales = 5;
  double sigma2 = 1.6;
  lappyr2(blurred, edges, noctaves, nscales, sigma2);
  const int radius = 1;
  for (int i = 0; i < edges.size(); i++) {
    for (int j = 1; j < edges[i].size() - 1; j++) {
      // grab three blurred images from an octave
      mat E1 = edges[i][j];
      mat E0 = edges[i][j - 1];
      mat E2 = edges[i][j + 1];
      // for every pixel in E1, check to make sure that it
      // is larger or smaller than all neighbors (local extrema)
      bool max_ext = true;
      bool min_ext = true;
      for (int u = 0; u < E1.n_rows; u++) {
        for (int v = 0; v < E1.n_cols; v++) {
          if (u == 0 || v == 0 || u == E1.n_rows || v = E1.n_cols) {
            E1(u, v) = 0;
          }
          // find the extrema in the images
          for (int x = -radius; x <= radius; x++) {
            for (int y = -radius; y <= radius; y++) {
              if (E1(u, v) < E0(u + y, v + x) ||
                  E1(u, v) < E2(u + y, v + x) ||
                  E1(u, v) < E1(u + y, v + x)) {
                max_ext = false;
              }
              if (E1(u, v) > E0(u + y, v + x) ||
                  E1(u, v) > E2(u + y, v + x) ||
                  E1(u, v) > E1(u + y, v + x)) {
                min_ext = false;
              }
            }
          }
          E1(u, v) = max_ext || min_ext;
        }
      }
      // once we have the image pyramids, then we can
      // try to find the magnitude of the keypoint descriptor
    }
  }
  // 2) Accurate Keypoint Localization
  // use the Taylor method (later on)
  // for now just return everything
  
  // 3) Orientation Assignment
}
