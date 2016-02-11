#ifndef __SB_GRIDMAP_H__
#define __SB_GRIDMAP_H__

#include <cstdio>
#include <string>
#include <vector>
#include <armadillo>

extern float dummy;

// Radix Tree using the granularity to determine the size
class GridNode2 {
  public:
    GridNode2(float min_x, float max_x, float min_y, float max_y, void *env,
        GridNode2 *parent = NULL, float precision = 1.0f, float min_precision = 1.0f);
    ~GridNode2(void);
    bool inRange(float x, float y);
    float &operator()(const float x, const float y);
    int getIndex(float x, float y);
    void store(const std::string &foldername, FILE *fp);

    GridNode2 **subgrid;
    int n_rows;
    int n_cols;
    float precision;
    float min_precision;

    GridNode2 *parent;
    void *env;

    float *map;
    float min_x;
    float max_x;
    float min_y;
    float max_y;
};

class GridMap { // 2d
  public:
    GridMap(void);
    ~GridMap(void);
    float &operator()(const float x, const float y);
    int determineQuad(float x, float y);
    void load(const std::string &foldername);
    void store(const std::string &foldername);
    void clear(void);
    arma::mat getPortion(float x, float y, float theta,
        int radius, float precision = 1.0f);
    std::vector<GridNode2 *> grids;
    GridNode2 *quad[4];

  private:
    void loadMap(const std::string &filepath,
        float min_x, float max_x, float min_y, float max_y,
        int blocksize, size_t floatsize, float min_precision);
    int blocksize;
    float min_precision;
};

#endif
