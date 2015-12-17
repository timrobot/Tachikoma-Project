#ifndef __SB_GRIDMAP_H__
#define __SB_GRIDMAP_H__

#ifdef __cplusplus
extern "C" {
#endif

typedef struct gridnode gridnode_t;
typedef struct gridmap gridmap_t;

// Radix Tree using the granularity to determine the size
struct gridnode { // private structure
  gridnode_t **subgrid;
  int n_rows;
  int n_cols;
  float precision;
  float min_precision;

  gridnode_t *parent;
  gridmap_t *env;

  float *map;
  float min_x; // inclusive
  float max_x; // exclusive
  float min_y; // inclusive
  float max_y; // exclusive
};

struct gridmap {
  gridnode_t **grids;
  int n_grids;
  int n_gridsize;
  gridnode_t *quad[4];
  int blocksize;
  float min_precision;
};

int gridmap_create(gridmap_t *map);
void gridmap_destroy(gridmap_t *map);
float gridmap_get(gridmap_t *map, float x, float y);
void gridmap_set(gridmap_t *map, float x, float y, float value);
void gridmap_load(gridmap_t *map, char *foldername);
void gridmap_store(gridmap_t *map, char *foldername);
void gridmap_query(gridmap_t *map, float x, float y, float theta,
    float *buffer, int diameter, float precision);

#ifdef __cplusplus
}
#endif

#endif
