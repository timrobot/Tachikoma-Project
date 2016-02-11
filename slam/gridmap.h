#ifndef __TK_GRIDMAP_H__
#define __TK_GRIDMAP_H__

#include <stdint.h>

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
  int unitsize; // lowest of 1

  gridnode_t *parent;
  gridmap_t *env;

  uint8_t *map;
  int min_x; // inclusive
  int max_x; // exclusive
  int min_y; // inclusive
  int max_y; // exclusive
};

struct gridmap {
  gridnode_t **grids;
  int n_grids;
  int n_gridsize;
  gridnode_t *quad[4];
  int blocksize;
};

int gridmap_create(gridmap_t *map);
void gridmap_destroy(gridmap_t *map);
uint8_t gridmap_get(gridmap_t *map, int x, int y);
void gridmap_set(gridmap_t *map, int x, int y, uint8_t value);
void gridmap_load(gridmap_t *map, char *foldername);
void gridmap_store(gridmap_t *map, char *foldername);
void gridmap_query(gridmap_t *map, int x, int y, double theta,
    uint8_t *buffer, int diameter, int unitsize);

#ifdef __cplusplus
}
#endif

#endif
