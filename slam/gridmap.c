#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <dirent.h>
#include <unistd.h>
#include <SDL2/SDL.h>
#include "gridmap.h"

#define MAX_MAPS 1024
#define STD_GRIDSIZE 256

static int nmaps;

static int gridnode_create(gridnode_t *node, int min_x, int max_x, int min_y, int max_y,
    gridmap_t *env, gridnode_t *parent, int unitsize);
static void gridnode_destroy(gridnode_t *node);
static int gridnode_inRange(gridnode_t *node, int x, int y);
static uint8_t *gridnode_reference(gridnode_t *node, int x, int y, int allow_create);
static int gridnode_getIndex(gridnode_t *node, int x, int y);
static void gridnode_load(gridnode_t *node, char *filepath,
    int min_x, int max_x, int min_y, int max_y, int blocksize);
static void gridnode_store(gridnode_t *node, char *foldername, FILE *infofile);
static int gridmap_getQuad(gridmap_t *map, int x, int y);
static void gridmap_appendGridNode(gridmap_t *map, gridnode_t *node);

static int gridnode_create(gridnode_t *node, int min_x, int max_x, int min_y, int max_y,
    gridmap_t *env, gridnode_t *parent, int unitsize) {
  node->min_x = min_x;
  node->max_x = max_x;
  node->min_y = min_y;
  node->max_y = max_y;
  node->unitsize = unitsize;
  node->n_rows = (max_y - min_y) / unitsize;
  node->n_cols = (max_x - min_x) / unitsize;
  if (unitsize == 1) {
    node->map = (uint8_t *)calloc(node->n_rows * node->n_cols, sizeof(uint8_t));
    node->subgrid = NULL;
  } else {
    node->subgrid = (gridnode_t **)calloc(node->n_rows * node->n_cols, sizeof(gridnode_t *));
    node->map = NULL;
  }
  node->env = env;
  node->parent = parent;
  return 0;
}

static void gridnode_destroy(gridnode_t *node) {
  // careful - make non-recursive later on
  if (node->subgrid) {
    free(node->subgrid);
    node->subgrid = NULL;
  }
  if (node->map) {
    free(node->map);
    node->map = NULL;
  }
}

static int gridnode_inRange(gridnode_t *node, int x, int y) {
  return node->min_x <= x && x < node->max_x && node->min_y <= y && y < node->max_y;
}

uint8_t *gridnode_reference(gridnode_t *node, int x, int y, int allow_create) {
  gridnode_t *new_node;
  gridnode_t *g;
  g = node;
  // resolve upper range
  while (!gridnode_inRange(g, x, y)) {
    if (!g->parent) {
      if (!allow_create) {
        return NULL;
      } else if (nmaps >= MAX_MAPS) {
        fprintf(stderr, "Error: Out of memory!\n");
        return NULL;
      } else {
        nmaps++;
      }
      new_node = (gridnode_t *)malloc(sizeof(gridnode_t));
      // define new parameters
      new_node->n_rows = g->n_rows;
      new_node->n_cols = g->n_cols;
      new_node->unitsize = g->unitsize * g->n_rows;
      new_node->subgrid = (gridnode_t **)calloc(g->n_rows * g->n_cols, sizeof(gridnode_t *));
      new_node->map = NULL;
      // making the assumption that the structure is a quad starting at 0 in at least min or max
      // not a general purpose algorithm
      new_node->min_x = g->min_x * g->n_cols;
      new_node->max_x = g->max_x * g->n_cols;
      new_node->min_y = g->min_y * g->n_rows;
      new_node->max_y = g->max_y * g->n_rows;
      // attach the parent to the current node and the env
      new_node->subgrid[gridnode_getIndex(new_node, g->min_x, g->min_y)] = g;
      g->parent = new_node;
      new_node->env = g->env;
      g->env->quad[gridmap_getQuad(g->env, g->min_x, g->min_y)] = new_node;
      gridmap_appendGridNode(g->env, new_node);
      g = new_node;
    } else {
      g = g->parent;
    }
  }
  // resolve lower range
  while (g->unitsize > 1) {
    if (!g->subgrid[gridnode_getIndex(g, x, y)]) {
      int new_blocksize;
      if (!allow_create) {
        return NULL;
      } else if (nmaps >= MAX_MAPS) {
        fprintf(stderr, "Error: Out of memory!\n");
        return NULL;
      } else {
        nmaps++;
      }
      new_node = (gridnode_t *)malloc(sizeof(gridnode_t));
      // define new parameters
      new_node->n_rows = g->n_rows;
      new_node->n_cols = g->n_cols;
      new_node->unitsize = g->unitsize / g->n_rows;
      if (new_node->unitsize == 1) {
        new_node->map = (uint8_t *)calloc(g->n_rows * g->n_cols, sizeof(uint8_t));
        new_node->subgrid = NULL;
      } else {
        new_node->subgrid = (gridnode_t **)calloc(g->n_rows * g->n_cols, sizeof(gridnode_t *));
        new_node->map = NULL;
      }
      new_blocksize = new_node->unitsize * g->n_rows;
      if (x >= 0) {
        new_node->min_x = x - (x % new_blocksize);
        new_node->max_x = new_node->min_x + new_blocksize;
      } else {
        x++;
        new_node->max_x = x - (x % new_blocksize);
        new_node->min_x = new_node->max_x - new_blocksize;
        x--;
      }
      if (y >= 0) {
        new_node->min_y = y - (y % new_blocksize);
        new_node->max_y = new_node->min_y + new_blocksize;
      } else {
        y++;
        new_node->max_y = y - (y % new_blocksize);
        new_node->min_y = new_node->max_y - new_blocksize;
        y--;
      }
      // attach the child to the current node
      g->subgrid[gridnode_getIndex(g, x, y)] = new_node;
      new_node->parent = g;
      new_node->env = g->env;
      gridmap_appendGridNode(g->env, new_node);
      g = new_node;
    } else {
      g = g->subgrid[gridnode_getIndex(g, x, y)];
    }
  }
  return &g->map[gridnode_getIndex(g, x, y)];
}

static int gridnode_getIndex(gridnode_t *node, int x, int y) {
  return (y - node->min_y) / node->unitsize * node->n_cols + (x - node->min_x) / node->unitsize;
}

// fix me
static void gridnode_load(gridnode_t *node, char *filepath,
    int min_x, int max_x, int min_y, int max_y, int blocksize) {
  int x;
  int y;
  int i;
  int j;
  int n_elems;
  int datafd;
  uint8_t *databuf;
  n_elems = blocksize * blocksize;
  databuf = (uint8_t *)malloc(sizeof(uint8_t) * n_elems);
  datafd = open(filepath, O_RDONLY);
  read(datafd, (void *)databuf, sizeof(uint8_t) * n_elems);
  close(datafd);
  for (i = 0; i < blocksize; i++) {
    for (j = 0; j < blocksize; j++) {
      x = min_x + j;
      y = min_y + i;
      *gridnode_reference(node, x, y, 1) = databuf[i * blocksize + j];
    }
  }
  free(databuf);
}

// fix me
static void gridnode_store(gridnode_t *node, char *foldername, FILE *infofile) {
  char filename[256];
  int datafd;
  int i;
  int j;
  if (!node->map) {
    return;
  }
  fprintf(infofile, "L%dR%dD%dU%d\n", node->min_x, node->max_x, node->min_y, node->max_y);
  sprintf(filename, "%s/L%dR%dD%dU%d.txt", foldername,
      node->min_x, node->max_x, node->min_y, node->max_y);
  datafd = open(filename, O_RDWR | O_CREAT, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
  write(datafd, (void *)node->map, sizeof(uint8_t) * node->n_rows * node->n_cols);
  close(datafd);
}

static int gridmap_getQuad(gridmap_t *map, int x, int y) {
  int index;
  index = 0;
  index |= (x < 0) << 0;
  index |= (y < 0) << 1;
  return index;
}

static void gridmap_appendGridNode(gridmap_t *map, gridnode_t *node) {
  if (!map->grids) {
    map->n_gridsize = 8;
    map->grids = (gridnode_t **)malloc(map->n_gridsize * sizeof(gridnode_t *));
    map->n_grids = 0;
  } else if (map->n_grids >= map->n_gridsize) {
    gridnode_t **new_grids;
    new_grids = (gridnode_t **)malloc(map->n_gridsize * 2 * sizeof(gridnode_t *));
    // Bottleneck!
    memcpy(new_grids, map->grids, map->n_gridsize * sizeof(gridnode_t *));
    map->n_gridsize *= 2;
    free(map->grids);
    map->grids = new_grids;
  }
  map->grids[map->n_grids++] = node;
}

int gridmap_create(gridmap_t *map) {
  int i;
  map->blocksize = STD_GRIDSIZE;
  for (i = 0; i < 4; i++) {
    map->quad[i] = (gridnode_t *)malloc(sizeof(gridnode_t));
    nmaps++;
  }
  gridnode_create(map->quad[0], 0, map->blocksize, 0, map->blocksize, map, NULL, 1);
  gridnode_create(map->quad[1], -map->blocksize, 0, 0, map->blocksize, map, NULL, 1);
  gridnode_create(map->quad[2], 0, map->blocksize, -map->blocksize, 0, map, NULL, 1);
  gridnode_create(map->quad[3], -map->blocksize, 0, -map->blocksize, 0, map, NULL, 1);
  map->grids = NULL;
  for (i = 0; i < 4; i++) {
    gridmap_appendGridNode(map, map->quad[i]);
  }
  return 0;
}

void gridmap_destroy(gridmap_t *map) {
  int i;
  if (map->grids) {
    for (i = 0; i < map->n_grids; i++) {
      gridnode_destroy(map->grids[i]);
      free(map->grids[i]);
      nmaps--;
    }
    free(map->grids);
  }
  memset(map, 0, sizeof(gridmap_t));
}

uint8_t gridmap_get(gridmap_t *map, int x, int y) {
  uint8_t *valueref;
  valueref = gridnode_reference(map->quad[gridmap_getQuad(map, x, y)], x, y, 0);
  return valueref ? (*valueref) : 0;
}

void gridmap_set(gridmap_t *map, int x, int y, uint8_t value) {
  *gridnode_reference(map->quad[gridmap_getQuad(map, x, y)], x, y, 1) = value;
}

void gridmap_load(gridmap_t *map, char *foldername) {
  FILE *infofile;
  char buffer[256];
  int blocksize;
  int left;
  int right;
  int down;
  int up;
  char *line;
  size_t n;
  sprintf(buffer, "%s/info.txt", foldername);
  if (!(infofile = fopen(buffer, "r"))) {
    return;
  }
  line = NULL;
  if (getline(&line, &n, infofile) > 0) {
    sscanf(line, "%d\n", &blocksize);
    free(line);
    line = NULL;
  }
  while (getline(&line, &n, infofile) > 0) {
    sscanf(line, "L%dR%dD%dL%d\n", &left, &right, &down, &up);
    line[strlen(line) - 1] = '\0';
    sprintf(buffer, "%s/%s.txt", foldername, line);
    gridnode_load(map->quad[gridmap_getQuad(map, left, down)], buffer,
        left, right, down, up, blocksize);
    free(line);
    line = NULL;
  }
}

void gridmap_store(gridmap_t *map, char *foldername) {
  char buffer[256];
  int i;
  DIR *dp;
  FILE *infofile;
  if ((dp = opendir(foldername))) {
    closedir(dp);
    sprintf(buffer, "rm -rf %s", foldername);
    system(buffer);
  }
  mkdir(foldername, 0755);
  sprintf(buffer, "%s/info.txt", foldername);
  infofile = fopen(buffer, "w");
  fprintf(infofile, "%d\n", map->blocksize);
  for (i = 0; i < map->n_grids; i++) {
    gridnode_store(map->grids[i], foldername, infofile);
  }
  fclose(infofile);
}

void gridmap_query(gridmap_t *map, int x, int y, double theta,
    uint8_t *buffer, int diameter, int unitsize) {
  double radius;
  int i;
  int j;
  double x0;
  double y0;
  int X;
  int Y;
  radius = (double)diameter / 2.0;
  for (i = 0; i < diameter; i++) {
    for (j = 0; j < diameter; j++) {
      x0 = ((double)j - radius) * unitsize;
      y0 = ((double)i - radius) * unitsize;
      X = (int)round(x0 * cos(theta) - y0 * sin(theta)) + x;
      Y = (int)round(x0 * sin(theta) + y0 * cos(theta)) + y;
      buffer[i * diameter + j] = gridmap_get(map, X, Y);
    }
  }
}
