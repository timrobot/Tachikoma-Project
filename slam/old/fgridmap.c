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

static int gridnode_create(gridnode_t *node, float min_x, float max_x, float min_y, float max_y,
    gridmap_t *env, gridnode_t *parent, float precision, float min_precision);
static void gridnode_destroy(gridnode_t *node);
static int gridnode_inRange(gridnode_t *node, float x, float y);
static float *gridnode_reference(gridnode_t *node, float x, float y, int allow_create);
static int gridnode_getIndex(gridnode_t *node, float x, float y);
static void gridnode_load(gridnode_t *node, char *filepath,
    float min_x, float max_x, float min_y, float max_y,
    int blocksize, size_t floatsize, float min_precision);
static void gridnode_store(gridnode_t *node, char *foldername, FILE *infofile);
static int gridmap_determineQuad(gridmap_t *map, float x, float y);
static void gridmap_appendGridNode(gridmap_t *map, gridnode_t *node);

static int gridnode_create(gridnode_t *node, float min_x, float max_x, float min_y, float max_y,
    gridmap_t *env, gridnode_t *parent, float precision, float min_precision) {
  node->min_x = min_x;
  node->max_x = max_x;
  node->min_y = min_y;
  node->max_y = max_y;
  node->precision = precision;
  node->min_precision = min_precision;
  node->n_rows = (int)ceil((max_y - min_y) / precision);
  node->n_cols = (int)ceil((max_x - min_x) / precision);
  if (precision == min_precision) {
    node->map = (float *)calloc(node->n_rows * node->n_cols, sizeof(float));
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

static int gridnode_inRange(gridnode_t *node, float x, float y) {
  return node->min_x <= x && x < node->max_x &&
    node->min_y <= y && y < node->max_y;
}

float *gridnode_reference(gridnode_t *node, float x, float y, int allow_create) {
  float new_precision;
  float new_width;
  float new_height;
  float new_min_x;
  float new_max_x;
  float new_min_y;
  float new_max_y;
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
      // define new parameters
      new_precision = g->precision * (float)g->n_rows;
      new_width = new_precision * (float)g->n_rows;
      new_height = new_precision * (float)g->n_cols;
      new_min_x = floor(g->min_x / new_width) * new_width;
      new_max_x = new_min_x + new_width;
      new_min_y = floor(g->min_y / new_height) * new_height;
      new_max_y = new_min_y + new_height;
      // create the parent
      g->parent = (gridnode_t *)malloc(sizeof(gridnode_t));
      gridnode_create(g->parent, new_min_x, new_max_x, new_min_y, new_max_y,
          g->env, NULL, new_precision, g->min_precision);
      // attach the parent to the current node and the env
      g->parent->subgrid[gridnode_getIndex(g->parent, g->min_x, g->min_y)] = g;
      g->env->quad[gridmap_determineQuad(g->env, g->min_x, g->min_y)] = g->parent;
      gridmap_appendGridNode(g->env, g->parent);
    }
    g = g->parent;
  }
  // resolve lower range
  while (g->precision > g->min_precision) {
    if (!g->subgrid[gridnode_getIndex(g, x, y)]) {
      if (!allow_create) {
        return NULL;
      } else if (nmaps >= MAX_MAPS) {
        fprintf(stderr, "Error: Out of memory!\n");
        return NULL;
      } else {
        nmaps++;
      }
      // define new parameters
      new_precision = g->precision / (float)g->n_rows;
      if (new_precision < g->min_precision) {
        new_precision = g->min_precision;
      }
      new_width = new_precision * (float)g->n_rows;
      new_height = new_precision * (float)g->n_cols;
      new_min_x = floor(x / new_width) * new_width;
      new_max_x = new_min_x + new_width;
      new_min_y = floor(y / new_width) * new_width;
      new_max_y = new_min_y + new_width;
      // create the child
      gridnode_t *child = (gridnode_t *)malloc(sizeof(gridnode_t));
      gridnode_create(child, new_min_x, new_max_x, new_min_y, new_max_y,
          g->env, g, new_precision, g->min_precision);
      // attach the child to the current node
      g->subgrid[gridnode_getIndex(g, x, y)] = child;
      g = child;
      gridmap_appendGridNode(g->env, child);
    } else {
      g = g->subgrid[gridnode_getIndex(g, x, y)];
    }
  }
  return &g->map[gridnode_getIndex(g, x, y)];
}

static int gridnode_getIndex(gridnode_t *node, float x, float y) {
  return (int)floor((y - node->min_y) / node->precision) * node->n_cols +
    (int)floor((x - node->min_x) / node->precision);
}

static void gridnode_load(gridnode_t *node, char *filepath,
    float min_x, float max_x, float min_y, float max_y,
    int blocksize, size_t floatsize, float min_precision) {
  float x;
  float y;
  int i;
  int j;
  int n_elems;
  int datafd;
  float *databuf;
  n_elems = blocksize * blocksize;
  databuf = (float *)malloc(sizeof(float) * n_elems);
  datafd = open(filepath, O_RDONLY);
  read(datafd, (void *)databuf, sizeof(float) * n_elems);
  close(datafd);
  for (i = 0; i < blocksize; i++) {
    for (j = 0; j < blocksize; j++) {
      x = (float)j * min_precision + min_x;
      y = (float)i * min_precision + min_y;
      *gridnode_reference(node, x, y, 1) = databuf[i * blocksize + j];
    }
  }
  free(databuf);
}

static void gridnode_store(gridnode_t *node, char *foldername, FILE *infofile) {
  char filename[256];
  int datafd;
  SDL_Surface *image;
  int i;
  int j;
  if (!node->map) {
    return;
  }
  fprintf(infofile, "L%fR%fD%fU%f\n", node->min_x, node->max_x, node->min_y, node->max_y);
  // write to a file
  sprintf(filename, "%s/L%fR%fD%fU%f.txt", foldername,
      node->min_x, node->max_x, node->min_y, node->max_y);
  datafd = open(filename, O_RDWR | O_CREAT, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
  write(datafd, (void *)node->map, sizeof(float) * node->n_rows * node->n_cols);
  close(datafd);
  // write to an image
  image = SDL_CreateRGBSurface(0, node->n_rows, node->n_cols, 32, 0, 0, 0, 0);
  for (i = 0; i < node->n_rows; i++) {
    for (j = 0; j < node->n_cols; j++) {
      float v;
      uint8_t color;
      v = node->map[gridnode_getIndex(node, j, i)];
      v = (v < 0.0f) ? 0.0f : ((v > 1.0f) ? 1.0f : v);
      color = (uint8_t)(v * 255.0f);
      ((uint32_t *)image->pixels)[i * node->n_cols + j] =
        SDL_MapRGB(image->format, color, color, color);
    }
  }
  sprintf(filename, "%s/L%fR%fD%fU%f.bmp", foldername,
      node->min_x, node->max_x, node->min_y, node->max_y);
  SDL_SaveBMP(image, filename);
  SDL_FreeSurface(image);
}

static int gridmap_determineQuad(gridmap_t *map, float x, float y) {
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
  float size;
  int i;
  size = (float)STD_GRIDSIZE;
  map->blocksize = STD_GRIDSIZE;
  map->min_precision = 1.0f;
  for (i = 0; i < 4; i++) {
    map->quad[i] = (gridnode_t *)malloc(sizeof(gridnode_t));
  }
  gridnode_create(map->quad[gridmap_determineQuad(map, 1.0f, 1.0f)],
      0.0f, size, 0.0f, size,
      map, NULL, 1.0f, map->min_precision);
  gridnode_create(map->quad[gridmap_determineQuad(map, -1.0f, 1.0f)],
      -size, 0.0f, 0.0f, size,
      map, NULL, 1.0f, map->min_precision);
  gridnode_create(map->quad[gridmap_determineQuad(map, 1.0f, -1.0f)],
      0.0f, size, -size, 0.0f,
      map, NULL, 1.0f, map->min_precision);
  gridnode_create(map->quad[gridmap_determineQuad(map, -1.0f, -1.0f)],
      -size, 0.0f, -size, 0.0f,
      map, NULL, 1.0f, map->min_precision);
  for (i = 0; i < 4; i++) {
    gridmap_appendGridNode(map, map->quad[i]);
  }
  nmaps += map->n_grids;
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

float gridmap_get(gridmap_t *map, float x, float y) {
  float *valueref;
  valueref = gridnode_reference(map->quad[gridmap_determineQuad(map, x, y)], x, y, 0);
  return valueref ? (*valueref) : 0.0f;
}

void gridmap_set(gridmap_t *map, float x, float y, float value) {
  *gridnode_reference(map->quad[gridmap_determineQuad(map, x, y)], x, y, 1) = value;
}

void gridmap_load(gridmap_t *map, char *foldername) {
  FILE *infofile;
  char buffer[256];
  int blocksize;
  size_t floatsize;
  float min_precision;
  float left;
  float right;
  float down;
  float up;
  char *line;
  size_t n;
  sprintf(buffer, "%s/info.txt", foldername);
  if (!(infofile = fopen(buffer, "r"))) {
    return;
  }
  line = NULL;
  if (getline(&line, &n, infofile) > 0) {
    sscanf(line, "%d %zd %f\n", &blocksize, &floatsize, &min_precision);
    free(line);
    line = NULL;
  }
  while (getline(&line, &n, infofile) > 0) {
    sscanf(line, "L%fR%fD%fL%f\n", &left, &right, &down, &up);
    line[strlen(line) - 1] = '\0';
    sprintf(buffer, "%s/%s.txt", foldername, line);
    gridnode_load(map->quad[gridmap_determineQuad(map, left, down)], buffer,
        left, right, down, up, blocksize, floatsize, min_precision);
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
  fprintf(infofile, "%d %zd %f\n", map->blocksize, sizeof(float), map->min_precision);
  for (i = 0; i < map->n_grids; i++) {
    gridnode_store(map->grids[i], foldername, infofile);
  }
  fclose(infofile);
}

void gridmap_query(gridmap_t *map, float x, float y, float theta,
    float *buffer, int diameter, float precision) {
  int radius;
  int i;
  int j;
  float x0;
  float y0;
  float X;
  float Y;
  radius = diameter / 2;
  for (i = 0; i < diameter; i++) {
    for (j = 0; j < diameter; j++) {
      x0 = (float)(j - radius) * precision;
      y0 = (float)(i - radius) * precision;
      X = x0 * cos(theta) - y0 * sin(theta) + x;
      Y = x0 * sin(theta) + y0 * cos(theta) + y;
      buffer[i * diameter + j] = gridmap_get(map, X, Y);
    }
  }
}
