#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <fcntl.h>
#include <cmath>
#include <SDL2/SDL.h>
#include "gridmap.h"

#define MAX_MAPS 3000
#define STD_GRIDSIZE 128

static int nmaps; // use replacement policy to push to hard drive
float dummy = 0.0f;

GridNode2::GridNode2(float min_x, float max_x, float min_y, float max_y, void *env,
    GridNode2 *parent, float precision, float min_precision) {
  // this class needs to be manaaged properly
  this->min_x = min_x;
  this->max_x = max_x;
  this->min_y = min_y;
  this->max_y = max_y;
  this->precision = precision;
  this->min_precision = min_precision;
  this->n_rows = (int)ceil((max_x - min_x) / precision);
  this->n_cols = (int)ceil((max_y - min_y) / precision);
  if (precision == min_precision) {
    this->map = new float[this->n_rows * this->n_cols];
    memset(this->map, 0, sizeof(float) * this->n_rows * this->n_cols);
    this->subgrid = NULL;
  } else {
    this->subgrid = new GridNode2 *[this->n_rows * this->n_cols];
    memset(this->subgrid, 0, sizeof(GridNode2 *) * this->n_rows * this->n_cols);
    this->map = NULL;
  }
  this->env = env;
  this->parent = parent;
}

GridNode2::~GridNode2(void) {
  if (this->subgrid) {
    for (int i = 0; i < this->n_rows * this->n_cols; i++) {
      if (this->subgrid[i]) {
        delete this->subgrid[i];
      }
    }
    delete this->subgrid;
  }
  if (this->map) {
    delete this->map;
  }
  nmaps--;
}

bool GridNode2::inRange(float x, float y) {
  return this->min_x <= x && x < this->max_x &&
         this->min_y <= y && y < this->max_y;
}

float &GridNode2::operator()(const float x, const float y) {
  //printf("\tEntered node operator(%f %f)\n", x, y);
  GridNode2 *g = this;
  GridMap *env = (GridMap *)this->env;
  float new_precision;
  float new_width;
  float new_height;
  float new_min_x;
  float new_max_x;
  float new_min_y;
  float new_max_y;
  while (!g->inRange(x, y)) { // resolve upper echelon
    //printf("\tNot in range - entering parent\n");
    if (!g->parent) {
      if (nmaps >= MAX_MAPS) {
        //printf("Out of memory! Dummy reference\n");
        return dummy;
      } else {
        nmaps++;
      }
      new_precision = g->precision * g->n_rows;
      new_width = new_precision * g->n_rows;
      new_height = new_precision * g->n_cols;
      new_min_x = floor(g->min_x / new_width) * new_width;
      new_max_x = new_min_x + new_width;
      new_min_y = floor(g->min_y / new_height) * new_height;
      new_max_y = new_min_y + new_height;
      g->parent = new GridNode2(
          new_min_x, new_max_x, new_min_y, new_max_y, this->env,
          NULL, new_precision, this->min_precision);
      g->parent->subgrid[g->parent->getIndex(g->min_x, g->min_y)] = g;
      env->quad[env->determineQuad(new_min_x, new_min_y)] = g->parent;
      env->grids.push_back(g->parent);
    }
    g = g->parent;
  }
  while (g->precision > this->min_precision) { // resolve lower echelon
    //printf("\tEntering child\n");
    if (!g->subgrid[g->getIndex(x, y)]) {
      if (nmaps >= MAX_MAPS) {
        printf("Out of memory! Dummy reference\n");
        return dummy;
      } else {
        nmaps++;
      }
      new_precision = g->precision / g->n_rows;
      if (new_precision < this->min_precision) {
        new_precision = this->min_precision;
      }
      new_width = new_precision * g->n_rows;
      new_height = new_precision * g->n_cols;
      new_min_x = floor(x / new_width) * new_width;
      new_max_x = new_min_x + new_width;
      new_min_y = floor(y / new_height) * new_height;
      new_max_y = new_min_y + new_height;
      printf("\tChild doesn't exist, creating %f %f %f %f\n", new_min_x, new_max_x, new_min_y, new_max_y);
      GridNode2 *child = new GridNode2(
          new_min_x, new_max_x, new_min_y, new_max_y, this->env,
          g, new_precision, this->min_precision);
      g->subgrid[g->getIndex(x, y)] = child;
      g = child;
      env->grids.push_back(g);
    } else {
      g = g->subgrid[g->getIndex(x, y)];
    }
  }
  //printf("\tFound\n");
  return g->map[g->getIndex(x, y)];
}

int GridNode2::getIndex(float x, float y) {
  return (int)floor((y - this->min_y) / this->precision) * this->n_cols +
         (int)floor((x - this->min_x) / this->precision);
}

void GridNode2::store(const std::string &foldername, FILE *infofile) {
  if (this->map) {
    char filename[256];
    // write to a file
    sprintf(filename, "L%fR%fD%fU%f.txt", this->min_x, this->max_x, this->min_y, this->max_y);
    fprintf(infofile, "L%fR%fD%fU%f\n", this->min_x, this->max_x, this->min_y, this->max_y);
    int datafile = open((foldername + "/" + filename).c_str(), O_RDWR | O_CREAT,
      S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
    write(datafile, (void *)this->map, sizeof(float) * this->n_rows * this->n_cols);
    close(datafile);
    // write to an image
    SDL_Surface *image = SDL_CreateRGBSurface(0, this->n_cols, this->n_rows, 32, 0, 0, 0, 0);
    for (int i = 0; i < this->n_rows; i++) {
      for (int j = 0; j < this->n_cols; j++) {
        float v = this->map[this->getIndex(j, i)];
        v = (v < 0.0f) ? 0.0f : ((v > 1.0f) ? 1.0f : v);
        uint8_t color = (uint8_t)(v * 255.0f);
        ((uint32_t *)image->pixels)[i * this->n_cols + j] =
            SDL_MapRGB(image->format, color, color, color);
      }
    }
    sprintf(filename, "L%fR%fD%fU%f.bmp", this->min_x, this->max_x, this->min_y, this->max_y);
    SDL_SaveBMP(image, (foldername + "/" + filename).c_str());
    SDL_FreeSurface(image);
  } else {
    for (int i = 0; i < this->n_rows; i++) {
      for (int j = 0; j < this->n_cols; j++) {
        if (this->subgrid[this->getIndex(j, i)]) {
          this->subgrid[this->getIndex(j, i)]->store(foldername, infofile);
        }
      }
    }
  }
}

GridMap::GridMap(void) {
  this->blocksize = 64;
  float size = (float)this->blocksize;
  this->min_precision = 1.0f;
  this->quad[0] = new GridNode2(0.0f, size, 0.0f, size,
      this, NULL, 1.0f, this->min_precision);
  this->quad[1] = new GridNode2(-size, 0.0f, 0.0f, size,
      this, NULL, 1.0f, this->min_precision);
  this->quad[2] = new GridNode2(0.0f, size, -size, 0.0f,
      this, NULL, 1.0f, this->min_precision);
  this->quad[3] = new GridNode2(-size, 0.0f, -size, 0.0f,
      this, NULL, 1.0f, this->min_precision);
  this->grids.push_back(this->quad[0]);
  this->grids.push_back(this->quad[1]);
  this->grids.push_back(this->quad[2]);
  this->grids.push_back(this->quad[3]);
  nmaps += 4;
}

GridMap::~GridMap(void) {
  for (int i = 0; i < 4; i++) {
    if (this->quad[i]) {
      delete this->quad[i];
      this->quad[i] = NULL;
    }
  }
  this->grids.clear();
}

float &GridMap::operator()(const float x, const float y) {
  //printf("Entered operator(%f %f) => quad[%d]\n", x, y, this->determineQuad(x, y));
  return (*this->quad[this->determineQuad(x, y)])(x, y);
}

int GridMap::determineQuad(float x, float y) {
  uint32_t quadindex = 0;
  quadindex |= 0x00000001 * (x < 0);
  quadindex |= 0x00000002 * (y < 0);
  return (int)quadindex;
}

void GridMap::load(const std::string &foldername) {
  DIR *dp = opendir(foldername.c_str());
  if (!dp) {
    return;
  }
  closedir(dp);
  FILE *infofile = fopen((foldername + "/info.txt").c_str(), "r");
  if (!infofile) {
    return;
  }

  int blocksize;
  size_t floatsize;
  float min_precision;
  float left;
  float right;
  float down;
  float up;

  char *line = NULL;
  size_t n;
  bool valid = false;
  if (getline(&line, &n, infofile) > 0) {
    valid = strcmp(line, "begin.\n") == 0;
    free(line);
    line = NULL;
  }
  if (!valid) {
    return;
  }
  if (getline(&line, &n, infofile) > 0) {
    sscanf(line, "%d %zd %f\n", &blocksize, &floatsize, &min_precision);
    free(line);
    line = NULL;
  }
  while (getline(&line, &n, infofile) > 0) {
    if (strcmp(line, "end.\n") == 0) {
      free(line);
      line = NULL;
      break;
    }
    line[strlen(line) - 1] = '\0';
    sscanf(line, "L%fR%fD%fU%f", &left, &right, &down, &up);
    this->loadMap(foldername + "/" + line + ".txt",
        left, right, down, up, blocksize, floatsize, min_precision);
    free(line);
    line = NULL;
  }
}

void GridMap::loadMap(const std::string &filepath, float min_x, float max_x, float min_y, float max_y,
    int blocksize, size_t floatsize, float min_precision) {
  // naive implementation for best results
  int num_elems = blocksize * blocksize;
  float *map = new float[num_elems];
  int datafile = open(filepath.c_str(), O_RDONLY);
  read(datafile, (void *)map, sizeof(float) * num_elems);
  close(datafile);
  for (int i = 0; i < blocksize; i++) { // inefficient but works
    for (int j = 0; j < blocksize; j++) {
      int x = (float)j * min_precision + min_x;
      int y = (float)i * min_precision + min_y;
      (*this->quad[this->determineQuad(x, y)])(x, y) = map[i * blocksize + j];
    }
  }
}

void GridMap::store(const std::string &foldername) {
  // delete the current existing directory
  DIR *dp = opendir(foldername.c_str());
  if (dp) {
    closedir(dp);
    system(("rm -rf " + foldername).c_str());
  }
  // create a new directory and info file
  mkdir(foldername.c_str(), 0755);
  FILE *infofile = fopen((foldername + "/info.txt").c_str(), "w");
  for (int i = 0; i < 4; i++) {
    if (!this->quad[i]) {
      fclose(infofile);
      return;
    }
  }
  fprintf(infofile, "begin.\n");
  fprintf(infofile, "%d %zd %f\n", this->blocksize, sizeof(float), this->min_precision);

  // store the images
  for (int i = 0; i < 4; i++) {
    this->quad[i]->store(foldername, infofile);
  }

  fprintf(infofile, "end.\n");
  fclose(infofile);
}

void GridMap::clear(void) {
  for (GridNode2 *g : this->grids) {
    if (g->map) {
      memset(g->map, 0, sizeof(float) * this->blocksize * this->blocksize);
    }
  }
}

arma::mat GridMap::getPortion(float x, float y, float theta, int radius, float precision) {
  arma::mat map = arma::zeros<arma::mat>(radius, radius);
  if (radius == 0) {
    return map;
  }
  //printf("choosing preprocessing metadata...\n");
  x -= (float)radius / 2;
  y -= (float)radius / 2;
  //printf("filling in map...\n");
  for (int i = 0; i < radius; i++) {
    for (int j = 0; j < radius; j++) {
      float X = x + (float)j * precision;
      float Y = y + (float)i * precision;
      //printf("filling: [%d %d] : { %f %f }\n", i, j, X, Y);
      float value = (*this->quad[this->determineQuad(X, Y)])(X, Y);
      map(i, j) = (double)value;
    }
  }
  //printf("points on map: %lf\n", arma::accu(map));
  return map;
}
