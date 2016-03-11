#include <cmath>
#include <fstream>
#include <iostream>
#include <sys/types.h>
#include <dirent.h>
#include "gridmap.h"

using namespace std;

static int nmaps;

static int limit(int x, int a, int b);
static double limitf(double x, double a, double b);

// FINISHED
GridNode::GridNode(int x, int y, GridMap *env,
    size_t blocksize, size_t unitsize, GridNode *parent) {
  if (unitsize != 1) {
    this->subgrid = new GridNode*[blocksize * blocksize];
    memset(this->subgrid, 0, sizeof(GridNode *) * blocksize * blocksize);
  } else {
    this->subgrid = NULL;
  }
  this->blocksize = blocksize;
  this->unitsize = unitsize;
  this->parent = parent;
  this->env = env;
  if (unitsize == 1) {
    this->map = new uint8_t[blocksize * blocksize];
    memset(this->map, 0, sizeof(uint8_t) * blocksize * blocksize);
  } else {
    this->map = NULL;
  }
  this->min_x = x;
  this->max_x = x + blocksize * unitsize;
  this->min_y = y;
  this->max_y = y + blocksize * unitsize;
}

// FINISHED
GridNode::~GridNode(void) {
  if (this->map) {
    delete this->map;
  }
}

uint8_t *GridNode::at(int x, int y, bool allowCreate) {
  // resolve upper range
  GridNode *curr = this;
  GridNode *newNode;
  while (!curr->inRange(x, y)) {
    if (!curr->parent) {
      if (!allowCreate) {
        return NULL;
      } else if (nmaps >= GM_MAX_MAPS) {
        // for now, if it goes out of bounds, just return NULL
        // next time, save old maps to disk, and load new map
        return NULL;
      } else {
        nmaps++;
      }
      newNode = new GridNode(
          curr->min_x * curr->blocksize,
          curr->min_y * curr->blocksize,
          curr->env, curr->blocksize,
          curr->unitsize * curr->blocksize, NULL);
      // attach the parent to the child
      newNode->subgrid[newNode->getIndex(curr->min_x, curr->min_y)] = curr;
      curr->parent = newNode;
      // attach the env to the parent
      if (curr->env) {
        curr->env->quad[curr->env->getQuad(x, y)] = newNode;
        curr->env->grids.push_back(newNode);
      }
      curr = newNode;
    } else {
      curr = curr->parent;
    }
  }
  // resolve lower range
  while (curr->unitsize > 1) {
    if (!curr->subgrid[curr->getIndex(x, y)]) {
      if (!allowCreate) {
        return NULL;
      } else if (nmaps >= GM_MAX_MAPS) {
        return NULL;
      } else {
        nmaps++;
      }
      newNode = new GridNode(
          this->roundDown(x, curr->unitsize),
          this->roundDown(y, curr->unitsize),
          curr->env, curr->blocksize,
          curr->unitsize / curr->blocksize, NULL);
      // attach the child to the current node
      curr->subgrid[curr->getIndex(x, y)] = newNode;
      newNode->parent = curr;
      // attach the env to the child
      if (curr->env) {
        curr->env->grids.push_back(newNode);
      }
      curr = newNode;
    } else {
      curr = curr->subgrid[curr->getIndex(x, y)];
    }
  }
  return &curr->map[curr->getIndex(x, y)];
}

int GridNode::getIndex(int x, int y) {
  return (y - this->min_y) / this->unitsize * this->blocksize +
         (x - this->min_x) / this->unitsize;
}

bool GridNode::inRange(int x, int y) {
  return this->min_x <= x && x < this->max_x &&
         this->min_y <= y && y < this->max_y;
}

int GridNode::roundDown(int x, int radix) {
  return (x - (x % radix)) - (x < 0) * radix * (x % radix != 0);
}

// FINISHED
GridMap::GridMap(size_t blocksize) {
  this->blocksize = blocksize;
  // create the nodes
  quad[0] = new GridNode(0, 0, this, blocksize);
  quad[1] = new GridNode(-blocksize, 0, this, blocksize);
  quad[2] = new GridNode(0, -blocksize, this, blocksize);
  quad[3] = new GridNode(-blocksize, -blocksize, this, blocksize);
  // place the nodes into the gridspace
  for (int i = 0; i < 4; i++) {
    grids.push_back(quad[i]);
  }
}

// FINISHED
GridMap::~GridMap(void) {
  this->reset();
}

// FINISHED
bool GridMap::get(int x, int y, double &v) {
  uint8_t *cell = this->quad[this->getQuad(x, y)]->at(x, y, false);
  v = cell ? (double)(*cell) / (double)255 : 0.0;
  return cell != NULL;
}

// FINISHED
bool GridMap::set(double x, double y, double v) {
  uint8_t *cell = this->quad[this->getQuad(x, y)]->at(x, y, true);
  if (cell) {
    *cell = (uint8_t)(limit(v, 0.0, 1.0) * 255);
  }
  return cell != NULL;
}

// FINISHED
void GridMap::load(const std::string &foldername) {
  // reset this map
  this->reset();

  // make the foldername a proper foldername
  string pfname = foldername;
  if (pfname[pfname.length()-1] != '/') {
    pfname += "/";
  }

  // open the infofile
  ifstream infofile(pfname + "info.txt");
  if (!infofile.is_open()) {
    cerr << "Could not open " + pfname + "info.txt" << endl;
    return;
  }
  
  // read the information
  int nmaps;
  int blocksize;
  string line;
  getline(infofile, line);
  if (line.length() > 0) {
    sscanf(line.c_str(), "blocksize: %d", &blocksize);
  } else {
    cerr << "Could not read the blocksize from " + pfname + "info.txt" << endl;
    return;
  }
  getline(infofile, line);
  if (line.length() > 0) {
    sscanf(line.c_str(), "nmaps: %d", &nmaps);
  }

  // load all the maps
  int left, right, up, down;
  for (int i = 0; i < nmaps; i++) {
    getline(infofile, line);
    if (line.length() > 0) {
      sscanf(line.c_str(), "L%dR%dU%dD%d.tkgm", &left, &right, &up, &down);
    }
    // use the left and up to determine which quad this map belongs in
    this->quad[this->getQuad(left, up)]->load(pfname + line);
  }

  // close the infofile and sync everything
  infofile.close();
  this->blocksize = blocksize;
}

// FINISHED
void GridMap::store(const std::string &foldername) {
  // make the foldername into a proper foldername
  string pfname = foldername;
  if (pfname[pfname.length()-1] != '/') {
    cerr << "Could not open " + pfname + "info.txt" << endl;
    return;
  }

  // reset the folder data
  this->clearFolder(pfname);

  // open the infofile
  ofstream infofile(pfname + "info.txt");

  // write the information
  infofile << "blocksize: " << this->blocksize << endl;
  infofile << "nmaps: " << this->grids.size() << endl;
  for (GridNode *grid : this->grids) {
    if (!grid->map) {
      continue;
    }
    char filename[256];
    sprintf(filename, "L%dR%dU%dD%d.tkgm",
        grid->min_x, grid->max_x, grid->max_y, grid->min_y);
    infofile << filename << endl;
    grid->store(pfname + filename);
  }

  // close the infofile
  infofile.close();
}

// FINISHED
GridMap::reset(void) {
  for (GridNode *grid : this->grids) {
    delete grid;
  }
  this->grids.clear();
  for (int i = 0; i < 4; i++) {
    quad[i] = NULL;
  }
}

// FINISHED
int GridMap::getQuad(int x, int y) {
  int index;
  index = 0;
  index |= (x < 0) << 0;
  index |= (y < 0) << 1;
  return index;
}

// FINISHED
void GridMap::clearFolder(const string &foldername) {
  struct dirent *entry;
  DIR *dp = opendir(foldername.c_str());
  if (!dp) {
    return;
  }
  struct stat sb;
  while ((entry = readdir(dp))) {
    if (strcmp(entry->d_name, ".") == 0 ||
        strcmp(entry->d_name, "..") == 0) {
      continue;
    }
    string fullpath = foldername + entry->d_name;
    stat(fullpath.c_str(), &sb);
    if (sb.st_mode & S_IFMT == S_IFREG) {
      unlink(fullpath.c_str());
    } else {
      //cerr << "Error occurred: folder exists and contains strange files. Continue [Y/n]? ";
      string response;
      //getline(cin, response);
      response = "Y";
      if (response == "" || response == "y" || response == "Y") {
        system("rm -rf " + foldername);
        mkdir(foldername.c_str());
        return;
      }
    }
  }
}

static int limit(int x, int a, int b) {
  if (x <= a) {
    return a;
  } else if (x >= b) {
    return b;
  } else {
    return x;
  }
}

static double limitf(double x, double a, double b) {
  if (x <= a) {
    return a;
  } else if (x >= b) {
    return b;
  } else {
    return x;
  }
}
