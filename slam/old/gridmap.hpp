#ifndef __TK_GRIDMAP_H__
#define __TK_GRIDMAP_H__

#include <string>
#include <vector>

#define GM_MAX_MAPS 128
#define GM_BLOCKSIZE 64

class GridNode {
  public:
    GridNode(int x, int y, GridMap *env = NULL,
        size_t blocksize = GM_BLOCKSIZE, size_t unitsize = 1,
        GridNode *parent = NULL);
    ~GridNode(void);
    uint8_t *at(int x, int y, bool allowCreate = false);

    GridNode **subgrid;
    size_t blocksize;
    size_t unitsize;

    GridNode *parent;
    GridMap *env;
    
    uint8_t *map;
    int min_x;
    int max_x;
    int min_y;
    int max_y;

  private:
    bool inRange(int x, int y);
    int getIndex(int x, int y);
    int roundDown(int x, int radix);
};

class GridMap {
  friend class GridNode;
  public:
    GridMap(size_t blocksize = GM_BLOCKSIZE);
    ~GridMap(void);
    bool get(double x, double y);
    bool set(double x, double y, double v);
    void load(const std::string &foldername);
    void store(const std::string &foldername);

  private:
    std::vector<GridNode *> grids;
    GridNode *quad[4];
    size_t blocksize;

    void reset(void);
    int getQuad(int x, int y);
    void clearFolder(void);
};

#endif
