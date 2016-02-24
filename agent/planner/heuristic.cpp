#include "heuristic.h"
#include <cmath>

#define ABS(x) (((x) < 0) ? -(x) : (x))

int mdist(int x1, int y1, int x2, int y2) {
  return ABS(x1 - x2) + ABS(y1 - y2);
}

int eucdist(int x1, int y1, int x2, int y2) {
  int dx = x1 - x2;
  int dy = y1 - y2;
  return sqrt(dx * dx + dy * dy);
}
