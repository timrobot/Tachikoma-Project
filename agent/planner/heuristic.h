#ifndef heuristic_h
#define heuristic_h

#include <armadillo>

/** Return the manhattan distance from A to B
 *  @param x1 A.x
 *  @param y1 A.y
 *  @param x2 B.x
 *  @param y2 B.y
 *  @return the distance
 */
int mdist(int x1, int y1, int x2, int y2);
int eucdist(int x1, int y1, int x2, int y2);

#endif
