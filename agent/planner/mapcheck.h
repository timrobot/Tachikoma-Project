#ifndef mapcheck_h
#define mapcheck_h

#include <armadillo>

/** Check to see if a map (stored in a matrix of 1's and 0's) can find a path from A to B
 *  @param map the map where the positions are stored
 *  @param x1 the x coord of point A
 *  @param y1 the y coord of point A
 *  @param x2 the x coord of point B
 *  @param y2 the y coord of point B
 *  @return true if a path can be found, false otherwise
 */
bool isValidMap(arma::imat map, int x1, int y1, int x2, int y2);

#endif
