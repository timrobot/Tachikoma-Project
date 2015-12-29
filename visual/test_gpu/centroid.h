#ifndef __TK_CENTROID_H__
#define __TK_CENTROID_H__

#include "gcube.h"

/** Create two histograms of an image (only grayscale for now)
 *  @param F (input) image
 *  @param H (output) histogram
 *  @param rowhist true to get a histogram of each row, false for each column
 */
void gpu_hist(const gcube &F, gcube &H, bool rowhist = false);

/** Find the centroid of an image
 *  @param F image
 *  @param x centroid.x
 *  @param y centroid.y
 */
void gpu_centroid(const gcube &F, double &x, double &y);

#endif
