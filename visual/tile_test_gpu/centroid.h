#ifndef __TK_CENTROID_H__
#define __TK_CENTROID_H__

#include "gcube.h"

/** Create two histograms of an image (only grayscale for now)
 *  @param H (output) histogram
 *  @param F (input) image
 *  @param rowhist (optional) true to get a histogram of each row, false for each column
 *  @param weighted (optional) true to get a weighted histogram, false otherwise
 */
//void gpu_hist(gcube &H, const gcube &F, bool rowhist = false, bool weighted = false);

/** Find the centroid of an image
 *  @param F image
 *  @param x centroid.x
 *  @param y centroid.y
 */
void gpu_centroid(const gcube &F, size_t &x, size_t &y);

#endif
