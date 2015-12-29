#ifndef __TK_IMGFMT_H__
#define __TK_IMGFMT_H__

#include <string>
#include <opencv2/core/core.hpp>

#ifdef __NVCC__

#include "gcube.h"

/** Display a gcube onto the screen
 *  @param window_name the name of the window
 *  @param image the gcube to display
 */
void disp_gcube(const std::string &window_name, gcube &image);

/** Wait until the user presses a key to close the window
 */
void disp_wait(void);

/** Transform an image from RGB to Grayscale
 *  @param image the image to transform into grayscale
 *  @return the transformed image
 */
gcube gpu_rgb2gray(const gcube &image);

/** Transform an image from Grayscale to RGB
 *  @param image the image to transform into RGB
 *  @return the transformed image
 */
gcube gpu_gray2rgb(const gcube &image);

#endif
#endif
