#ifndef __TK_VISUAL_H__
#define __TK_VISUAL_H__

#include <armadillo>

namespace visual {

  /** Initialize the visual space
   */
  void init(void);

  /** Check to see if the camera has been initialized
   *  @return true if a camera was found, else false
   */
  bool camera_opened(void);

  /** Check to see if the depth has been initialized
   * 
   */
  bool depth_opened(void);

  /** Try to grab a depth frame
   *  @return an error code if the depth frame cannot be accessed
   */
  int get_depth(arma::mat &depth_frame);

}

#endif
