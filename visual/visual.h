#ifndef __TK_VISUAL_H__
#define __TK_VISUAL_H__

#include <armadillo>

namespace visual {

  /** Initialize the visual space,
   *  such as the camera and/or other depth sensors.
   */
  void init(void);

  /** Check to see if the camera has been initialized
   *  @return true if a camera was found, else false
   */
  bool hasCamera(void);

  /** Get the type of the camera
   * 
   */
  bool hasDepth(void);
  
}

#endif
