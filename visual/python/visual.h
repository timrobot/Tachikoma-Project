#ifndef visual_h
#define visual_h

#include "coord.h"

#define DETECT_BALL 12
#define DETECT_BASKET 13

#ifdef __cplusplus
extern "C" {
#endif

int start_visual(void);
void set_detection(int mode);
pose3d_t *get_position(int *found, int *type, int *readdata);
int stop_visual(void);

#ifdef __cplusplus
}
#endif

#endif
