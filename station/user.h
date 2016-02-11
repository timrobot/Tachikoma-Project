#ifndef __TK_USER_H__
#define __TK_USER_H__

#include "xboxctrl.h"
#include "coord.h"
#define USER_SERVER   0x0001
#define USER_XBOXCTRL 0x0002
#define USER_ENABLE   1
#define USER_DISABLE  0

namespace user {
  bool connect(int type);
  bool disconnect(void);
  void set_enable(int en);
  bool get_override(void);
  int get_poses(pose3d_t *base, pose3d_t *arm);
  void log(const char *msg);

  xboxctrl_t *device_xboxctrl(void);
}

#endif
