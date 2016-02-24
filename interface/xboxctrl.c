#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>
#include <linux/joystick.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include "xboxctrl.h"

#define INPUT_DIR "/dev/input/"
#define JS_PREFIX "js"
#define MAX_16BIT 0x7FFF

/** Connect to a joystick device.
 *  @param controller
 *    A pointer to a controller struct.
 */
void xboxctrl_connect(xboxctrl_t *ctrl) {
  DIR *dp;
  struct dirent *ent;

  /* find the device */
  if (!(dp = opendir(INPUT_DIR))) {
    return;
  }
  ctrl->name = NULL;
  ctrl->connected = 0;
  while ((ent = readdir(dp))) {
    if (strstr(ent->d_name, JS_PREFIX)) {
      ctrl->name = (char *)malloc((strlen(INPUT_DIR) + strlen(ent->d_name) + 1) * sizeof(char));
      sprintf(ctrl->name, "%s%s", INPUT_DIR, ent->d_name);
      ctrl->fd = open(ctrl->name, O_RDONLY);
      if (ctrl->fd == -1) {
        goto error;
      }
      ctrl->connected = 1;
      ctrl->buttons = 0;
      ctrl->axes = 0;
      memset(&ctrl->A, 0, (size_t)ctrl + sizeof(xboxctrl_t) - (size_t)&ctrl->A);
      break;
    }
  }
  if (!ctrl->connected) {
    goto error;
  }
  return;

error:
  fprintf(stderr, "Error: Cannot connect to controller\n");
  ctrl->connected = 0;
  if (ctrl->fd != -1) {
    close(ctrl->fd);
  }
  ctrl->fd = -1;
  if (ctrl->name != NULL) {
    free(ctrl->name);
  }
  ctrl->name = NULL;
  memset(ctrl, 0, (size_t)&ctrl->A - (size_t)ctrl);
}

/** Update a joystick device.
 *  @param ctrl
 *    A pointer to a controller struct.
 *  @return 0 on success, else -1
 */
int xboxctrl_update(xboxctrl_t *ctrl) {
  struct js_event event;

  /* dynamically reconnect the device */
  if (access(ctrl->name, O_RDONLY) != 0) {
    if (ctrl->connected) {
      ctrl->connected = 0;
      ctrl->fd = -1;
      ctrl->buttons = 0;
      ctrl->axes = 0;
      memset(&ctrl->A, 0, (size_t)&ctrl->HOME + sizeof(char) - (size_t)&ctrl->A);
    }
  } else {
    if (!ctrl->connected) {
      ctrl->fd = open(ctrl->name, O_RDONLY);
      if (ctrl->fd != -1) {
        ctrl->connected = 1;
      }
    }
  }
  if (!ctrl->connected) {
    return -1;
  }

  /* update device values */
  if (read(ctrl->fd, &event, sizeof(struct js_event)) == -1) {
    /* perhaps this is blocking? check later... (bug) */
    return -1;
  }
  if (event.type & JS_EVENT_BUTTON) {
    if (event.type & JS_EVENT_INIT) {
      ctrl->buttons++;
    }
    switch (event.number) {
      case 0:
        ctrl->A = event.value;
        break;
      case 1:
        ctrl->B = event.value;
        break;
      case 2:
        ctrl->X = event.value;
        break;
      case 3:
        ctrl->Y = event.value;
        break;
      case 4:
        ctrl->LB = event.value;
        break;
      case 5:
        ctrl->RB = event.value;
        break;
      case 6:
        ctrl->START = event.value;
        break;
      case 7:
        ctrl->SELECT = event.value;
        break;
      case 8:
        ctrl->HOME = event.value;
        break;
      case 9:
        ctrl->LJOY.pressed = event.value;
        break;
      case 10:
        ctrl->RJOY.pressed = event.value;
        break;
    }
  } else if (event.type & JS_EVENT_AXIS) {
    float value;
    value = (float)event.value;
    if (value > 1.0 || value < -1.0) {
      value /= (float)MAX_16BIT;
    }
    if (event.type & JS_EVENT_INIT) {
      ctrl->axes++;
    }
    switch (event.number) {
      case 0:
        ctrl->LJOY.x = value;
        break;
      case 1:
        ctrl->LJOY.y = -value;
        break;
      case 2:
        ctrl->LT = value;
        break;
      case 3:
        ctrl->RJOY.x = value;
        break;
      case 4:
        ctrl->RJOY.y = -value;
        break;
      case 5:
        ctrl->RT = value;
        break;
      case 6:
        ctrl->LEFT = value < 0.0;
        ctrl->RIGHT = value > 0.0;
        break;
      case 7:
        ctrl->UP = value < 0.0;
        ctrl->DOWN = value > 0.0;
        break;
    }
  }
  return 0;
}

/** Disconnect from a joystick device.
 *  @param controller
 *    A pointer to a controller struct.
 */
void xboxctrl_disconnect(xboxctrl_t *ctrl) {
  /* clean up */
  if (ctrl->fd != -1) {
    close(ctrl->fd);
  }
  if (ctrl->name != NULL) {
    free(ctrl->name);
  }
  memset(ctrl, 0, sizeof(xboxctrl_t));
  ctrl->fd = -1;
}
