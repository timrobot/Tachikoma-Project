#ifndef __TK_BUTTONINPUT_H__
#define __TK_BUTTONINPUT_H__

#include <SDL2/SDL.h>

class ButtonInput {
  public:
  int x;
  int y;
  int w;
  int h;
  uint32_t c;

  bool trigger;
  ButtonInput(int _x, int _y, int _w, int _h, uint32_t _c);
  ~ButtonInput(void);
  void handle_input(void);
  bool clicked(void);
  void show(void);
};

#endif 
