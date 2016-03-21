#include "buttoninput.h"

ButtonInput::ButtonInput(int _x, int _y, int _w, int _h, uint32_t _c) {
  x = _x;
  y = _y;
  w = _w;
  h = _h;
  c = _c;
  trigger = false;
}

ButtonInput::~ButtonInput() {
}

void ButtonInput::handle_input() {
  trigger = false;
  if (event.type == SDL_MOUSEBUTTONDOWN) {
    if (event.button.x >= x && event.button.x <= x + w &&
        event.button.y >= y && event.button.y <= y + h) {
      trigger = true;
    }
  }
}

bool ButtonInput::clicked() {
  return trigger;
}

void ButtonInput::show() {
  SDL_Rect r;
  r.x = x;
  r.y = y;
  r.w = w;
  r.h = h;
  SDL_FillRect(screen, &r, c);
}
