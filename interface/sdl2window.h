#ifndef __TK_SDL2WINDOW_H__
#define __TK_SDL2WINDOW_H__

#include <SDL2/SDL.h>
#include <armadillo>

namespace SDL2Window {
  SDL_Surface *init(int width, int height);
  SDL_Event *get_event(void);
  void blit(SDL_Surface *s, arma::icube &frame);
  void update(void);
  void destroy(void);
}

#endif
