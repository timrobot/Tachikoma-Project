#include <iostream>
#include <signal.h>
#include <SDL/SDL.h>
#include <cstdint>
#include <iostream>
#include <fstream>
#include "tachikoma.h"

#define FPS 25
#define KEYID(k) ((k)-'a')

using namespace arma;
using namespace std;
static bool stopsig;

void stopsignal(int) {
  stopsig = true;
}

double deg2rad(double deg) {
  return deg * M_PI / 180.0;
}

int main() {
  signal(SIGINT, stopsignal);
  SDL_Init(SDL_INIT_EVERYTHING);
  SDL_Surface *screen;
  uint32_t start;

  // connect to the tachikoma
  screen = SDL_SetVideoMode(640, 480, 32, SDL_SWSURFACE);
  Tachikoma tachikoma;
  tachikoma.connect();
  
  if (!tachikoma.connected()) {
    printf("[TACHI TEST] Not connected to anything, disconnecting...\n");
    tachikoma.disconnect();
    return 1;
  }
  tachikoma.set_calibration_params("calib_params.json");

  mat leg_pos(NUM_LEGS, NUM_JOINTS, fill::zeros);
  mat leg_vel(NUM_LEGS, NUM_JOINTS, fill::zeros);
  vec wheels(NUM_LEGS, fill::zeros);

  // run loop
  char key_pressed[26];
  int legid = 0;
  bool position_en = false;
  bool velocity_en = false;
  int standstate = 0;
  int dposestate = 0;
  memset(key_pressed, 0, 26 * sizeof(char));
  while (!stopsig) {
    SDL_Event event;
    start = SDL_GetTicks();

    // grab events
    while (SDL_PollEvent(&event)) {
      switch (event.type) {
        case SDL_QUIT:
          stopsig = true;
          break;
        case SDL_KEYDOWN: {
          switch (event.key.keysym.sym) {
            case SDLK_u: key_pressed[KEYID('u')] = 1; break;
            case SDLK_h: key_pressed[KEYID('h')] = 1; break;
            case SDLK_i: key_pressed[KEYID('i')] = 1; break;
            case SDLK_j: key_pressed[KEYID('j')] = 1; break;
            case SDLK_p: key_pressed[KEYID('p')] = 1; break;
            case SDLK_l: key_pressed[KEYID('l')] = 1; break;
            case SDLK_q: key_pressed[KEYID('q')] = 1; break;
            case SDLK_w: key_pressed[KEYID('w')] = 1; break;
            case SDLK_e: key_pressed[KEYID('e')] = 1; break;
            case SDLK_a: key_pressed[KEYID('a')] = 1; break;
            case SDLK_s: key_pressed[KEYID('s')] = 1; break;
            case SDLK_d: key_pressed[KEYID('d')] = 1; break;
            case SDLK_1: legid = 0; break;
            case SDLK_2: legid = 1; break;
            case SDLK_3: legid = 2; break;
            case SDLK_4: legid = 3; break;
            case SDLK_8: velocity_en = true; position_en = false; break;
            case SDLK_9:
              velocity_en = false;
              position_en = true;
              dposestate = 0;
              standstate = -1;
              break;
            case SDLK_0: velocity_en = false; position_en = false; break;
            default: break;
          }
        } break;
        case SDL_KEYUP: {
          switch (event.key.keysym.sym) {
            case SDLK_u: key_pressed[KEYID('u')] = 0; dposestate = 1; break;
            case SDLK_h: key_pressed[KEYID('h')] = 0; dposestate = 0; break;
            case SDLK_i: key_pressed[KEYID('i')] = 0; standstate = 1; break;
            case SDLK_j: key_pressed[KEYID('j')] = 0; standstate = 0; break;
            case SDLK_p: key_pressed[KEYID('p')] = 0; standstate = -1; break;
            case SDLK_l: key_pressed[KEYID('l')] = 0; standstate = 0; break;
            case SDLK_q: key_pressed[KEYID('q')] = 0; break;
            case SDLK_w: key_pressed[KEYID('w')] = 0; break;
            case SDLK_e: key_pressed[KEYID('e')] = 0; break;
            case SDLK_a: key_pressed[KEYID('a')] = 0; break;
            case SDLK_s: key_pressed[KEYID('s')] = 0; break;
            case SDLK_d: key_pressed[KEYID('d')] = 0; break;
            default: break;
          }
        } break;
        default:
          break;
      }
    }
    if (stopsig) {
      continue;
    }

    // send over the values to the robot
    cout << "calibrated? " << tachikoma.calibrated() << endl;
    cout << "leg id? " << legid << endl;

    int k_u = key_pressed[KEYID('u')];
    int k_i = key_pressed[KEYID('i')];
    int k_p = key_pressed[KEYID('p')];
    int k_h = key_pressed[KEYID('h')];
    int k_j = key_pressed[KEYID('j')];
    int k_l = key_pressed[KEYID('l')];
    int k_q = key_pressed[KEYID('q')];
    int k_w = key_pressed[KEYID('w')];
    int k_e = key_pressed[KEYID('e')];
    int k_a = key_pressed[KEYID('a')];
    int k_s = key_pressed[KEYID('s')];
    int k_d = key_pressed[KEYID('d')];

    if (velocity_en) {
      printf("velocity en\n");
      leg_vel(legid, WAIST) = (k_u - k_h);
      leg_vel(legid, THIGH) = (k_i - k_j);
      wheels(legid) = (k_p - k_l);
    } else if (position_en) {
      printf("position en\n");
      double coeff[] = { 1.0, -1.0, -1.0, 1.0 };
      for (int i = 0; i < NUM_LEGS; i++) {
        cout << "dstate: " << coeff[i] << endl;
        leg_pos(i, WAIST) = (double)(dposestate) * M_PI_4 * coeff[i];
        leg_pos(i, THIGH) = (double)(standstate) * M_PI_4;
        switch (i) {
          case UL:
            wheels(i) = k_w - k_a - k_s + k_d - k_q + k_e;
            break;
          case UR:
            wheels(i) = k_w + k_a - k_s - k_d + k_q - k_e;
            break;
          case DL:
            wheels(i) = k_w - k_a - k_s + k_d + k_q - k_e;
            break;
          case DR:
            wheels(i) = k_w + k_a - k_s - k_d - k_q + k_e;
            break;
        }
      }
    }
    tachikoma.move(leg_pos, leg_vel, wheels, zeros<mat>(1, 1), position_en, velocity_en);
    // print out the feedback from the robot
    mat leg_sensors;
    mat leg_feedback;
    tachikoma.sense(leg_sensors, leg_feedback);
    std::cout << leg_sensors.t() << std::endl;
    std::cout << leg_feedback.t() << std::endl;

    // render screen
    SDL_Flip(screen);
    if (1000 / FPS > SDL_GetTicks() - start) {
      SDL_Delay(1000 / FPS - (SDL_GetTicks() - start));
    }
  }

  SDL_Quit();
  return 0;
}
