#include "armv1.h"
#include "defs.h"
#include <iostream>
#include <signal.h>
#include <SDL/SDL.h>
#include <cstdint>
#include <iostream>
#include <fstream>
#include <njson/json.hpp>

#define FPS 25
#define KEYID(k) ((k)-'a')

using namespace arma;
using namespace std;
using json = nlohmann::json;
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

  // connect to the arm 
  screen = SDL_SetVideoMode(640, 480, 32, SDL_SWSURFACE);
  Arm arm;
  arm.connect();
  
  if (!arm.connected()) {
    printf("[ARM TEST] Not connected to anything, disconnecting...\n");
    arm.disconnect();
    return 1;
  }

/*  string params;
  ifstream params_file("calib_params.json");
  string temp;
  while (getline(params_file, temp)) {
    params += temp;
  }
  params_file.close();
  arm.set_calibration_params(json::parse(params));*/

  mat arm_pos(1, DOF, fill::zeros);
  mat arm_vel(1, DOF, fill::zeros);

  // run loop
  char key_pressed[26];
  bool position_en = false;
  bool velocity_en = false;
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
            case SDLK_q: key_pressed[KEYID('q')] = 1; break;
            case SDLK_a: key_pressed[KEYID('a')] = 1; break;
            case SDLK_w: key_pressed[KEYID('w')] = 1; break;
            case SDLK_s: key_pressed[KEYID('s')] = 1; break;
            case SDLK_e: key_pressed[KEYID('e')] = 1; break;
            case SDLK_d: key_pressed[KEYID('d')] = 1; break;
            case SDLK_i: key_pressed[KEYID('i')] = 1; break;
            case SDLK_j: key_pressed[KEYID('j')] = 1; break;
            case SDLK_o: key_pressed[KEYID('o')] = 1; break;
            case SDLK_k: key_pressed[KEYID('k')] = 1; break;
            case SDLK_p: key_pressed[KEYID('p')] = 1; break;
            case SDLK_l: key_pressed[KEYID('l')] = 1; break;
            case SDLK_8: velocity_en = true; position_en = false; break;
            case SDLK_9: velocity_en = false; position_en = true; break;
            case SDLK_0: velocity_en = false; position_en = false; break;
            default: break;
          }
        } break;
        case SDL_KEYUP: {
          switch (event.key.keysym.sym) {
            case SDLK_q: key_pressed[KEYID('q')] = 0; break;
            case SDLK_a: key_pressed[KEYID('a')] = 0; break;
            case SDLK_w: key_pressed[KEYID('w')] = 0; break;
            case SDLK_s: key_pressed[KEYID('s')] = 0; break;
            case SDLK_e: key_pressed[KEYID('e')] = 0; break;
            case SDLK_d: key_pressed[KEYID('d')] = 0; break;
            case SDLK_i: key_pressed[KEYID('i')] = 0; break;
            case SDLK_j: key_pressed[KEYID('j')] = 0; break;
            case SDLK_o: key_pressed[KEYID('o')] = 0; break;
            case SDLK_k: key_pressed[KEYID('k')] = 0; break;
            case SDLK_p: key_pressed[KEYID('p')] = 0; break;
            case SDLK_l: key_pressed[KEYID('l')] = 0; break;
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
    cout << "calibrated? " << arm.calibrated() << endl;

    int k_q = key_pressed[KEYID('q')];
    int k_a = key_pressed[KEYID('a')];
    int k_w = key_pressed[KEYID('w')];
    int k_s = key_pressed[KEYID('s')];
    int k_e = key_pressed[KEYID('e')];
    int k_d = key_pressed[KEYID('d')];
    int k_i = key_pressed[KEYID('i')];
    int k_j = key_pressed[KEYID('j')];
    int k_o = key_pressed[KEYID('o')];
    int k_k = key_pressed[KEYID('k')];
    int k_p = key_pressed[KEYID('p')];
    int k_l = key_pressed[KEYID('l')];
    if (velocity_en) {
      printf("velocity enabled\n");
      printf("Controls:\n");
      arm_vel(0) = (k_q - k_a);
      arm_vel(1) = (k_w - k_s);
      arm_vel(2) = (k_e - k_d);
      arm_vel(3) = (k_i - k_j);
      arm_vel(4) = (k_o - k_k);
      arm_vel(5) = (k_p - k_l);
      cout << arm_vel << endl;
    } else if (position_en) {
      printf("position enabled\n");
    } else {
      printf("everything disabled\n");
    }
    arm.move(arm_pos, arm_vel, position_en, velocity_en);
    // print out the feedback from the robot
    vec arm_sensors = arm.sense();
    printf("SENSORS:\n");
    std::cout << arm_sensors.t() << std::endl;

    // render screen
    SDL_Flip(screen);
    if (1000 / FPS > SDL_GetTicks() - start) {
      SDL_Delay(1000 / FPS - (SDL_GetTicks() - start));
    }
  }

  SDL_Quit();
  return 0;
}
