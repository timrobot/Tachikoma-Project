#include <njson/json.hpp>
#include <string>
#include <iostream>
#include <fstream>
#include "tachikoma.h"
#include "defs.h"
#include <signal.h>
#include <SDL/SDL.h>
#include <armadillo>

#define FPS 15

using namespace std;
using namespace arma;
using json = nlohmann::json;

static bool stopsig;

static void stopsignal(int) {
  stopsig = true;
}

int main(int argc, char *argv[]) {
  signal(SIGINT, stopsignal);

  // create a window to show the results of the calibration
  SDL_Init(SDL_INIT_EVERYTHING);
  SDL_Surface *screen;
  screen = SDL_SetVideoMode(640, 480, 32, SDL_SWSURFACE);

  // try to connect to the tachikoma's arduinos
  // NOTE: this will try to do ALL-IN-ONE Calibration
  Tachikoma *tachikoma;
  tachikoma = new Tachikoma();
  if (!tachikoma->connected()) {
    printf("[TACHI CALIBRATE] Not connected to anything, disconnecting...\n");
    tachikoma->disconnect();
    return 1;
  }

  // create space to store the position information
  mat leg_sensors;
  mat leg_feedback;

  uint32_t start;
  mat min_sensors;
  mat max_sensors;
  bool minset = false;
  bool maxset = false;

  // start reading events
  while (!stopsig) {
    SDL_Event event;
    start = SDL_GetTicks();

    while (SDL_PollEvent(&event)) {
      switch (event.type) {
        case SDL_QUIT:
          stopsig = true;
          break;
        default:
          break;
      }
    }
    if (stopsig) {
      continue;
    }

    // grab sensor readings
    tachikoma->recv(leg_sensors, leg_feedback);
    if (!minset) {
      minset = true;
      min_sensors = leg_sensors;
    } else {
      min_sensors += (leg_sensors - min_sensors) % (leg_sensors < min_sensors || min_sensors == 0);
    }
    if (!maxset) {
      maxset = true;
      max_sensors = leg_sensors;
    } else {
      max_sensors += (leg_sensors - max_sensors) % (leg_sensors > max_sensors || leg_sensors == 0);
    }
    
    cout << "[TACHI CALIBRATE] New data: \n";
    cout << leg_sensors.t() << endl;
    cout << min_sensors.t() << endl;
    cout << max_sensors.t() << endl;

    // SDL render
    SDL_Flip(screen);
    if (1000 / FPS > SDL_GetTicks() - start) {
      SDL_Delay(1000 / FPS - (SDL_GetTicks() - start));
    }
  }

  // convert to string and save to file
  json potvals = json({});
  vector<string> legnames = { "ul", "ur", "dl", "dr" };
  vector<int> legids = { UL, UR, DL, DR };
  vector<string> jointnames = { "waist", "thigh", "knee" };
  vector<int> jointids = { WAIST, THIGH, KNEE };
  for (int i = 0; i < NUM_LEGS; i++) {
    for (int j = 0; j < NUM_JOINTS; j++) {
      string legname = legnames[i];
      int legid = legids[i];
      string jointname = jointnames[j];
      int jointid = jointids[j];
      potvals[legname][jointname]["min"] = min_sensors(legid, jointid);
      potvals[legname][jointname]["max"] = max_sensors(legid, jointid);
      potvals[legname][jointname]["reversed"] = false;
    }
  }
  FILE *fp = fopen("calib_params.json", "w");
  fprintf(fp, "%s", potvals.dump().c_str());
  fclose(fp);
  cout << "[TACHI CALIBRATE] saved information to: calib_params.json\n";
  return 0;
}
