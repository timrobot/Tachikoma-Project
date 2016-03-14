#include "sim_map.h"
#include "sim_robot.h"
#include "sim_lidar.h"
#include "sim_window.h"
#include "pfilter.h"
#include "sim_landmark.h"
#include <cstdio>
#include <armadillo>
#include <SDL2/SDL.h>
#include <cassert>
#include <iostream>
#include <cstdlib>

using namespace std;
using namespace arma;

// This is a simpler landmark extractor with the goal of extracting the location of the landmarks easily
void sim_tag_extract(mat &tag_landmarks,
    mat &sensor_vals,
    vector<sim_landmark> &landmarks,
    sim_robot &robot,
    sim_map &map) {
  tag_landmarks = mat(2, landmarks.size());
  for (int lid = 0; lid < landmarks.size(); lid++) {
    tag_landmarks.col(lid) = landmarks[lid].sense(robot);
  }
}

icube doubleImageSize(icube &image) {
  icube newframe(image.n_rows/2, image.n_cols/2, 3);
  for (uword i = 0; i < image.n_rows / 2; i++) {
    for (uword j = 0; j < image.n_cols / 2; j++) {
      newframe(i, j, 0) = (image(i*2, j*2, 0) + image(i*2+1, j*2, 0) + image(i*2, j*2+1, 0) + image(i*2+1, j*2+1, 0)) / 4;
      newframe(i, j, 1) = (image(i*2, j*2, 1) + image(i*2+1, j*2, 1) + image(i*2, j*2+1, 1) + image(i*2+1, j*2+1, 1)) / 4;
      newframe(i, j, 2) = (image(i*2, j*2, 2) + image(i*2+1, j*2, 2) + image(i*2, j*2+1, 2) + image(i*2+1, j*2+1, 2)) / 4;
    }
  }
  return newframe;
}

icube partial_frame(icube frame, int x, int y, int width, int height) {
  icube partial_frame(height, width, 3, fill::zeros);
  for (int i = 0; i < (int)partial_frame.n_rows; i++) {
    for (int j = 0; j < (int)partial_frame.n_cols; j++) {
      int x_ = x - width/2 + j;
      int y_ = y - height/2 + i;
      if (x_ < 0 || x_ >= (int)frame.n_cols ||
          y_ < 0 || y_ >= (int)frame.n_rows) {
        continue;
      }
      partial_frame(i, j, 0) = frame(y_, x_, 0);
      partial_frame(i, j, 1) = frame(y_, x_, 1);
      partial_frame(i, j, 2) = frame(y_, x_, 2);
    }
  }
  return partial_frame;
}

int main() {
  srand(getpid());

  // load the map (custom)
  sim_map map;
  map.load("bengC.jpg");

  // create the landmarks (custom)
  vector<sim_landmark> landmarks;
  landmarks.push_back(sim_landmark(88, 202));
  landmarks.push_back(sim_landmark(601, 617));
  landmarks.push_back(sim_landmark(0, 171));
  landmarks.push_back(sim_landmark(98, 627));
  landmarks.push_back(sim_landmark(384, 627));
  landmarks.push_back(sim_landmark(1225, 627));
  landmarks.push_back(sim_landmark(3400, 513));
  landmarks.push_back(sim_landmark(3442, 583));
  landmarks.push_back(sim_landmark(1961, 539));
  landmarks.push_back(sim_landmark(762, 539));

  // load the robot
  sim_robot robot(&map);
  robot.set_size(8);
  robot.set_pose(70, 48, M_PI_2);
  robot.set_noise(0.2, 0.05);

  // load the lidar
  sim_lidar lidar(&robot);
  lidar.set_noise(1.0, 0.1);

/////////////////////////
//  START OF THE BOT
/////////////////////////

  // create the particle filter
  pfilter pf(1000, &map, landmarks, 70, 48, M_PI_2);
  pf.set_size(12);
  pf.set_noise(0.2, 0.05);
  
  // start up the window
  SDL_Surface *screen = sim_window::init(400, 400);
  icube frame(map.n_rows, map.n_cols, 3, fill::zeros), newframe;
  bool forward = false;
  bool backward = false;
  bool turn_left = false;
  bool turn_right = false;
  bool strafe_left = false;
  bool strafe_right = false;

  for (;;) {
    // see if something is about to quit
    SDL_Event *e;
    bool done = false;
    while ((e = sim_window::get_event())) {
      if (e->type == SDL_QUIT) {
        done = true;
      } else if (e->type == SDL_KEYDOWN) {
        // attempt to see if the user wants to make the robot move
        if (e->key.keysym.sym == SDLK_w)  { forward = true; }
        if (e->key.keysym.sym == SDLK_s)  { backward = true; }
        if (e->key.keysym.sym == SDLK_q)  { turn_left = true; }
        if (e->key.keysym.sym == SDLK_e)  { turn_right = true; }
        if (e->key.keysym.sym == SDLK_a)  { strafe_left = true; }
        if (e->key.keysym.sym == SDLK_d)  { strafe_right = true; }
      } else if (e->type == SDL_KEYUP) {
        if (e->key.keysym.sym == SDLK_w)  { forward = false; }
        if (e->key.keysym.sym == SDLK_s)  { backward = false; }
        if (e->key.keysym.sym == SDLK_q)  { turn_left = false; }
        if (e->key.keysym.sym == SDLK_e)  { turn_right = false; }
        if (e->key.keysym.sym == SDLK_a)  { strafe_left = false; }
        if (e->key.keysym.sym == SDLK_d)  { strafe_right = false; }
      }
    }
    if (done) {
      break;
    }

    // update the robot
    robot.move((strafe_right - strafe_left) * 4, (forward - backward) * 4, (turn_left - turn_right) * .1);
    pf.move((strafe_right - strafe_left) * 4, (forward - backward) * 4, (turn_left - turn_right) * .1);
    mat sensor_values = lidar.sense();
    //mat tag_landmarks;
    //sim_tag_extract(tag_landmarks, sensor_values, landmarks, robot, map);
    //pf.observe(tag_landmarks);

    // predict the position
    vec mu;
    mat sigma;
    pf.predict(mu, sigma);
    cout << "position: " << mu(0) << ", " << mu(1) << ", angle: " << mu(2) * 180 / M_PI << ", error: \n" << sigma << endl;

    // put stuff on the screen
    map.blit(frame);
    for (sim_landmark &lm : landmarks) {
      lm.blit(frame);
    }
    //lidar.blit(frame);
    pf.blit(frame);
    //robot.blit(frame);
    newframe = partial_frame(frame, (int)round(robot.x), (int)round(robot.y), screen->w, screen->h);
    sim_window::blit(screen, newframe);
    SDL_Delay(25);

    // draw the screen
    sim_window::update();
  }

  // clean up
  sim_window::destroy();
}
