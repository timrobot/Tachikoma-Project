#include <iostream>
#include <string>
#include <cstdio>
#include <signal.h>
#include <armadillo>
#include "astar.h"
#include "sim_window.h"
#include "draw.h"
#include "maze_gen.h"
#include "maze_imgio.h"
#include "robot.h"

#define t_delay 25
using namespace std;
using namespace arma;

void stopprog(int signo) {
  printf("Time ran out, quitting.\n");
  exit(1);
}

SDL_Surface *screen;
static bool mouseon;

/// OPTIONS ///
static int RANDOM = 1;
static int F = F_FORWARD;
static int H = H_REPEATED;
static int G = G_MAX;
static string fname;

ivec getClickedPoint(void) {
  SDL_Event *e;
  for (;;) {
    while ((e = sim_window::get_event())) {
      if (e->type == SDL_QUIT) {
        sim_window::destroy();
        exit(1);
      }
      if (e->type == SDL_MOUSEBUTTONDOWN) {
        if (mouseon == false) {
          mouseon = true;
        }
      }
      if (e->type == SDL_MOUSEBUTTONUP) {
        if (mouseon == true) {
          mouseon = false;
          return ivec({ gridx(e->button.x), gridy(screen->h - e->button.y - 1) });
        }
      }
    }
  }
  return ivec({0, 0});
}

void run_map(imat map) {
  // select two points, for now the corners
  icube pathmap;
  vector<ivec> path;
  drawGrid(pathmap, map);
  blitRGB(screen, pathmap);
  sim_window::update();
  SDL_Delay(t_delay * 5);

  // grab the start and goal positions
  bool isSame = 1;
  ivec start;
  ivec goal;
  while(isSame) {
    printf("Waiting for point 1\n");
    start = getClickedPoint();

    printf("Waiting for point 2\n");
    goal = getClickedPoint();

    if (start(0) == goal(0) && start(1) == goal(1)) {
      printf("You clicked on the same point twice. Please pick two different points.\n");
    }
    else {
      isSame = 0;
    }
  }
  Robot robot(start);
  robot.search(map, start, goal, F, H, G);
  int i_blip = 0;
  while (!robot.complete() && !robot.stuck()) {
    // search the space to find a path
    while (!robot.searchalgo->complete() && !robot.searchalgo->impossible()) {
      SDL_Event *e;
      while ((e = sim_window::get_event())) {
        if (e->type == SDL_QUIT) {
          sim_window::destroy();
          exit(1);
        }
      }
      robot.run();
      robot.searchalgo->decision_space(path);
      if (i_blip % 100 == 0) {
        drawGrid(pathmap, map);
        drawPath(pathmap, path);
        drawBot(pathmap, robot.x, robot.y);
        blitRGB(screen, pathmap);
        sim_window::update();
        SDL_Delay(t_delay);
      }
      i_blip++;
    }
    if (robot.searchalgo->impossible()) {
      printf("Impossible to find a path!\n");
      return;
    }
    // draw the final decision
    robot.searchalgo->final_decision(path);
    drawGrid(pathmap, map);
    drawPath(pathmap, path);
    drawBot(pathmap, robot.x, robot.y);
    blitRGB(screen, pathmap);
    sim_window::update();
    SDL_Delay(t_delay * 5);

    // move the robot to the new position
    if (path.size() >= 2) {
      robot.move(robot.getMotion());
    }
  }
  drawGrid(pathmap, map);
  drawBot(pathmap, robot.x, robot.y);
  blitRGB(screen, pathmap);
  sim_window::update();
  SDL_Delay(t_delay * 5);
}

int main(int argc, char *argv[]) {
  //signal(SIGALRM, stopprog);
  //alarm(120);
  srand(getpid());
  // TODO: create argument labels for generating maps and using static maps, as well as options for the simulation
  imat maze;
  int block_prob;

  if (argc < 3) {
    printf("usage: %s [random=<0..100>|file=<filename>] [forward_max|forward_min|backward|adaptive]\n", argv[0]);
    return 1;
  } else {
    string arg1 = argv[1];
    string arg2 = argv[2];
    size_t pos;
    if ((pos = arg1.find("=")) == string::npos) {
      printf("error: format is [random=<block_prob>|file=<filename>]\n");
      return 1;
    }
    if (arg1.substr(0, pos).compare("file") == 0) {
      RANDOM = 0;
      fname = arg1.substr(pos+1, arg1.size()-pos-1);
      maze = load_maze(fname);
    } else if (arg1.substr(0, pos).compare("random") == 0) {
      RANDOM = 1;
      block_prob = atoi(arg1.substr(pos+1, arg1.size()-pos-1).c_str());
      if (block_prob < 0 || block_prob > 100) {
        printf("error: probability must be between 0 to 100\n");
        return 1;
      }
      maze = maze_gen(MAZESIZE, block_prob);
    } else {
      printf("error: format is [random=<block_prob>|file=<filename>]\n");
      return 1;
    }

    if (arg2.compare("forward_max") == 0) {
      H = H_REPEATED;
      F = F_FORWARD;
      G = G_MAX;
    } else if (arg2.compare("forward_min") == 0) {
      H = H_REPEATED;
      F = F_FORWARD;
      G = G_MIN;
    } else if (arg2.compare("backward") == 0) {
      H = H_REPEATED;
      F = F_BACKWARD;
      G = G_MAX;
    } else if (arg2.compare("adaptive") == 0) {
      H = H_ADAPTIVE;
      F = F_FORWARD;
      G = G_MAX;
    } else {
      printf("error: format is [forward_max|forward_min|backward|adaptive]\n");
      return 1;
    }
  }
  // create maps
  setBlockSize(5);
  screen = sim_window::init(getGridWidth(maze.n_cols), getGridHeight(maze.n_rows));
  run_map(maze);
  sleep(1);
  sim_window::destroy();

  return 0;
}
