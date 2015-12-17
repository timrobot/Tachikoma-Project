#include <vector>
#include <armadillo>
#include <cstdio>
#include <cmath>
#include <sys/time.h>
#include <SDL2/SDL.h>
#include <signal.h>
#include <ctime>
#include <iostream>
#include "lidar.h"
#include "gridmap.h"

static int stop_signal;
static FILE *testdata;
static int testcounter;

#define WINDOW_SIZE 500

void stopTest(int signo) {
  stop_signal = 1;
}

void nsleep(double seconds) {
  struct timespec sleeptime;
  sleeptime.tv_sec = (int)floor(seconds);
  sleeptime.tv_nsec = (int)floor((seconds - (double)sleeptime.tv_sec) * 1000000000);
  nanosleep(&sleeptime, NULL);
}

std::vector<polar_t> test_readPoints(void) {
  char *line = NULL;
  size_t n;
  std::vector<polar_t> points;
  polar_t pt;
  ssize_t status;
  if (!testdata) {
    testdata = fopen("lidardata.txt", "r");
    getline(&line, &n , testdata);
  }
  while ((status = getline(&line, &n, testdata)) != -1) {
    if (strstr(line, "segment")) {
      printf("%s", line);
      free(line);
      break;
    }
    sscanf(line, "%f %f\n", &pt.radius, &pt.theta);
    points.push_back(pt);
    free(line);
    line = NULL;
  }
  if (status == -1) {
    if (testdata) {
      fclose(testdata);
      testdata = NULL;
      testcounter = -1;
    }
  }
  return points;
}

int main(int argc, char *argv[]) {

  signal(SIGINT, stopTest);

  if (argc != 4) {
    fprintf(stderr, "usage: %s [folder name] [x] [y]\n", argv[0]);
    return 1;
  }

  // get the lidar device
  Lidar lidar;
  if (!lidar.connected()) {
    fprintf(stderr, "Error occurred in connecting to the lidar\n");
    return 1;
  }

  // create a gridmap
  //printf("Initializing the grid map\n");
  gridmap_t map;
  gridmap_create(&map);
//  map.load(argv[1]);

  // create a window to display the graph
  printf("Initializing the SDL layer\n");
  if (SDL_Init(SDL_INIT_VIDEO) != 0) {
    fprintf(stderr, "Error occurred in initializing the video: %s\n", SDL_GetError());
    return 1;
  }
  SDL_Window *window = SDL_CreateWindow(
      "Minimap",
      SDL_WINDOWPOS_CENTERED,
      SDL_WINDOWPOS_CENTERED,
      WINDOW_SIZE, WINDOW_SIZE, SDL_WINDOW_SHOWN);
  if (!window) {
    fprintf(stderr, "Error occurred in creating the window: %s\n", SDL_GetError());
    SDL_Quit();
    return 1;
  }
  // get the renderer from the GPU
  SDL_Renderer *renderer = SDL_CreateRenderer(window, -1,
      SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
  if (!renderer) {
    fprintf(stderr, "Error occurred in creating the renderer: %s\n", SDL_GetError());
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 1;
  }
  // create an image for the minimap
  SDL_Surface *screen = SDL_CreateRGBSurface(0, WINDOW_SIZE, WINDOW_SIZE, 32, 0, 0, 0, 0);
  if (!screen) {
    fprintf(stderr, "Error occurred in creating a surface for the minimap: %s\n", SDL_GetError());
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 1;
  }
  SDL_Texture *texture = SDL_CreateTextureFromSurface(renderer, screen);
  if (!texture) {
    fprintf(stderr, "Error occurred in creating a texture from a surface and renderer: %s\n", SDL_GetError());
    SDL_FreeSurface(screen);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 1;
  }

  // take a distribution of the map (noise and everything)
  printf("Displaying sample test\n");
  int x0 = atoi(argv[2]);
  int y0 = atoi(argv[3]);
  bool done = false;
  std::vector<arma::vec> points;
  uint8_t framebuffer[WINDOW_SIZE * WINDOW_SIZE];
  struct timeval lasttime;
  gettimeofday(&lasttime, NULL);
  while (!stop_signal) {
    // detect for exiting the window
    SDL_Event e;
    while (SDL_PollEvent(&e)) {
      if (e.type == SDL_QUIT) {
        done = true;
        break;
      }
    }
    if (done) {
      break;
    }
    // place read points onto grid map
    std::vector<polar_t> points = lidar.readPoints();
    //std::vector<polar_t> points = test_readPoints();
    printf("Got lidar data\n");
    for (polar_t pt : points) {
      float x = pt.radius * cos(pt.theta) + (float)x0;
      float y = pt.radius * sin(pt.theta) + (float)y0;
      gridmap_set(&map, x, y, 255);
    }
    // see if we can update the minimap
    /*struct timeval currtime;
    gettimeofday(&currtime, NULL);
    if ((currtime.tv_usec - lasttime.tv_usec) * 1000000 +
        (currtime.tv_sec - lasttime.tv_sec) < 100000) { // 10 fps
      continue;
    }
    memcpy(&lasttime, &currtime, sizeof(struct timeval));*/
    // grab and display the minimap
    gridmap_query(&map, x0, y0, M_PI, framebuffer, WINDOW_SIZE, 1);
    SDL_FillRect(screen, NULL, SDL_MapRGB(screen->format, 0, 0, 0));
    for (int i = 0; i < WINDOW_SIZE; i++) {
      for (int j = 0; j < WINDOW_SIZE; j++) {
        uint8_t colorvalue = framebuffer[i * WINDOW_SIZE + j];
        uint32_t color = SDL_MapRGB(screen->format, 0, colorvalue, colorvalue);
        ((uint32_t *)screen->pixels)[i * WINDOW_SIZE + j] = color;
      }
    }
    ((uint32_t *)screen->pixels)[WINDOW_SIZE / 2 * (WINDOW_SIZE + 1)] =
      SDL_MapRGB(screen->format, 255, 255, 255);
    SDL_UpdateTexture(texture, NULL, screen->pixels, screen->pitch);
    // clear the renderer in the GPU
    SDL_RenderClear(renderer);
    // draw the texture
    SDL_RenderCopy(renderer, texture, NULL, NULL);
    // update the screen
    SDL_RenderPresent(renderer);
    SDL_Delay(100);
  }

  // store the map
  //printf("Storing the data\n");
  //map.store(argv[1]);

  printf("Clearing everything up\n");
  SDL_DestroyTexture(texture);
  SDL_FreeSurface(screen);
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
  gridmap_destroy(&map);
  return 0;
}
