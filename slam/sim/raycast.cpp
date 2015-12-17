#include <SDL2/SDL.h>
#include <vector>
#include <armadillo>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <sys/types.h>
#include <unistd.h>
#include <iostream>

double cos_rule_angle(double A, double B, double C) {
  if (fabs((A + B) - C) < 0.000001) {
    return M_PI;
  }
  if (!(A + B >= C)) {
    fprintf(stderr, "Assertion Error: A + B >= C\n");
    exit(1);
  }
  return acos((A * A + B * B - C * C) / (2.0 * A * B));
}

bool concave(const arma::vec &l, const arma::vec &c, const arma::vec &r, double lrad, double crad, double rrad) {
  double dlcx = c(0) - l(0);
  double dlcy = c(1) - l(1);
  double drcx = c(0) - r(0);
  double drcy = c(1) - r(1);
  double lc = sqrt(dlcx * dlcx + dlcy * dlcy);
  double rc = sqrt(drcx * drcx + drcy * drcy);
  double theta1 = cos_rule_angle(lc, crad, lrad);
  double theta2 = cos_rule_angle(rc, crad, rrad);
  return ((theta1 + theta2) - M_PI) > -0.1;
}

bool within(const arma::vec &l, const arma::vec &c, const arma::vec &r, double lrad, double crad, double rrad) {
  if (crad == 0.0) {
    return true;
  }
  return concave(l, c, r, lrad, crad, rrad);
}

std::vector<arma::vec> get_filled_boundary(const std::vector<arma::vec> &pts) {
  arma::vec X(pts.size());
  arma::vec Y(pts.size());
  for (int i = 0; i < pts.size(); i++) {
    X(i) = pts[i](0) * cos(pts[i](1));
    Y(i) = pts[i](0) * sin(pts[i](1));
  }
  int min_x = 0;
  int min_y = 0;
  int max_x = 0;
  int max_y = 0;
  int tx;
  int ty;
  for (int i = 0; i < pts.size(); i++) {
    tx = (int)floor(X(i));
    if (tx < min_x) {
      min_x = tx;
    }
    tx = (int)ceil(X(i));
    if (tx > max_x) {
      max_x = tx;
    }
    ty = (int)floor(Y(i));
    if (ty < min_y) {
      min_y = ty;
    }
    ty = (int)ceil(Y(i));
    if (ty > max_y) {
      max_y = ty;
    }
  }
  max_x++;
  max_y++;
  // create vmap
  arma::mat vmap = arma::zeros<arma::mat>(max_y - min_y, max_x - min_x);
  std::vector<arma::vec> colored;
  std::vector<arma::vec> infection_queue = {{ 0.0, 0.0 }};
  vmap(-min_y, -min_x) = 1.0;
  int index = 0;
  while (infection_queue.size() > index) {
    double x = infection_queue[index](0);
    double y = infection_queue[index](1);

    double theta = atan2(y, x);
    if (theta < 0.0) {
      theta += 2.0 * M_PI;
    }
    double radius = sqrt(x * x + y * y);
    // binary search
    int left = 0;
    int right = pts.size() - 1;
    if (pts[right](1) <= theta || theta < pts[left](1)) {
      left = right;
      right = 0;
    } else {
      int mid = (left + right) / 2;
      while (left + 1 != right) {
        if (theta >= pts[mid](1)) {
          left = mid;
        } else {
          right = mid;
        }
        mid = (left + right) / 2;
      }
    }

    // bfs
    arma::vec l({X(left), Y(left)});
    arma::vec c({x, y});
    arma::vec r({X(right), Y(right)});
    int dx = x - min_x;
    int dy = y - min_y;
    if (within(l, c, r, pts[left](0), radius, pts[right](0))) {
      colored.push_back(c);
      if (y + 1 < max_y && vmap(dy + 1, dx) == 0.0) {
        infection_queue.push_back({x, y + 1});
        vmap(dy + 1, dx) = 1.0;
      }
      if (y - 1 >= min_y && vmap(dy - 1, dx) == 0.0) {
        infection_queue.push_back({x, y - 1});
        vmap(dy - 1, dx) = 1.0;
      }
      if (x + 1 < max_x && vmap(dy, dx + 1) == 0.0) {
        infection_queue.push_back({x + 1, y});
        vmap(dy, dx + 1) = 1.0;
      }
      if (x - 1 >= min_x && vmap(dy, dx - 1) == 0.0) {
        infection_queue.push_back({x - 1, y});
        vmap(dy, dx - 1) = 1.0;
      }
    }
    //infection_queue.erase(infection_queue.begin());
    index++;
  }
  return colored;
}

std::vector<arma::vec> lidar_read(void) {
  std::vector<arma::vec> pts;
  double theta = 0.0;
  while (theta < 360.0) {
    theta += (double)rand() / (double)RAND_MAX + 2.0;
    if (theta > 360.0) {
      break;
    }
    double radius = (double)rand() / (double)RAND_MAX * 100.0 + 100.0;
    pts.push_back(arma::vec({radius, theta * M_PI / 180.0}));
  }
  return pts;
}

int main(int argc, char *argv[]) {
  arma::vec center = { 300.0, 300.0 };

  srand(getpid());
  SDL_Init(SDL_INIT_VIDEO);
  SDL_Window *window = SDL_CreateWindow("raycast", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 640, 640, SDL_WINDOW_SHOWN);
  SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
  SDL_Surface *screen = SDL_CreateRGBSurface(0, 640, 640, 32, 0, 0, 0, 0);
  SDL_Texture *texture = SDL_CreateTextureFromSurface(renderer, screen);

  bool done = false;
  uint32_t color_black = SDL_MapRGB(screen->format, 0, 0, 0);
  uint32_t color_red = SDL_MapRGB(screen->format, 255, 0, 0);
  uint32_t color_green = SDL_MapRGB(screen->format, 0, 255, 0);
  uint32_t color_blue = SDL_MapRGB(screen->format, 0, 0, 255);
  while (true) {
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
    
    std::vector<arma::vec> data = lidar_read();
    printf("getting raycast area...\n");
    std::vector<arma::vec> raycast_area = get_filled_boundary(data);

    SDL_FillRect(screen, NULL, color_black);
    for (arma::vec &pt : raycast_area) {
      int x = (int)(pt(0) + center(0));
      int y = (int)(-pt(1) + center(1));
      ((uint32_t *)screen->pixels)[y * 640 + x] = color_red;
    }
    for (arma::vec &pt : data) {
      int x = (int)(pt(0) * cos(pt(1)) + center(0));
      int y = (int)(-pt(0) * sin(pt(1)) + center(1));
      ((uint32_t *)screen->pixels)[y * 640 + x] = color_green;
    }
    ((uint32_t *)screen->pixels)[300 * 640 + 299] = color_blue;
    ((uint32_t *)screen->pixels)[300 * 640 + 300] = color_blue;
    ((uint32_t *)screen->pixels)[300 * 640 + 301] = color_blue;
    ((uint32_t *)screen->pixels)[299 * 640 + 300] = color_blue;
    ((uint32_t *)screen->pixels)[301 * 640 + 300] = color_blue;

    SDL_UpdateTexture(texture, NULL, screen->pixels, screen->pitch);
    SDL_RenderClear(renderer);
    SDL_RenderCopy(renderer, texture, NULL, NULL);
    SDL_RenderPresent(renderer);
    SDL_Delay(25);
  }
  SDL_DestroyTexture(texture);
  SDL_FreeSurface(screen);
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return 0;
}
