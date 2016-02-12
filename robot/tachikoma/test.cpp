#include <iostream>
#include <signal.h>
#include <SDL/SDL.h>
#include <cstdint>
#include <iostream>
#include <fstream>
#include "tachikoma.h"
#include <njson/json.hpp>
#include "xboxctrl.h"
#include <sys/time.h>

#define FPS 25
#define KEYID(k) ((k)-'a')

using namespace arma;
using namespace std;
using json = nlohmann::json;
static bool stopsig;

static xboxctrl_t xBoxController;
static pthread_t xBoxControllerThread;

void *updateXboxController(void* args)
{
	while (!stopsig)
	{
		xboxctrl_update(&xBoxController);
	}
	return NULL;
}

void stopsignal(int) {
  stopsig = true;
}

double deg2rad(double deg) {
  return deg * M_PI / 180.0;
}

double remap_axis(double v) {
  if (-0.2 <= v && v <= 0.2) {
    return 0;
  } else {
    return v * abs(v);
  }
}

int main() {
  signal(SIGINT, stopsignal);
  SDL_Init(SDL_INIT_EVERYTHING);
  SDL_Surface *screen;
  uint32_t start;

  // connect to the tachikoma
  screen = SDL_SetVideoMode(640, 480, 32, SDL_SWSURFACE);
  Tachikoma tachikoma;
  printf("trying to connect to robot...\n");
  tachikoma.connect();
  printf("connection successful!\n");
  
  if (!tachikoma.connected()) {
    printf("[TACHI TEST] Not connected to anything, disconnecting...\n");
    tachikoma.disconnect();
    return 1;
  }

  string params;
  ifstream params_file("calib_params.json");
  string temp;
  while (getline(params_file, temp)) {
    params += temp;
  }
  params_file.close();
  tachikoma.set_calibration_params(json::parse(params));

  mat leg_pos(NUM_LEGS, NUM_JOINTS, fill::zeros);
  mat leg_vel(NUM_LEGS, NUM_JOINTS, fill::zeros);
  vec wheels(NUM_LEGS, fill::zeros);


  // connect to xbox controller
  xboxctrl_connect(&xBoxController);
  
  pthread_create(&xBoxControllerThread, NULL, updateXboxController, NULL);

  // run loop
  char key_pressed[26];
  int legid = 0;
  bool position_en = false;
  bool velocity_en = false;
  int standstate = 0;
  int dposestate = 0;
  int xboxStandState = -1;
  int xboxOmni = 0;
  int unlockCode = 0;
  int gait[] = { 0, 0, 0, 0 };
  int tiltstate = 0; // default tilt state is 0

  struct timeval starttime;
  int inGait = 0;
  int xboxInGait = 0;
  int xboxGaitState = 0;
  int maxGaitStates = 17;
  struct timeval xboxstarttime;
  int x_prev = 0;
  int y_prev = 0;
  int LJOY_prev = 0;
  // LOG TO FILE
  FILE *fp;
  fp = fopen("gait_data1.txt", "w");
  int number;
  number = 1;

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
            case SDLK_o: key_pressed[KEYID('o')] = 1; break;
            case SDLK_k: key_pressed[KEYID('k')] = 1; break;
            case SDLK_p: key_pressed[KEYID('p')] = 1; break;
            case SDLK_l: key_pressed[KEYID('l')] = 1; break;
            case SDLK_q: key_pressed[KEYID('q')] = 1; break;
            case SDLK_w: key_pressed[KEYID('w')] = 1; break;
            case SDLK_e: key_pressed[KEYID('e')] = 1; break;
            case SDLK_a: key_pressed[KEYID('a')] = 1; break;
            case SDLK_s: key_pressed[KEYID('s')] = 1; break;
            case SDLK_d: key_pressed[KEYID('d')] = 1; break;
            case SDLK_x: key_pressed[KEYID('x')] = 1; break;
            case SDLK_1: legid = 0; break;
            case SDLK_2: legid = 1; break;
            case SDLK_3: legid = 2; break;
            case SDLK_4: legid = 3; break;
            case SDLK_8: velocity_en = true; position_en = false; break;
            case SDLK_9:
              velocity_en = false;
              position_en = true;
              dposestate = 0;
              standstate = 0;
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
            case SDLK_o: key_pressed[KEYID('o')] = 0; break;
            case SDLK_k: key_pressed[KEYID('k')] = 0; break;
            case SDLK_p: key_pressed[KEYID('p')] = 0; standstate = -1; break;
            case SDLK_l: key_pressed[KEYID('l')] = 0; standstate = 0; break;
            case SDLK_q: key_pressed[KEYID('q')] = 0; break;
            case SDLK_w: key_pressed[KEYID('w')] = 0;
                         if (!inGait) {
                           inGait = 1;
                           gettimeofday(&starttime, NULL);
                         }
                         break;
            case SDLK_e: key_pressed[KEYID('e')] = 0; break;
            case SDLK_a: key_pressed[KEYID('a')] = 0; break;
            case SDLK_s: key_pressed[KEYID('s')] = 0; break;
            case SDLK_d: key_pressed[KEYID('d')] = 0; break;
			      case SDLK_x: key_pressed[KEYID('x')] = 0; break;
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
    int quit = key_pressed[KEYID('x')];


    // CONTROLLER JOINS
//    if (xBoxController.Y) {
//      xboxStandState = 0;
//    }
//    else if (xBoxController.X) {
//      xboxStandState = -1;
//    }

    if (xBoxController.Y) {
      if (y_prev == 0) {
        if (!xboxInGait) {
          xboxGaitState = (xboxGaitState + 1) % maxGaitStates;
          gettimeofday(&xboxstarttime, NULL);
          xboxInGait = 1;
        }
      }
    } else if (xBoxController.X) {
      if (x_prev == 0) {
        if (!xboxInGait) {
          xboxGaitState = (xboxGaitState - 1) % maxGaitStates;
          gettimeofday(&xboxstarttime, NULL);
          xboxInGait = 1;
        }
      }
    }

    
    y_prev = xBoxController.Y;
    x_prev = xBoxController.X;

    if (xBoxController.LB) {
      xboxOmni = 1;
    } else if (xBoxController.RB) {
      xboxOmni = 0;
    }

    if (xBoxController.SELECT) {
      velocity_en = false;
      position_en = false;
    } else if (xBoxController.START) {
      velocity_en = false;
      position_en = false;
    } else if (xBoxController.HOME) {
      velocity_en = false;
      position_en = false;
      leg_vel.zeros();
      tachikoma.move(leg_pos, leg_vel, wheels, zeros<mat>(1, 1), position_en, velocity_en);
      stopsig = true;
      break; // get out of the loop! will shut down the robot
    }

    if (xBoxController.LT > 0 && xBoxController.RT > 0 && xBoxController.B) {
      position_en = true;
      velocity_en = false;
      // start the initial gait
      xboxGaitState = 0;
      gettimeofday(&xboxstarttime, NULL);
      xboxInGait = 1;
    }

    if (xBoxController.LJOY.pressed > 0 ) {
      if ( LJOY_prev == 0 ) {
        number++;
        string fileCount = to_string(number);
        string name = "gait_data" + fileCount + ".txt"; 
        fclose(fp);
        fp = fopen(name.c_str(),"w");
      }
    }

    LJOY_prev = xBoxController.LJOY.pressed;

    if (velocity_en)
    {
      if (quit != 0)
      {
      	printf("Quitting\n");
      	SDL_Quit();
      }

      printf("velocity en\n");
      leg_vel(legid, WAIST) = (k_u - k_h);
      leg_vel(legid, THIGH) = (k_i - k_j);
      //for (int r = 0; r < 4; r++) {
      //  wheels(r) = (k_p - k_l);
      //}
//    for (int r = 0; r < (int)tachikoma.current_reading.size(); r++) {
//      double I = tachikoma.current_reading[r];
//      fprintf(fp, "%d,\n", (int)I);
//    }



    }

    else if (position_en) {
      printf("position en\n");

      /*double coeff[] = { 1.0, -1.0, -1.0, 1.0 };
      for (int i = 0; i < NUM_LEGS; i++) {
        cout << "dstate: " << coeff[i] << endl;
        leg_pos(i, WAIST) = xboxOmni * M_PI_4 * coeff[i];
        leg_pos(i, THIGH) = xboxStandState * M_PI_4;
        // switch (i) {
        //   case UL:
        //     wheels(i) = k_w - k_a - k_s + k_d - k_q + k_e;
        //     break;
        //   case UR:
        //     wheels(i) = k_w + k_a - k_s - k_d + k_q - k_e;
        //     break;
        //   case DL:
        //     wheels(i) = k_w - k_a - k_s + k_d + k_q - k_e;
        //     break;
        //   case DR:
        //     wheels(i) = k_w + k_a - k_s - k_d - k_q + k_e;
        //     break;
        // }
      }


      printf("standstate: %d\n", standstate);

      double w[4];

      double ly = remap_axis(xBoxController.LJOY.y);
      double lx = remap_axis(xBoxController.LJOY.x);
      double rx = remap_axis(xBoxController.RJOY.x);
      w[0] = (ly + lx + rx); //Front Left
      w[1] = (ly - lx - rx); //Front Left
      w[2] = (ly - lx + rx); //Front Left
      w[3] = (ly + lx - rx); //Front Left

      for (int i=0; i<4; i++)
      {
      	if (abs(w[i]) < 0.2)
      	{
      		w[i] = 0;
      	}
        if (w[i] > 0.5) w[i] = 0.5;
        if (w[i] < -0.5) w[i] = -0.5;
        
        wheels(i) = w[i];
  	  }*/

      if (xboxInGait) {
        struct timeval currtime;
        gettimeofday(&currtime, NULL);
        int dsec = currtime.tv_sec - xboxstarttime.tv_sec;
        int dusec = currtime.tv_usec - xboxstarttime.tv_usec;
        double dt = (double)dsec + (double)dusec / 1000000;
        if ((int)(dt/1) > 0) { // 1 second
          xboxInGait = 0;
        }
        // FSM chooses the state of each leg
        int gs = xboxGaitState + 1;
        switch (gs) {
          // reset the state
          case  1: gait[0] = 0; gait[1] = 0; gait[2] = 0; gait[3] = 0; tiltstate = 0; break;
          // start of the back left leg motion
          case  2: gait[0] = 0; gait[1] = 0; gait[2] = 0; gait[3] = 0; tiltstate = 2; break;
          case  3: gait[0] = 0; gait[1] = 0; gait[2] = 1; gait[3] = 0; tiltstate = 2; break;
          case  4: gait[0] = 0; gait[1] = 0; gait[2] = 2; gait[3] = 0; tiltstate = 2; break;
          // end of the back left leg motion
          case  5: gait[0] = 0; gait[1] = 0; gait[2] = 3; gait[3] = 0; tiltstate = 1; break;
          // start of the back right leg motion
          case  6: gait[0] = 0; gait[1] = 0; gait[2] = 3; gait[3] = 0; tiltstate = 0; break;
          case  7: gait[0] = 0; gait[1] = 0; gait[2] = 3; gait[3] = 1; tiltstate = 0; break;
          case  8: gait[0] = 0; gait[1] = 0; gait[2] = 3; gait[3] = 2; tiltstate = 0; break;
          // end of the back right leg motion
          case  9: gait[0] = 0; gait[1] = 0; gait[2] = 3; gait[3] = 3; tiltstate = 2; break;
          // start of the front left leg motion
          case 10: gait[0] = 0; gait[1] = 0; gait[2] = 3; gait[3] = 3; tiltstate = 3; break;
          case 11: gait[0] = 1; gait[1] = 0; gait[2] = 3; gait[3] = 3; tiltstate = 3; break;
          case 12: gait[0] = 2; gait[1] = 0; gait[2] = 3; gait[3] = 3; tiltstate = 3; break;
          // end of the front left leg motion
          case 13: gait[0] = 3; gait[1] = 0; gait[2] = 3; gait[3] = 3; tiltstate = 3; break;
          // start of the front right leg motion
          case 14: gait[0] = 3; gait[1] = 0; gait[2] = 3; gait[3] = 3; tiltstate = 3; break;
          case 15: gait[0] = 3; gait[1] = 1; gait[2] = 3; gait[3] = 3; tiltstate = 3; break;
          case 16: gait[0] = 3; gait[1] = 2; gait[2] = 3; gait[3] = 3; tiltstate = 3; break;
          // end of the front right leg motion
          case 17: gait[0] = 3; gait[1] = 3; gait[2] = 3; gait[3] = 3; tiltstate = 4; break;
        }
        // map each leg's state to actual positions
        double coeff[] = { -1.0, 1.0, -1.0, 1.0 }; // set these up
        double coeff2[] = { -1.0, 1.0, 1.0, -1.0 }; // set these up
        double variance = M_PI_4/2;
        double bias = M_PI_4/4;
        tiltstate = 0;
        for (int i = 0; i < NUM_LEGS; i++) {
          if (gait[i] == 0) {
            leg_pos(i, THIGH) = bias + variance;
            leg_pos(i, WAIST) = (-coeff[i] / 2 - coeff2[i] / 2 - coeff[i] / 4 * tiltstate) * M_PI_4/2;
          }
          if (gait[i] == 1) {
            leg_pos(i, THIGH) = bias - variance;
            leg_pos(i, WAIST) = (-coeff[i] / 2 - coeff2[i] / 2 - coeff[i] / 4 * tiltstate) * M_PI_4/2;
          }
          if (gait[i] == 2) {
            leg_pos(i, THIGH) = bias - variance;
            leg_pos(i, WAIST) = (coeff[i] / 2 - coeff2[i] / 2 - coeff[i] / 4 * tiltstate) * M_PI_4/2;
          }
          if (gait[i] == 3) {
            leg_pos(i, THIGH) = bias + variance;
            leg_pos(i, WAIST) = (coeff[i] / 2 - coeff2[i] / 2 - coeff[i] / 4 * tiltstate) * M_PI_4/2;
          }
          if (gait[i] == 4) {
            leg_pos(i, THIGH) = bias + variance;
            leg_pos(i, WAIST) = (-coeff[i] / 2 - coeff2[i] / 2 - coeff[i] / 4 * tiltstate) * M_PI_4/2;
          }
          if (gait[i] == 5) {
            leg_pos(i, THIGH) = bias + variance;
            leg_pos(i, WAIST) = (coeff[i] / 2 - coeff2[i] / 2 - coeff[i] / 4 * tiltstate) * M_PI_4/2;
          }
        }
      }

      /*      if (inGait) {
        printf("in gait\n");
        struct timeval currtime;
        gettimeofday(&currtime, NULL); // grab the time to make sure the gait is in the correct frame
        // find the difference in order to get the gait
        int dsec = currtime.tv_sec - starttime.tv_sec;
        int dusec = currtime.tv_usec - starttime.tv_usec;
        double dt = (double)dsec + (double)dusec / 1000000;
        int gait_state = (int)(dt / 1) + 1; // 1 second per gait state transition

        // FSM here
        switch (gait_state) {
          case  1: gait[0] = 0; gait[1] = 0; gait[2] = 0; gait[3] = 0; break;
          case  2: gait[0] = 0; gait[1] = 0; gait[2] = 1; gait[3] = 0; break;
          case  3: gait[0] = 0; gait[1] = 0; gait[2] = 2; gait[3] = 0; break;
          case  4: gait[0] = 0; gait[1] = 0; gait[2] = 3; gait[3] = 0; break;
          case  5: gait[0] = 0; gait[1] = 0; gait[2] = 3; gait[3] = 1; break;
          case  6: gait[0] = 0; gait[1] = 0; gait[2] = 3; gait[3] = 2; break;
          case  7: gait[0] = 0; gait[1] = 0; gait[2] = 3; gait[3] = 3; break;
          case  8: gait[0] = 1; gait[1] = 0; gait[2] = 3; gait[3] = 3; break;
          case  9: gait[0] = 2; gait[1] = 0; gait[2] = 3; gait[3] = 3; break;
          case 10: gait[0] = 3; gait[1] = 0; gait[2] = 3; gait[3] = 3; break;
          case 11: gait[0] = 3; gait[1] = 1; gait[2] = 3; gait[3] = 3; break;
          case 12: gait[0] = 3; gait[1] = 2; gait[2] = 3; gait[3] = 3; break;
          case 13: gait[0] = 3; gait[1] = 3; gait[2] = 3; gait[3] = 3; break;
          case 14: gait[0] = 0; gait[1] = 0; gait[2] = 0; gait[3] = 0; break;
          case 15: inGait = false;
        }


        double coeff[] = { -1.0, 1.0, -1.0, 1.0 }; // set these up
        double coeff2[] = { -1.0, 1.0, 1.0, -1.0 }; // set these up
        for (int i = 0; i < NUM_LEGS; i++) {
          if (gait[i] == 0) {
            leg_pos(i, THIGH) = M_PI_4/2;
            leg_pos(i, WAIST) = (-coeff[i] / 2 - coeff2[i] / 2) * M_PI_4;
          }
          if (gait[i] == 1) {
            leg_pos(i, THIGH) = -M_PI_4/2;
            leg_pos(i, WAIST) = (-coeff[i] / 2 - coeff2[i] / 2) * M_PI_4;
          }
          if (gait[i] == 2) {
            leg_pos(i, THIGH) = -M_PI_4/2;
            leg_pos(i, WAIST) = (coeff[i] / 2 - coeff2[i] / 2) * M_PI_4;
          }
          if (gait[i] == 3) {
            leg_pos(i, THIGH) = M_PI_4/2;
            leg_pos(i, WAIST) = (coeff[i] / 2 - coeff2[i] / 2) * M_PI_4;
          }
        }
      }*/



    }
    tachikoma.move(leg_pos, leg_vel, wheels, zeros<mat>(1, 1), position_en, velocity_en);
    // print out the feedback from the robot
    mat leg_sensors;
    mat leg_feedback;
    tachikoma.sense(leg_sensors, leg_feedback);
    std::cout << leg_sensors.t() << std::endl;
    std::cout << leg_feedback.t() << std::endl;
    for (int r = 0; r < (int)tachikoma.current_reading.n_elem; r++) {
      double I = tachikoma.current_reading(r);
      fprintf(fp, "%d,\n", (int)I);
    }

    // render screen
    SDL_Flip(screen);
    if (1000 / FPS > SDL_GetTicks() - start) {
      SDL_Delay(1000 / FPS - (SDL_GetTicks() - start));
    }
  }

  fclose(fp);
  tachikoma.disconnect();
  sleep(1);
  pthread_join(xBoxControllerThread, NULL);
  xboxctrl_disconnect(&xBoxController);

  SDL_Quit();
  return 0;
}

