#include "armv1.h"
#include "defs.h"
#include <iostream>
#include <signal.h>
#include <SDL/SDL.h>
#include <SDL/SDL_image.h>
#include <SDL/SDL_ttf.h>
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

// SDL SHIT
const int SCREEN_WIDTH = 640;
const int SCREEN_HEIGHT = 480;
const int SCREEN_BPP = 32;
static SDL_Surface *message;
static SDL_Surface *screen;
static SDL_Event event;
static TTF_Font *font;
Arm arm;
SDL_Color textColor = { 0xFF, 0xFF, 0xFF }; // white

class ButtonInput {
  public:
  int x;
  int y;
  int w;
  int h;
  uint32_t c;

  bool trigger;
  ButtonInput(int _x, int _y, int _w, int _h, uint32_t _c);
  ~ButtonInput();
  void handle_input();
  bool clicked();
  void show();
};

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

//The key press interpreter
class StringInput
{
    private:
    //The storage string
    std::string str;

    //The text surface
    SDL_Surface *text;

    public:
    //Initializes variables
    StringInput();

    //Does clean up
    ~StringInput();

    //Handles input
    void handle_input();

    //Shows the message on screen
    void show_centered();

    //Clear the message buf
    void clear_input();

    //Get the input string
    string get_input();
};

void stopsignal(int) {
  stopsig = true;
}

double deg2rad(double deg) {
  return deg * M_PI / 180.0;
}

void init() {
  SDL_Init(SDL_INIT_EVERYTHING);
  screen = SDL_SetVideoMode( SCREEN_WIDTH, SCREEN_HEIGHT, SCREEN_BPP, SDL_SWSURFACE );
  TTF_Init();
  SDL_WM_SetCaption("Robot Interface", NULL);

  font = TTF_OpenFont("lazy.ttf", 16);
}

void destroy() {
  SDL_FreeSurface(message);
  TTF_CloseFont(font);
  TTF_Quit();
  SDL_Quit();
}

void apply_surface( int x, int y, SDL_Surface* source, SDL_Surface* destination, SDL_Rect* clip = NULL )
{
    //Holds offsets
    SDL_Rect offset;

    //Get offsets
    offset.x = x;
    offset.y = y;

    //Blit
    SDL_BlitSurface( source, clip, destination, &offset );
}

StringInput::StringInput() {
  str = "";
  text = NULL;
  SDL_EnableUNICODE(SDL_ENABLE);
}

StringInput::~StringInput() {
  SDL_FreeSurface(text);
  text = NULL;
  SDL_EnableUNICODE(SDL_DISABLE);
}

void StringInput::show_centered() {
  if (text) {
    apply_surface((SCREEN_WIDTH - text->w) / 2, (SCREEN_HEIGHT - text->h) / 2, text, screen);
  }
}

void StringInput::handle_input()
{
    //If a key was pressed
    if( event.type == SDL_KEYDOWN )
    {
        //Keep a copy of the current version of the string
        std::string temp = str;

        //If the string less than maximum size
        if( str.length() <= 16 )
        {
            //If the key is a space
            if( event.key.keysym.unicode == (Uint16)' ' )
            {
                //Append the character
                str += (char)event.key.keysym.unicode;
            }
            //If the key is a number
            else if( ( event.key.keysym.unicode >= (Uint16)'0' ) && ( event.key.keysym.unicode <= (Uint16)'9' ) )
            {
                //Append the character
                str += (char)event.key.keysym.unicode;
            }
            //If the key is a uppercase letter
            else if( ( event.key.keysym.unicode >= (Uint16)'A' ) && ( event.key.keysym.unicode <= (Uint16)'Z' ) )
            {
                //Append the character
                str += (char)event.key.keysym.unicode;
            }
            //If the key is a lowercase letter
            else if( ( event.key.keysym.unicode >= (Uint16)'a' ) && ( event.key.keysym.unicode <= (Uint16)'z' ) )
            {
                //Append the character
                str += (char)event.key.keysym.unicode;
            }
        }

        //If backspace was pressed and the string isn't blank
        if( ( event.key.keysym.sym == SDLK_BACKSPACE ) && ( str.length() != 0 ) )
        {
            //Remove a character from the end
            str.erase( str.length() - 1 );
        }

        //If the string was changed
        if( str != temp )
        {
            //Free the old surface
            SDL_FreeSurface( text );

            //Render a new text surface
            text = TTF_RenderText_Solid( font, str.c_str(), textColor );
        }
    }
}

void StringInput::clear_input() {
  str = "";
  SDL_FreeSurface(text);
  text = TTF_RenderText_Solid(font, str.c_str(), textColor);
}

string StringInput::get_input() {
  return str;
}

void start_arm() {
  arm.connect();
  if (!arm.connected()) {
    printf("[ARM TEST] Not connected to anything, disconnecting...\n");
    arm.disconnect();
    exit(1);
  }

  string params;
  ifstream params_file("calib_params.json");
  string temp;
  while (getline(params_file, temp)) {
    params += temp;
  }
  params_file.close();
  arm.set_calibration_params(json::parse(params));
  if (!arm.calibrated()) {
    arm.disconnect();
    exit(1);
  }
}

void stop_arm() {
  arm.disconnect();
}

int main() {
  signal(SIGINT, stopsignal);
  init();
  //ButtonInput send_button((SCREEN_WIDTH-100)/2 + 100, SCREEN_HEIGHT - ((SCREEN_HEIGHT/2)-40)/2, 100, 40, 0x0000FF00);
  ButtonInput stop_button((SCREEN_WIDTH-100)/2, SCREEN_HEIGHT - ((SCREEN_HEIGHT/2)-40)/2, 100, 40, 0x00FF0000);
  StringInput serial_message;
  message = TTF_RenderText_Solid(font, "Enter message:", textColor);

  // connect to the arm 
  //start_arm();

  mat arm_pos(1, DOF, fill::zeros);
  mat arm_vel(1, DOF, fill::zeros);

  // run loop
  bool position_en = false;
  bool velocity_en = false;


  while (!stopsig) {
    while (SDL_PollEvent(&event)) {
      stop_button.handle_input();
      if (stop_button.clicked()) {
        printf("clicked stop button\n");
      }
      serial_message.handle_input();
      if ((event.type == SDL_KEYDOWN) && (event.key.keysym.sym == SDLK_RETURN)) {
        string text = serial_message.get_input();
        printf("typed: %s\n", text.c_str());
        serial_message.clear_input();
      }
      if (event.type == SDL_QUIT) {
        stopsig = true;
      }
    }

    SDL_FillRect(screen, NULL, 0);
    apply_surface((SCREEN_WIDTH - message->w)/2,((SCREEN_HEIGHT/2)-message->h)/2, message, screen);
    serial_message.show_centered();
    stop_button.show();

    // render screen
    SDL_Flip(screen);
    SDL_Delay(25);
  }

  stop_arm();
  destroy();
  return 0;
}
