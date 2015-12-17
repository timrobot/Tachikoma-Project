#ifndef __QUADENCODER_H__
#define __QUADENCODER_H__

#include "Arduino.h"

class QuadEncoder {
  public:
    long long pos;
    bool reversed; // set
    char pin[2];
    QuadEncoder(void);
    int attach(int pin1, int pin2);
    long long read(void);
    void reset(void);
    void reverse(bool rev);
  private:
    void update();
    char pin_state[2];
    long long velocity;  // estimated
};

#endif
