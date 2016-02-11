#include "QuadEncoder.h"

QuadEncoder::QuadEncoder(void) {
  reset();
}

void QuadEncoder::attach(int pin1, int pin2) {
  pinMode(pin1, INPUT);
  pinMode(pin2, INPUT);
  this->pin[0] = pin1;
  this->pin[1] = pin2;
  this->pin_state[0] = digitalRead(this->pin[0]) == HIGH;
  this->pin_state[1] = digitalRead(this->pin[1]) == HIGH;
}

long long QuadEncoder::read(void) {
  this->update();
  return this->pos;
}

void QuadEncoder::reset(void) {
  this->pin[0] = 0;
  this->pin[1] = 0;
  this->pos = 0;
  this->velocity = 1; // velocity can either be 1 or -1
  this->reversed = false;
  this->pin_state[0] = 0;
  this->pin_state[1] = 0;
}

void QuadEncoder::reverse(bool rev) {
  this->reversed = rev;
}

void QuadEncoder::update(void) {
  if (this->pin[0] == 0 || this->pin[1] == 0) {
    return;
  }
  // FSA : reg :: 00 01 11 10
  //     : rev :: 00 10 11 01
  char new_state[2] = {
    digitalRead(this->pin[0]) == HIGH,
    digitalRead(this->pin[1]) == HIGH
  };
  char delta_state[2] = {
    this->new_state[0] != this->pin_state[0],
    this->new_state[1] != this->pin_state[1]
  };
  if (delta_state[0] && delta_state[1]) {
    this->pos += this->velocity * 2 * (this->reversed ? -1 : 1);
  } else if (delta_state[1]) {
    this->velocity = (new_state[0] == new_state[1]) ? -1 : 1;
    this->pos += this->velocity * (this->reversed ? -1 : 1);
  } else if (delta_state[0]) {
    this->velocity = (new_state[0] == new_state[1]) ? 1 : -1;
    this->pos += this->velocity * (this->reversed ? -1 : 1);
  }
  this->pin_state[0] = new_state[0];
  this->pin_state[1] = new_state[1];
}
