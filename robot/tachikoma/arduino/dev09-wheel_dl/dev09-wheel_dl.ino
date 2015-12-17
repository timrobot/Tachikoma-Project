#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include <string.h>

#define DEV_ID 9

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motors[4];
static int v;

class QuadEncoder {
  public:
    long long pos;
    bool reversed; // set
    char pin[2];
    QuadEncoder() {
      reset();
    }
    int attach(int pin1, int pin2) {
      pinMode(pin1, INPUT);
      pinMode(pin2, INPUT);
      pin[0] = pin1;
      pin[1] = pin2;
      pin_state[0] = digitalRead(pin[0]) == HIGH;
      pin_state[1] = digitalRead(pin[1]) == HIGH;
    }
    int read() {
      update();
      return pos;
    }
    void reset() {
      pin[0] = 0;
      pin[1] = 0;
      pos = 0;
      velocity = 1; // velocity can either be 1 or -1
      reversed = false;
      pin_state[0] = 0;
      pin_state[1] = 0;
    }
  private:
    void update() {
      if (pin[0] == 0 || pin[1] == 0)
        return;
      // FSA : reg :: 00 01 11 10
      //     : rev :: 00 10 11 01
      char new_state[2] = {
        digitalRead(pin[0]) == HIGH,
        digitalRead(pin[1]) == HIGH
      };
      char delta_state[2] = {
        new_state[0] != pin_state[0],
        new_state[1] != pin_state[1]
      };
      if (delta_state[0] && delta_state[1]) {
        pos += velocity * 2 * (reversed ? -1 : 1);
      } else if (delta_state[1]) {
        velocity = (new_state[0] == new_state[1]) ? -1 : 1;
        pos += velocity * (reversed ? -1 : 1);
      } else if (delta_state[0]) {
        velocity = (new_state[0] == new_state[1]) ? 1 : -1;
        pos += velocity * (reversed ? -1 : 1);
      }
      pin_state[0] = new_state[0];
      pin_state[1] = new_state[1];
    }
    char pin_state[2];
    long long velocity;  // estimated
} enc;

const int bufsize = 256;
const int safesize = bufsize / 2;
char buf[bufsize];
char msg[bufsize];
char wbuf[safesize];
unsigned long msecs;
unsigned long timeout;
char numbuf[4];

int limit(int x, int a, int b) {
  if (x > b) {
    return b;
  } else if (x < a) {
    return a;
  } else {
    return x;
  }
}

void setmotors(int v) {
  bool isneg = v < 0;
  v = limit(abs(v), 0, 255);
  motors[0]->setSpeed(v);
  motors[1]->setSpeed(v);
  motors[2]->setSpeed(v);
  motors[3]->setSpeed(v);
  if (v == 0) {
    motors[0]->run(RELEASE);
    motors[1]->run(RELEASE);
    motors[2]->run(RELEASE);
    motors[3]->run(RELEASE);
  } else if (isneg) {
    motors[0]->run(BACKWARD);
    motors[1]->run(BACKWARD);
    motors[2]->run(BACKWARD);
    motors[3]->run(BACKWARD);
  } else {
    motors[0]->run(FORWARD);
    motors[1]->run(FORWARD);
    motors[2]->run(FORWARD);
    motors[3]->run(FORWARD);
  }
}

void setup() {
  motors[0] = AFMS.getMotor(1);
  motors[1] = AFMS.getMotor(2);
  motors[2] = AFMS.getMotor(3);
  motors[3] = AFMS.getMotor(4);

  enc.attach(4, 5);

  pinMode(13, OUTPUT); // set status LED to OUTPUT and HIGH
  digitalWrite(13, HIGH);

  AFMS.begin();
  setmotors(0);
  Serial.begin(57600);
  msecs = millis();
  timeout = millis();
}

static int targetv;
static int prevv;

void loop() {
  int nbytes = 0;
  if ((nbytes = Serial.available())) {
    // read + attach null byte
    int obytes = strlen(buf);
    Serial.readBytes(&buf[obytes], nbytes);
    buf[nbytes + obytes] = '\0';

    // resize just in case
    if (strlen(buf) > safesize) {
      memmove(buf, &buf[strlen(buf) - safesize], safesize);
      buf[safesize] = '\0'; // just in case
    }

    // extract possible message
    char *s, *e;
    if ((e = strchr(buf, '\n'))) {
      e[0] = '\0';
      if ((s = strrchr(buf, '['))) {
        // CUSTOMIZE
        sscanf(s, "[%d]\n",
          &targetv);
        timeout = millis();
      }
      memmove(buf, &e[1], strlen(&e[1]) + sizeof(char));
    }
  }

  // EMERGENCY STOP: MASTER COMM LOST
  if (millis() - timeout > 500) {
    // after .5 seconds, stop the robot
    setmotors(0);
    prevv = 0;
  }

  int deltav = limit(targetv - prevv, -4, 4);
  v = limit(prevv + deltav, -255, 255);
  setmotors(v);
  prevv = v;
  enc.read();

  if (millis() - msecs > 100) {
    sprintf(wbuf, "[%d %d %d]\n",
      DEV_ID,
      enc.read(),
      v);
    Serial.print(wbuf);
    msecs = millis();
  }
}
