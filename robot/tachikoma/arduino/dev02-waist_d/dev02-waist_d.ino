#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include <string.h>

#define DEV_ID 2

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motors[4];
static int v[2];
static int instr_activate;
static bool leg_theta_act;
static bool leg_vel_act;

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

void setmotors(int topv, int btmv) {
  bool topisneg = topv < 0;
  bool btmisneg = btmv < 0;
  topv = limit(abs(topv), 0, 128);
  btmv = limit(abs(btmv), 0, 128);
  motors[0]->setSpeed(topv);
  motors[1]->setSpeed(topv);
  motors[2]->setSpeed(btmv);
  motors[3]->setSpeed(btmv);
  if (topv == 0) {
    motors[0]->run(RELEASE);
    motors[1]->run(RELEASE);
  } else if (topisneg) {
    motors[0]->run(FORWARD);
    motors[1]->run(FORWARD);
  } else {
    motors[0]->run(BACKWARD);
    motors[1]->run(BACKWARD);
  }
  if (btmv == 0) {
    motors[3]->run(RELEASE);
    motors[2]->run(RELEASE);
  } else if (btmisneg) {
    motors[3]->run(FORWARD);
    motors[2]->run(FORWARD);
  } else {
    motors[3]->run(BACKWARD);
    motors[2]->run(BACKWARD);
  }
}

void setup() {
  motors[0] = AFMS.getMotor(1);
  motors[1] = AFMS.getMotor(2);
  motors[2] = AFMS.getMotor(3);
  motors[3] = AFMS.getMotor(4);

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);

  pinMode(13, OUTPUT); // set status LED to OUTPUT and HIGH
  digitalWrite(13, HIGH);

  AFMS.begin();
  setmotors(0, 0);
  Serial.begin(57600);
  msecs = millis();
  timeout = millis();
}

static int targetv[2];
static int prevv[2];
static int targetp[2];

void loop() {
  int nbytes = 0;
  if ((nbytes = Serial.available())) {
    timeout = millis();
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
        sscanf(s, "[%d %d %d %d %d]\n",
          &instr_activate,
          &targetp[0],
          &targetp[1],
          &targetv[0],
          &targetv[1]);
        leg_theta_act = instr_activate & 0x01;
        leg_vel_act = (instr_activate & 0x02) >> 1;
        timeout = millis();
      }
      memmove(buf, &e[1], strlen(&e[1]) + sizeof(char));
    }
  }

  // EMERGENCY STOP: MASTER COMM LOST
  if (millis() - timeout > 500) {
    // after .5 seconds, stop the robot
    setmotors(0, 0);
    prevv[0] = 0;
    prevv[1] = 0;
  }

  if (leg_vel_act) {
    // do nothing, this will override all the later statements
  } else if (leg_theta_act) {
    targetv[0] = (targetp[0] - analogRead(A0)) * 2;
    targetv[1] = (targetp[1] - analogRead(A1)) * 2;
  }

  int deltav[2] = { limit(targetv[0] - prevv[0], -4, 4),
                    limit(targetv[1] - prevv[1], -4, 4) };
  v[0] = limit(prevv[0] + deltav[0], -255, 255);
  v[1] = limit(prevv[1] + deltav[1], -255, 255);
  setmotors(v[0], v[1]);
  prevv[0] = v[0];
  prevv[1] = v[1];

  if (millis() - msecs > 100) {
    sprintf(wbuf, "[%d %d %d %d %d]\n",
      DEV_ID,
      analogRead(A0),
      analogRead(A1),
      v[0],
      v[1]);
    Serial.print(wbuf);
    msecs = millis();
  }
}
