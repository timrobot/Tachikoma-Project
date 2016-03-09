#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include <Servo.h>
#include <string.h>

#define DEV_ID 1

#define POTPIN1   A0
#define POTPIN2   A1
#define POTPIN3   A2
#define CURSENSE  A3
#define BASE1     A4
#define BASE2     A5

Adafruit_MotorShield AFMS_base_pivot1(0x60);
Adafruit_MotorShield AFMS_base_pivot2(0x61);
Adafruit_DCMotor *motors[8];
Servo base_turn[2]; // only 2, since each wire will go to 2 motors
// the following are the ids
const int turn = 0;
const int pivot1 = 1;
const int pivot2 = 2;
const double Kp = 1.2;
const double Ki = 0.8;
//const double Kd = 0.4; // for now not used

static int instr_activate;
static bool arm_theta_act;
static bool arm_vel_act;
static int pos[3];
static int vel[3];
static int pvel[3];

const int bufsize = 256;
const int safesize = bufsize / 2;
static char buf[bufsize];
static char msg[bufsize];
static char wbuf[safesize];
unsigned long msecs;
unsigned long timeout;
unsigned long piddiff;
static char numbuf[4];

static double total_err[3];

int limit(int x, int a, int b) {
  if (x > b) {
    return b;
  } else if (x < a) {
    return a;
  } else {
    return x;
  }
}

void setmotors(int v[]) { // 6 numbers
  bool isneg[3];
  // this only applies to the motors on the shields
  int rev[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

  isneg[turn] = false;
  v[turn] = limit(v[turn] * 90 / 255 + 90, 0, 180);
  isneg[pivot1] = v[pivot1] < 0;
  v[pivot1] = limit(abs(v[pivot1]), 0, 255);
  isneg[pivot2] = v[pivot2] < 0;
  v[pivot2] = limit(abs(v[pivot2]), 0, 255);

  for (int i = 0; i < 2; i++) {
    base_turn[i].write(v[turn]);
  }
  for (int i = 0; i < 8; i++) {
    int vid = i / 4 + 1; // hack
    motors[i]->setSpeed(v[vid]);
    if (v[vid] == 0) {
      motors[i]->run(RELEASE);
    } else {
      bool neg = (isneg[vid] && !rev[i]) || (!isneg[vid] && rev[i]);
      motors[i]->run(neg ? FORWARD : BACKWARD);
    }
  }
}

void setup() {
  
  Serial.begin(57600);
  
  // set up the motors
  base_turn[0].attach(BASE1);
  base_turn[1].attach(BASE2);
  for (int i = 0; i < 4; i++) {
    motors[i] = AFMS_base_pivot1.getMotor(i + 1);
    motors[4 + i] = AFMS_base_pivot2.getMotor(i + 1);
  }

  // set up the sensors
  pinMode(POTPIN1, INPUT);
  pinMode(POTPIN2, INPUT);
  pinMode(POTPIN3, INPUT);
  pinMode(CURSENSE, INPUT);

  // flash led 13
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  // turn on the motor shields
  AFMS_base_pivot1.begin();
  AFMS_base_pivot2.begin();
 
  setmotors(vel);
  msecs = millis();
  timeout = millis();
  piddiff = millis();
  
}

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
        // CUSTOMIZE (set the setpoint)
        sscanf(s, "[%d %d %d %d %d %d %d]\n",
          &instr_activate,
          &pos[turn],
          &pos[pivot1],
          &pos[pivot2],
          &vel[turn],
          &vel[pivot1],
          &vel[pivot2]);
        arm_theta_act = instr_activate & 0x01;
        arm_vel_act = (instr_activate & 0x02) >> 1;
        timeout = millis();
      }
      memmove(buf, &e[1], strlen(&e[1]) + sizeof(char));
    }
  }

  // EMERGENCY STOP: MASTER COMM LOST
  if (millis() - timeout > 500) {
    // after .5 seconds, stop the robot
    setmotors(0, 0);
    vel[turn] = 0;
    vel[pivot1] = 0;
    vel[pivot2] = 0;
    arm_theta_act = false;
    arm_vel_act = false;
    // safety sets
    piddiff = millis();
  }

  if (arm_vel_act) {
    piddiff = millis();
  } else if (arm_theta_act) {
    double err;
    double dt = (double)(piddiff - millis()) / 1000.0;
    piddiff = millis();
    // determine the error for the turning base
    err = (double)(pos[turn] - analogRead(POTPIN1));
    total_err[turn] += err * dt;
    vel[turn] = err * Kp + total_err[turn] * Ki;
    err = (double)(pos[pivot1] - analogRead(POTPIN2));
    total_err[pivot1] += err * dt;
    vel[pivot1] = err * Kp + total_err[pivot1] * Ki;
    err = (double)(pos[pivot2] - analogRead(POTPIN3));
    total_err[pivot2] += err * dt;
    vel[pivot2] = err * Kp + total_err[pivot2] * Ki;
  }

  int deltav[3] = { limit(vel[turn] - pvel[turn], -4, 4),
                    limit(vel[pivot1] - pvel[pivot1], -4, 4),
                    limit(vel[pivot2] - pvel[pivot2], -4, 4) };
  int v[3];
  v[turn] = limit(pvel[turn] + deltav[turn], -255, 255);
  v[pivot1] = limit(pvel[pivot1] + deltav[pivot1], -255, 255);
  v[pivot2] = limit(pvel[pivot2] + deltav[pivot2], -255, 255);
  setmotors(v);
  pvel[turn] = v[turn];
  pvel[pivot1] = v[pivot1];
  pvel[pivot2] = v[pivot2];

  // push the values to the motors
  setmotors(prevv);

  if (millis() - msecs > 100) {
    sprintf(wbuf, "[%d %d %d %d %d]\n",
      DEV_ID,
      analogRead(A0),
      analogRead(A1),
      analogRead(A2),
      analogRead(A3));
    Serial.print(wbuf);
    msecs = millis();
  }
}
