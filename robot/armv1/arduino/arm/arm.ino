#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include <Servo.h>
#include <string.h>
#include <PID_v1.h>

#define DEV_ID 1

Adafruit_MotorShield AFMS_bot(0x60);
Adafruit_MotorShield AFMS_top(0x61);
Adafruit_DCMotor *motors[8];
PID *pospid[6];
Servo claw;
static double pos[6];
static int vel[6];
static double in[6];
static double out[6];
static bool pid_en;
static int move_en;

const int bufsize = 256;
const int safesize = bufsize / 2;
char buf[bufsize], msg[bufsize], wbuf[safesize];
unsigned long msecs, timeout;

int limit(int x, int a, int b) {
  if (x > b) {
    return b;
  } else if (x < a) {
    return a;
  } else {
    return x;
  }
}

void setmotors(int *v) { // 6 numbers
  bool isneg[5];
  for (int i = 0; i < 5; i++) {
    isneg[i] = v[i] < 0;
    v[i] = limit(abs(v[i]), 0, 255);
  }
  int motormap[8] = { 0, 0, 1, 1, 3, 4, 2, 2 };
  for (int i = 0; i < 8; i++) {
    int vid = motormap[i];
    motors[i]->setSpeed(v[vid]);
    if (v[vid] == 0) {
      motors[i]->run(RELEASE);
    } else if (isneg[vid]) {
      motors[i]->run(BACKWARD);
    } else {
      motors[i]->run(FORWARD);
    }
  }
  v[5] = limit(v[5], -90, 90);
  claw.write(v[5] + 90);
}

void setup() {
  
  Serial.begin(57600);
  
  //Serial.println("setting up motors");
  for (int i = 0; i < 4; i++) {
    motors[i] = AFMS_bot.getMotor(i + 1);
    motors[4 + i] = AFMS_top.getMotor(i + 1);
  }
  claw.attach(3);

  //Serial.println("setting up pid");
  for (int i = 0; i < 6; i++) {
    pinMode(A0 + i, INPUT);
    pospid[i] = new PID(&in[i], &out[i], &pos[i], 2.0, 5.0, 1.0, DIRECT);
    pospid[i]->SetOutputLimits(0, 1024); // pot range
    in[i] = analogRead(A0 + i);
  }

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  //Serial.println("doing rest of init");
  AFMS_bot.begin();
  AFMS_top.begin();
  
  //Serial.println("sending motor values");
  setmotors(vel);
  msecs = millis();
  timeout = millis();
  
  //Serial.println("done init");
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
        // CUSTOMIZE (set the setpoint)
        sscanf(s, "[%d %d %d %d %d %d %d]\n",
          &move_en,
          &vel[0],
          &vel[1],
          &vel[2],
          &vel[3],
          &vel[4],
          &vel[5]);
        timeout = millis();
        if (!pid_en && move_en) {
          for (int i = 0; i < 6; i++) {
            pospid[i]->SetMode(AUTOMATIC);
          }
          pid_en = true;
        }
      }
      memmove(buf, &e[1], strlen(&e[1]) + sizeof(char));
    }
  }

  // EMERGENCY STOP: MASTER COMM LOST
  if (millis() - timeout > 500 || !move_en) {
    // after .5 seconds, stop the robot
    if (pid_en) {
      for (int i = 0; i < 6; i++) {
        pospid[i]->SetMode(MANUAL);
      }
      pid_en = false;
    }
    for (int i = 0; i < 6; i++) {
      vel[i] = 0;
    }
  }

  // Update the PID
  for (int i = 0; i < 6; i++) {
    /*in[i] = analogRead(A0 + i);
    pospid[i]->Compute();
    if (pid_en && move_en) {
      vel[i] = out[i] - in[i];
    }*/
  }

  // push the values to the motors
  setmotors(vel);

  if (millis() - msecs > 100) {
    sprintf(wbuf, "[%d %d %d %d %d %d %d]\n",
      DEV_ID,
      analogRead(A0),
      analogRead(A1),
      analogRead(A2),
      analogRead(A3),
      analogRead(A4),
      analogRead(A5));
    Serial.print(wbuf);
    msecs = millis();
  }
}
