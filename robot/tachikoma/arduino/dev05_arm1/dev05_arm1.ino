#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>
#include <string.h>

#define DEV_ID 1001

#define POTPIN1	 A0
#define POTPIN2	 A1
#define POTPIN3	 A2
#define POTPIN4	 A3

Adafruit_MotorShield AFMS_base_pivot1(0x60);
Adafruit_MotorShield AFMS_base_pivot2(0x61);
Adafruit_MotorShield AFMS_base_pivot3(0x62);
Adafruit_DCMotor *motors[12];
Servo claw;
const int shoulder = 0;
const int elbow = 1;
const int wrist = 2;
const int grab = 3;

// PID constants
const double Kp[4] = { 3.0, 3.0, 2.5, 2.5 };
const double Ki[4] = { 0, 0, 0, 0 };
const double Kd[4] = { 0, 0, 0, 0 };

static int instr_activate;
static bool arm_theta_act;
static bool arm_vel_act;
static int pos[4];
static int vel[4];
static int pvel[4];

const int bufsize = 256;
const int safesize = bufsize / 2;
static char buf[bufsize];
static char msg[bufsize];
static char wbuf[safesize];
unsigned long msecs;
unsigned long timeout;
unsigned long piddiff;
static char numbuf[8];

static double total_err[4];
static double prev_err[4];

int limit(int x, int a, int b) {
	if (x > b) {
		return b;
	} else if (x < a) {
		return a;
	} else {
		return x;
	}
}

void setmotors(int vv[]) {
	int v[4];
	int Ktolerance[4] = { 20, 15, 15, 10 };
	for (int i = 0; i < 4; i++) {
		v[i] = vv[i];
		if (abs(v[i]) < 10) {
			v[i] = 0;
		} else {
			v[i] += v[i] < 0 ? -Ktolerance[i] : Ktolerance[i];
		}
	}
	bool isneg[4];
	// this only applies to the motors on the shields
	int rev[12] = { 1, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0 }; // TODO

	isneg[shoulder] = v[shoulder] < 0;
	v[shoulder] = limit(abs(v[shoulder]), 0, 255);
	isneg[elbow] = v[elbow] < 0;
	v[elbow] = limit(abs(v[elbow]), 0, 255);
	isneg[wrist] = v[wrist] < 0;
	v[wrist] = limit(abs(v[wrist]), 0, 255);
	isneg[grab] = false;
	v[grab] = limit(v[grab] * 90 / 255 + 90, 0, 180);

	claw.write(v[grab]);

	int speeds[8] = {																		// TODO: assign these
		v[shoulder], v[shoulder], v[shoulder], v[shoulder],
		v[elbow], v[elbow], v[wrist], v[wrist],
		v[grab], 0, 0, 0 };
	int isneg2[8] = {																		// TODO
		isneg[shoulder], isneg[shoulder], isneg[shoulder], isneg[shoulder],
		isneg[elbow], isneg[elbow], isneg[wrist], isneg[wrist],
		v[grab], 0, 0, 0 };

	for (int i = 0; i < 8; i++) {
		motors[i]->setSpeed(speeds[i]);
		if (speeds[i] == 0) {
			motors[i]->run(RELEASE);
		} else {
			bool neg = (isneg2[i] && !rev[i]) || (!isneg2[i] && rev[i]);
			motors[i]->run(neg ? BACKWARD : FORWARD);
		}
	}
}

void setup() {

	Serial.begin(57600);

	// set up the motors
	claw.attach(10);
	for (int i = 0; i < 4; i++) {
		motors[i] = AFMS_base_pivot1.getMotor(i + 1);
		motors[4 + i] = AFMS_base_pivot2.getMotor(i + 1);
	}

	// set up the sensors
	pinMode(POTPIN1, INPUT); // grab
	pinMode(POTPIN2, INPUT); // wrist
	pinMode(POTPIN3, INPUT); // elbow
	pinMode(POTPIN4, INPUT); // shoulder

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
				sscanf(s, "[%d %d %d %d %d %d %d %d %d]\n",
						&instr_activate,
						&pos[shoulder],
						&pos[elbow],
						&pos[wrist],
						&pos[grab],
						&vel[shoulder],
						&vel[elbow],
						&vel[wrist],
						&vel[grab]);
				// limit the positional encoders
				pos[twist] = limit(pos[shoulder], 100, 930);
				pos[elbow] = limit(pos[elbow], 310, 1000);
				pos[wrist] = limit(pos[wrist], 100, 894);
				pos[grab] = limit(pos[grab], 600, 1000);
						
				arm_theta_act = instr_activate & 0x01;
				arm_vel_act = (instr_activate & 0x02) >> 1;
				timeout = millis();
			}
			memmove(buf, &e[1], strlen(&e[1]) + sizeof(char));
		}
	}

	// EMERGENCY STOP: MASTER COMM LOST (for testing turn this off)
	if (millis() - timeout > 500) {
		// after .5 seconds, stop the robot
		memset(pvel, 0, sizeof(int) * 4);
		memset(vel, 0, sizeof(int) * 4);
		setmotors(vel);
		arm_theta_act = false;
		arm_vel_act = false;
		// safety sets
		piddiff = millis();
	}

	if (arm_vel_act) {
		piddiff = millis();
	} else if (arm_theta_act) {
		double err;
		double delta_err;
		double dt = (double)(millis() - piddiff) / 1000.0; // in seconds
		if (dt > 0) {
			piddiff = millis();

			err = (double)(pos[shoulder] - analogRead(POTPIN4));
			total_err[shoulder] += err * dt;
			delta_err = (err - prev_err[shoulder]) / dt;
			vel[shoulder] = err * Kp[shoulder] + total_err[shoulder] * Ki[shoulder] + delta_err * Kd[shoulder];
			prev_err[shoulder] = err;

			err = (double)(pos[elbow] - analogRead(POTPIN3));
			total_err[elbow] += err * dt;
			delta_err = (err - prev_err[elbow]) / dt;
			vel[elbow] = err * Kp[elbow] + total_err[elbow] * Ki[elbow] + delta_err * Kd[elbow];
			prev_err[elbow] = err;

			err = (double)(pos[wrist] - analogRead(POTPIN2));
			total_err[wrist] += err * dt;
			delta_err = (err - prev_err[wrist]) / dt;
			vel[wrist] = err * Kp[wrist] + total_err[wrist] * Ki[wrist] + delta_err * Kd[wrist];
			prev_err[wrist] = err;

			err = (double)(pos[grab] - analogRead(POTPIN1));
			total_err[grab] += err * dt;
			delta_err = (err - prev_err[grab]) / dt;
			vel[grab] = err * Kp[grab] + total_err[grab] * Ki[grab] + delta_err * Kd[grab];
			prev_err[grab] = err;
		}
	} else {
		memset(pvel, 0, sizeof(int) * 4);
		memset(vel, 0, sizeof(int) * 4);
		setmotors(vel);
		piddiff = millis();
	}

	// ramp function for the signal output
	int deltav[4] = { limit(vel[shoulder] - pvel[shoulder], -8, 8),
										limit(vel[elbow] - pvel[elbow], -8, 8),
										limit(vel[wrist] - pvel[wrist], -8, 8),
										limit(vel[grab] - pvel[grab], -255, 255) };
	int v[4];
	v[shoulder] = limit(pvel[shoulder] + deltav[shoulder], -172, 172);
	v[elbow] = limit(pvel[elbow] + deltav[elbow], -172, 172);
	v[wrist] = limit(pvel[wrist] + deltav[wrist], -172, 172);
	v[grab] = limit(pvel[grab] + deltav[grab], -255, 255);
	
	// push the values to the motors
	setmotors(v);
	
	pvel[shoulder] = v[shoulder];
	pvel[elbow] = v[elbow];
	pvel[wrist] = v[wrist];
	pvel[grab] = v[grab];

	if (millis() - msecs > 50) {
		sprintf(wbuf, "[%d %d %d %d %d %d %d %d %d]\n",
				DEV_ID,
				analogRead(POTPIN4),
				analogRead(POTPIN3),
				analogRead(POTPIN2),
				analogRead(POTPIN1),
				pvel[shoulder],
				pvel[elbow],
				pvel[wrist],
				pvel[grab]);
		Serial.print(wbuf);
		msecs = millis();
	}
}
