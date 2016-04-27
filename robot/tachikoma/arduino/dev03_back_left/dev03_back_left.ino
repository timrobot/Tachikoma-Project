#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include <Encoder.h>
#include <string.h>

#define DEV_ID	3

#define POTPIN1	 A0
#define POTPIN2	 A1
#define CURSENSE1 A2
#define CURSENSE2 A3

Adafruit_MotorShield AFMS_lower_shield(0x61);
Adafruit_MotorShield AFMS_upper_shield(0x60);
Adafruit_DCMotor *motors[8];
Encoder wheelenc(2, 3);
const int waist = 0;
const int thigh = 1;
const int wheel = 2;

// PID constants
const double Kp[2] = { 2.5, 3.0 };
const double Ki[2] = { 0, 0 };
const double Kd[2] = { 0, 0 };

static int instr_activate;
static bool leg_theta_act;
static bool leg_vel_act;
static bool wheel_vel_act;
static int pos[2];
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
static char numbuf[8];

static double total_err[2];
static double prev_err[2];

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
	int v[3];

	// set velocity to 0 if |velocity| < 10, otherwise give an extra boost of +15 signal
	int Ktolerance[2] = { 15, 20 };
	for (int i = 0; i < 2; i++) {
		v[i] = vv[i];
		if (abs(v[i]) < 10) {
			v[i] = 0;
		} else {
			v[i] += v[i] < 0 ? -Ktolerance[i] : Ktolerance[i];
		}
	}
	v[2] = vv[2];

	// set isneg = sign(velocity), set velocity = |velocity|
	bool isneg[3];

	isneg[waist] = v[waist] < 0;
	v[waist] = limit(abs(v[waist]), 0, 255);
	isneg[thigh] = v[thigh] < 0;
	v[thigh] = limit(abs(v[thigh]), 0, 255);
	isneg[wheel] = v[wheel] < 0;
	v[wheel] = limit(abs(v[wheel]), 0, 255);

	// assign velocities to individual motors
	int rev[8] = { 1, 1, 0, 0, 1, 0, 1, 0 };
	int speeds[8] = {
		v[waist], v[waist], v[wheel], v[wheel],
		v[thigh], v[thigh], v[thigh], v[thigh] };
	int isneg2[8] = {
		isneg[waist], isneg[waist], isneg[wheel], isneg[wheel],
		isneg[thigh], isneg[thigh], isneg[thigh], isneg[thigh] };

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
	for (int i = 0; i < 4; i++) {
		motors[i] = AFMS_lower_shield.getMotor(i + 1);
		motors[4 + i] = AFMS_upper_shield.getMotor(i + 1);
	}

	// set up the sensors
	pinMode(POTPIN1, INPUT); // waist
	pinMode(POTPIN2, INPUT); // thigh
	pinMode(CURSENSE1, INPUT); // waist
	pinMode(CURSENSE2, INPUT); // thigh
	wheelenc.write(0);

	// flash led 13
	pinMode(13, OUTPUT);
	digitalWrite(13, HIGH);

	// turn on the motor shields
	AFMS_lower_shield.begin();
	AFMS_upper_shield.begin();

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
				sscanf(s, "[%d %d %d %d %d %d]\n",
						&instr_activate,
						&pos[waist],
						&pos[thigh],
						&vel[waist],
						&vel[thigh],
						&vel[wheel]);
				// limit the positional encoders
				pos[waist] = limit(pos[waist], 310, 1000);
				pos[thigh] = limit(pos[thigh], 100, 894);
						
				leg_theta_act = (instr_activate & 0x04) >> 2;
				leg_vel_act = (instr_activate & 0x08) >> 3;
				wheel_vel_act = (instr_activate & 0x10) >> 4;
				if (!wheel_vel_act) {
					vel[wheel] = 0;
				}
				timeout = millis();
			}
			memmove(buf, &e[1], strlen(&e[1]) + sizeof(char));
		}
	}

	// EMERGENCY STOP: MASTER COMM LOST (for testing turn this off)
	if (millis() - timeout > 500) {
		// after .5 seconds, stop the robot
		memset(pvel, 0, sizeof(int) * 3);
		memset(vel, 0, sizeof(int) * 3);
		setmotors(vel);
		leg_theta_act = false;
		leg_vel_act = false;
		// safety sets
		piddiff = millis();
	}

	if (leg_vel_act) {
		piddiff = millis();
	} else if (leg_theta_act) {
		double err;
		double delta_err;
		double dt = (double)(millis() - piddiff) / 1000.0; // in seconds
		if (dt > 0) {
			piddiff = millis();

			err = (double)(pos[waist] - analogRead(POTPIN1));
			total_err[waist] += err * dt;
			delta_err = (err - prev_err[waist]) / dt;
			vel[waist] = err * Kp[waist] + total_err[waist] * Ki[waist] + delta_err * Kd[waist];
			prev_err[waist] = err;

			err = (double)(pos[thigh] - analogRead(POTPIN2));
			total_err[thigh] += err * dt;
			delta_err = (err - prev_err[thigh]) / dt;
			vel[thigh] = err * Kp[thigh] + total_err[thigh] * Ki[thigh] + delta_err * Kd[thigh];
			prev_err[thigh] = err;
		}
	} else {
		memset(pvel, 0, sizeof(int) * 2);
		memset(vel, 0, sizeof(int) * 2);
		setmotors(vel);
		piddiff = millis();
	}

	// ramp function for the signal output
	int deltav[3] = { limit(vel[waist] - pvel[waist], -4, 4),
										limit(vel[thigh] - pvel[thigh], -4, 4),
										limit(vel[wheel] - pvel[wheel], -8, 8) };
	int v[3];
	v[waist] = limit(pvel[waist] + deltav[waist], -172, 172); // limit the joint speeds
	v[thigh] = limit(pvel[thigh] + deltav[thigh], -172, 172);
	v[wheel] = limit(pvel[wheel] + deltav[wheel], -255, 255);
	
	// push the values to the motors
	setmotors(v);
	
	pvel[waist] = v[waist];
	pvel[thigh] = v[thigh];
	pvel[wheel] = v[wheel];

	// update the quad encoder on the wheel
	long encval = wheelenc.read();

	if (millis() - msecs > 50) {
		sprintf(wbuf, "[%d %d %d %ld %d %d %d]\n",
				DEV_ID,
				analogRead(POTPIN1),
				analogRead(POTPIN2),
				encval,
				pvel[waist],
				pvel[thigh],
				pvel[wheel]);
		Serial.print(wbuf);
		msecs = millis();
	}
}
