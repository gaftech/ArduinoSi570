/*
 * Si570_SerialInterface.ino
 *
 * Copyright (c) 2012, Gabriel Fournier <gabriel@gaftech.fr>
 *
 * This file is part of ArduinoSi570 project.
 * Please read attached LICENSE file. 
 *
 *  Created on: 27 août 2012
 *      Author: Gabriel Fournier
 */

/**
 * This sketch provides some basic serial commands to configure the Si570.
 *
 * 	* Frequency setting
 * 		Simply type desired frequency in MHz then press enter
 *
 * 	* Display register values
 * 		Type "d" then press enter
 * 	* Reset/calibrate Si570
 * 		Type "r" (soft)  or "R" then press enter
 *
 */

//#define SI570_DEBUG

#include <Wire.h>
#include <Si570.h>
#include <Arduino.h>

static Si570 device;
static String buf;

void displayInfos() {
#ifdef SI570_DEBUG
	device.debugWriteRegisters();
#else
	Serial.println("debug disabled !");
#endif
}

void tune() {
	double freq;
	char freqString[9];
	byte err;

	buf.toCharArray(freqString, buf.length()+1);
	freq = atof(freqString);

	Serial.print("Tuning: ");
	Serial.print(freq);
	Serial.println(" MHz...");

	err = device.tune(freq*1000000);
	Serial.println("done.");

	displayInfos();

	if (err == SI570_SUCCESS) {
		Serial.println("OK");
	} else {
		Serial.println("ERROR");
	}
}

void checkRegisters() {
	bool ok;

	ok = device.checkFrequencyRegisters() == SI570_SUCCESS;

	Serial.println();
	Serial.print("Device register / local cache check: ");
	if(ok) Serial.println("OK");
	else Serial.print("ERROR");
	Serial.println();
}

void reset(bool hard) {
	byte err;
	Serial.println("Resetting Si570...");
	if (hard)
		err = device.hardReset();
	else
		err = device.reset();
	displayInfos();
	if (err)
		Serial.println("ERROR");
	else
		Serial.println("OK");
}

void handleCmd() {
	Serial.println();
	if (buf == "d") {
		displayInfos();
	} else if (buf == "r") {
		reset(false);
	} else if (buf == "R") {
		reset(true);
	} else if (buf == "c") {
		checkRegisters();
	} else {
		tune();
	}
	buf = "";
}

void loop() {

	if (Serial.available()) {

		char c = Serial.read();

		switch (c) {
		case '\r':
		case '\n':
			handleCmd();
			break;
		default:
			Serial.print(c);
			buf += c;
			break;
		}
	}
}

void setup() {
	Serial.begin(9600);
	buf.reserve(8);

	Serial.println("Device init...");

	while (device.init() != SI570_SUCCESS) {
		Serial.println("ERROR");
		delay(10000);
		Serial.println("Retrying...");
	}
	Serial.println("OK");

	displayInfos();

	Serial.println("Type frequency in MHz and press enter: ");

}
