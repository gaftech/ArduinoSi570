/*
 * Si570.cpp
 *
 * Copyright (c) 2012, Gabriel Fournier <gabriel@gaftech.fr>
 *
 * This file is part of ArduinoSi570 project.
 * Please read attached LICENSE file. 
 *
 *  Created on: 28 août 2012
 *      Author: Gabriel Fournier
 */

#include "Si570.h"
#include <Arduino.h>
#include <Wire.h>

byte Si570::init() {
	Serial.print("Si570: OE pin at init: ");
	Serial.println(SI570_OE_PIN);
	return init(SI570_I2C_ADDRESS, SI570_STARTUP_FREQ, SI570_7PPM_STABILITY, SI570_OE_PIN, SI570_OE_HIGH);
}

byte Si570::init(byte addr, unsigned long startupFreq, bool stability_7ppm, byte oepin, boolean oehigh) {
	this->i2cAddress = addr;
	this->startFreq = startupFreq;
	if (stability_7ppm)
		regAddr = 13;
	else
		regAddr = 7;
	this->oepin = oepin;
	this->oehigh = oehigh;

	Wire.begin();

	if (oepin) {
		pinMode(oepin,INPUT);
	}

	return reset();
}

byte Si570::hardReset() {
	writeAndWaitR135(SI570_R135_RESET); // returns error, probably because this command breaks i2c, regarding to datasheet
	return reset();
}

byte Si570::reset() {
	byte err = 0;

	err |= writeAndWaitR135(SI570_R135_RECALL);
	err |= readFrequencyRegisters();
	// Initially write regs must not be empty because default calculations are based on it
	memcpy(i2cWriteBuf, i2cReadBuf, 6);

	// fxtal calculation
	// First divide, to avoid 32 bits overflow
	fxtal = ((double) startFreq / getRfreq(i2cReadBuf)) * getHSDiv(i2cReadBuf) * getN1(i2cReadBuf);

	return err;
}

void Si570::enable(boolean on) {
	if (oepin == NOT_A_PIN) return;
	boolean pinLevel;
	pinLevel = oehigh ? on : !on;
	if (pinLevel) {
		pinMode(oepin, INPUT);
	}
	else {
		digitalWrite(oepin, LOW);
		pinMode(oepin, OUTPUT);
		digitalWrite(oepin, LOW);
	}
}

boolean Si570::isEnabled() {
	if (oepin == NOT_A_PIN) return 1;
	boolean pinLevel = digitalRead(oepin);
	return oehigh ? pinLevel : !pinLevel;
}

unsigned long Si570::getFrequency(void) {
	return getFrequency(i2cWriteBuf);
}

unsigned long Si570::getFrequency(const byte * regs) {
	return (getRfreq(regs) * fxtal) / (getHSDiv(regs) * getN1(regs));
}


byte Si570::getHSDiv(const byte * regs) {
	return ((regs[0] & 0xE0) >> 5) + 4;
}

unsigned int Si570::getN1(const byte * regs) {
	unsigned int n1;
	n1 = (( regs[0] & 0x1F ) << 2 ) + (( regs[1] & 0xC0 ) >> 6 );
	if (n1 == 0) {
		n1 = 1;
	} else if ((n1 & 1) != 0) {
		// add one to an odd number
		n1 = n1 + 1;
	}
	return n1;
}

unsigned int Si570::getRfreqInt(const byte * regs) {
	return (( regs[1] & 0x3F ) << 4 ) + (( regs[2] & 0xF0 ) >> 4 );
}

unsigned long Si570::getRfreqFrac(const byte * regs) {
	unsigned long rfreq_f;
	rfreq_f = (unsigned long) regs[5];
	rfreq_f |= (unsigned long) regs[4] << 8;
	rfreq_f |= (unsigned long) regs[3] << 16;
	rfreq_f |= (unsigned long) (regs[2] & 0xf) << 24;
	return rfreq_f;
}

double Si570::getRfreq(const byte * regs) {
	return (double) getRfreqFrac(regs) / POW_2_28 + getRfreqInt(regs) ;
}

unsigned long Si570::getFdco(const byte * regs) {
	return (fxtal / 1000) * getRfreq(regs);
}

byte Si570::tune(unsigned long frequency) {
	byte retries;
	byte err;

	err = SI570_SUCCESS;

	// TODO: unresolved bug: for some given frequencies, tuning operation fails
	// and device registers are reset to startup values. That's why we may need a second try.
	for (retries = SI570_TUNE_RETRIES + 1 ; retries > 0 ; --retries) {
		err |= setFrequency(frequency);
		if (!err) return SI570_SUCCESS;
	}

	return err;
}

byte Si570::setFrequency(unsigned long fout) {
	byte err;

	err = 0;

#ifdef SI570_DEBUG
	Serial.print(F("Si570: set frequency (Hz): "));
	Serial.println(fout);
#endif

	// Try small change
	if (SI570_SMALL_CHANGE && setRfreqSmallChange(fout, i2cWriteBuf) == SI570_SUCCESS) {
		err |= writeRegister(135, SI570_R135_FREEZE_M);
		err |= writeFrequencyRegisters();
		err |= writeRegister(135, SI570_R135_DEFAULT);
	}
	else {
		err |= setRfreq(fout, i2cWriteBuf);
		err |= freezeDCO();
		err |= writeFrequencyRegisters();
		err |= unfreezeDCO();
	}

#if SI570_CHECK_REGISTERS == 1
	if (checkFrequencyRegisters() != SI570_SUCCESS) {
		memcpy(i2cReadBuf, i2cWriteBuf, 6);
		err |= SI570_CHECKREG_ERROR;
	}
#endif

	return err;
}

byte Si570::setRfreqSmallChange(unsigned long foutNew, byte * regs) {
	double rfreqNew;
	unsigned long fdcoKHz;
	double foutCurrent;
	float diff;
	byte hsdiv;
	unsigned int n1;

#ifdef SI570_DEBUG
		Serial.println(F("Si570: Small Freq change"));
#endif

	foutCurrent = getFrequency(regs);
	diff = (float) 1000000 * ((float)abs( (double) foutNew - foutCurrent) / foutCurrent);
	if (diff >= SI570_SMALL_CHANGE_PPM)
		return SI570_VALUE_ERROR;

	hsdiv = getHSDiv(regs);
	n1 = getN1(regs);

	fdcoKHz = (foutNew / 1000) * hsdiv * n1;
	if (fdcoKHz < SI570_FDCO_MIN_KHZ || fdcoKHz > SI570_FDCO_MAX_KHZ)
		return SI570_VALUE_ERROR;

	rfreqNew = (double) fdcoKHz / ((double) fxtal / 1000);

	return setRfreqInts(rfreqNew, regs);
}

byte Si570::setRfreq(unsigned long fout, byte * regs) {
	byte err;
	byte hsdiv;
	unsigned int n1;

	err = SI570_SUCCESS;

#ifdef SI570_DEBUG
	Serial.println(F("Si570: Large Freq change"));
#endif

	err |= findDividers(fout, hsdiv, n1);
	if ( err != SI570_SUCCESS) return err;

	err |= setN1(n1, regs);
	err |= setHSDiv(hsdiv, regs);
	err |= setRfreqInts( ( (double) fout / (double) fxtal ) * hsdiv * n1, regs );
	return err;
}

byte Si570::findDividers(unsigned long fout, byte &hsdiv, unsigned int &n1) {
	const unsigned char HS_DIV[] = {11, 9, 7, 6, 5, 4};
	unsigned long fout_kHz = fout / 1000;
	unsigned int maxDivider = floor( (float) SI570_FDCO_MAX_KHZ / fout_kHz );

	n1 = ceil( (float) SI570_FDCO_MIN_KHZ / (float) fout_kHz / 11);

	if (n1 < 1 || n1 > 128) return SI570_VALUE_ERROR;

	while (n1 <= 128) {
		if (n1 % 2 == 0 || n1 == 1) {
			for (int i = 0; i < 6 ; ++i) {
				hsdiv = HS_DIV[i];
				if (hsdiv * n1 <= maxDivider) {
					return SI570_SUCCESS;
				}
			}
		}
		n1++;
	}
	return SI570_VALUE_ERROR;
}


byte Si570::setRfreqInts(double rfreq, byte * regs) {
	unsigned int rfreqInt;
	unsigned long rfreqFrac;
	rfreqInt = floor(rfreq);
	rfreqFrac = ( rfreq - rfreqInt ) * POW_2_28;
	return setRfreqRegisters(rfreqInt, rfreqFrac, regs);
}

byte Si570::freezeDCO(void) {
	return writeRegister(137, readRegister(137) | SI570_R137_FREEZE_DCO);
}

byte Si570::unfreezeDCO(void) {
	byte err = 0;
	byte r137;

	r137 = readRegister(137) & ~SI570_R137_FREEZE_DCO;
	err |= writeRegister(137, r137);

	err |= writeAndWaitR135(SI570_R135_NEWFREQ);

	return err;
}

byte Si570::waitR135() {
	byte r135;
	for (int i = 0 ; i <= SI570_R135_RETRIES ; ++i) {
		r135 = readRegister(135);
		if (r135 == SI570_R135_DEFAULT)
			break;
	}
	if (r135 == 0xFF)
		return SI570_I2C_ERROR;
	else if (r135 != SI570_R135_DEFAULT)
		return SI570_TIMEOUT_ERROR;
	return SI570_SUCCESS;
}

byte Si570::setHSDiv(byte hsdiv, byte * regs) {
	regs[0] &= 0x1f;
	regs[0] |= ((hsdiv - 4) << 5) & 0xe0;
	return SI570_SUCCESS;
}

byte Si570::setN1(unsigned int n1, byte * regs) {
	n1 -= 1;
	regs[0] &= 0xe0;
	regs[0] |= (n1 >> 2) & 0x1f;
	regs[1] &= 0x3f;
	regs[1] |= (n1 << 6) & 0xc0;
	return SI570_SUCCESS;
}

byte Si570::setRfreqRegisters(unsigned int rfreqInt, unsigned long rfreqFrac, byte * regs) {
	regs[1] &= 0xc0;
	regs[1] |= (rfreqInt >> 4) & 0x3f;
	regs[2] = (rfreqInt << 4) & 0xf0;
	regs[2] |= (rfreqFrac >> 24) & 0x0f;
	regs[3] = (rfreqFrac >> 16) & 0xff;
	regs[4] = (rfreqFrac >> 8) & 0xff;
	regs[5] = rfreqFrac & 0xff;
	return SI570_SUCCESS;
}

byte Si570::writeFrequencyRegisters() {
#ifdef SI570_DEBUG
	for (int i = 0 ; i < 6 ; ++i) {
		Serial.print(F("Si570: WRITE: r"));
		Serial.print(regAddr + i);
		Serial.print(": ");
		Serial.print("0x");
		Serial.println(i2cWriteBuf[i], HEX);
	}
#endif

	Wire.beginTransmission(i2cAddress);
	Wire.write(regAddr);
	for (byte i = 0; i < 6 ; ++i) {
		Wire.write(i2cWriteBuf[i]);
	}
	if (Wire.endTransmission() != 0) return SI570_I2C_ERROR;

	return SI570_SUCCESS;
}

byte Si570::writeRegister(byte byteAddress, byte value) {

	// IMPORTANT: This method must run fast
	// because used in the "Unfreeze DCO + Assert new freq" sequence
	// So don't debug here !

	byte retcode;

	Wire.beginTransmission(i2cAddress);
	Wire.write(byteAddress);
	Wire.write(value);
	retcode = Wire.endTransmission();

	if (retcode != 0) return SI570_I2C_ERROR;

	return SI570_SUCCESS;
}


/*
 * TODO: Handle Wire.endTransmission return value
 */
byte Si570::readRegister(byte byteAddress) {
	byte resp;

	Wire.beginTransmission(i2cAddress);
	Wire.write(byteAddress);
	//NOTE: Needs arduino libs >= 1.0.1 (use of sendStop option)
	Wire.endTransmission(false);
	Wire.requestFrom((byte) i2cAddress, (byte) 1);
	resp = Wire.read();
	Wire.endTransmission();

#ifdef SI570_DEBUG
	Serial.print(F("Si570: READ: r"));
	Serial.print(byteAddress);
	Serial.print(": ");
	Serial.print("0x");
	Serial.print(resp, HEX);
//	if (retcode != 0) {
//		Serial.print(F(" [WIRE ERROR="));
//		Serial.print(retcode);
//		Serial.println("]");
//	}
	else Serial.println();
#endif

	return resp;
}

byte Si570::readFrequencyRegisters() {

	/*
	 * Transmission
	 */

	Wire.beginTransmission(i2cAddress);
	Wire.write(regAddr);
	if (Wire.endTransmission(false) != 0) return SI570_I2C_ERROR;
	Wire.requestFrom( (byte) i2cAddress, (byte) 6);
	for (byte i = 0; i< 6; ++i) {
		if (! Wire.available()) {
			Wire.endTransmission();
			return SI570_I2C_ERROR;
		}
		i2cReadBuf[i] = Wire.read();
#ifdef SI570_DEBUG
		Serial.print(F("Si570: READ: r"));
		Serial.print(regAddr + i);
		Serial.print(": ");
		Serial.print("0x");
		Serial.println(i2cReadBuf[i], HEX);
#endif
	}
	if ( Wire.endTransmission() != 0 )
		return SI570_I2C_ERROR;





	return SI570_SUCCESS;
}

byte Si570::checkFrequencyRegisters() {
	byte i;

	readFrequencyRegisters();

	for (i = 0 ; i < 5 ; ++i) {
		if (i2cReadBuf[i] != i2cWriteBuf[i]) {
#ifdef SI570_DEBUG
			Serial.println(F("Si570: ERROR: Register error"));
			Serial.print(i + regAddr);
			Serial.print(": ");
			Serial.print(i2cReadBuf[i], HEX);
			Serial.print(" != ");
			Serial.println(i2cWriteBuf[i], HEX);

			Serial.println(F("Si570: ERROR: Register error"));
			debugWriteRegisters();
			debugReadRegisters();
#endif
			return SI570_CHECKREG_ERROR;
		}
	}
	return SI570_SUCCESS;
}

#ifdef SI570_DEBUG
void Si570::debugReadRegisters() {

	Serial.println(F("Si570: Read register values:"));
	debugRegisters(i2cReadBuf);
}

void Si570::debugWriteRegisters() {
	Serial.println(F("Si570: Write register values:"));
	debugRegisters(i2cWriteBuf);
}

void Si570::debugRegisters(const byte * regs) {

	Serial.print(F("\tFxtal      "));
	Serial.print(fxtal);
	Serial.println(F(" Hz"));

	Serial.print(F("\tHS_DIV     "));
	Serial.println((int) getHSDiv(regs));

	Serial.print(F("\tN1         "));
	Serial.println((int) getN1(regs));

	Serial.print(F("\tRfreq      "));
	Serial.print(getRfreq(regs));
	Serial.print(F(" ("));
	Serial.print(getRfreqInt(regs));
	Serial.print(F(" + "));
	Serial.print(getRfreqFrac(regs));
	Serial.println(F(" / 2^28)"));

	Serial.print(F("\tFdco       "));
	Serial.print(getFdco(regs));
	Serial.println(F(" kHz"));

	Serial.print(F("\tFout       "));
	Serial.print(getFrequency(regs));
	Serial.println(F(" Hz"));
	Serial.println();

}
#endif
