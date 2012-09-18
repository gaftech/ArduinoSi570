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
	return init(SI570_I2C_ADDRESS, SI570_STARTUP_FREQ, SI570_7PPM_STABILITY);
}

byte Si570::init(byte addr, unsigned long startupFreq, bool stability_7ppm) {
	i2cAddress = addr;
	startFreq = startupFreq;
	if (stability_7ppm)
		regAddr = 13;
	else
		regAddr = 7;

	Wire.begin();

	return reset();
}

byte Si570::hardReset() {
	byte r135;

	r135 = SI570_R135_RESET;
	writeRegister(135, r135); // returns error, probably because this command breaks i2c, regarding to datasheet

	do {
		r135 = readRegister(135);
	} while (r135 != SI570_R135_DEFAULT);

	return reset();
}
byte Si570::reset() {
	byte i;
	byte err = 0;
	byte r135;

	r135 = SI570_R135_RECALL;
	err |= writeRegister(135, r135);

	do {
		r135 = readRegister(135);
	} while (r135 != SI570_R135_DEFAULT);


	err |= readFrequencyRegisters();
	// Initially write regs must not be empty because default calculations are based on it
	for (i = 0 ; i < 6 ; i++)
		i2cWriteBuf[i] = i2cReadBuf[i];


	// fxtal calculation
	// First divide, to avoid 32 bits overflow
	fxtal = ((double) startFreq / getRfreq(i2cReadBuf)) * getHSDiv(i2cReadBuf) * getN1(i2cReadBuf);

	return err;
}

unsigned long Si570::getFrequency(void) {
	return getFrequency(i2cWriteBuf);
}

unsigned long Si570::getFrequency(const byte * regs) {
	return (getRfreq(regs) * fxtal) / (getHSDiv(regs) * getN1(regs));
}

//unsigned long Si570::getFxtal(void) {
//	return fxtal;
//}

//byte Si570::getHSDiv() {
//	return getHSDiv(i2cWriteBuf);
//}

byte Si570::getHSDiv(const byte * regs) {
	return ((regs[0] & 0xE0) >> 5) + 4;
}

//unsigned int Si570::getN1() {
//	return getN1(i2cWriteBuf);
//}

unsigned int Si570::getN1(const byte * regs) {
	unsigned int n1;
	n1 = (( regs[0] & 0x1F ) << 2 ) + (( regs[1] & 0xC0 ) >> 6 );
	if (n1 == 0) {
		n1 = 1;
	} else if (n1 & 1 != 0) {
		// add one to an odd number
		n1 = n1 + 1;
	}
	return n1;
}

//unsigned int Si570::getRfreqInt() {
//	return getRfreqInt(i2cWriteBuf);
//}

unsigned int Si570::getRfreqInt(const byte * regs) {
	return (( regs[1] & 0x3F ) << 4 ) + (( regs[2] & 0xF0 ) >> 4 );
}

//unsigned long Si570::getRfreqFrac() {
//	return getRfreqFrac(i2cWriteBuf);
//}

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
	unsigned char hsdiv;
	unsigned int n1;
	byte i;

	err = 0;

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
		for (i = 0 ; i < 6 ; ++i) {
			i2cWriteBuf[i] = i2cReadBuf[i];
		}
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
		Serial.println("Si570: Small Freq change");
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
	Serial.println("Si570: Large Freq change");
#endif

	err |= findDividers(fout, hsdiv, n1);
	if ( err != SI570_SUCCESS) return err;

	err |= setN1(n1, regs);
	err |= setHSDiv(hsdiv, regs);
	err |= setRfreqInts( ( (double) fout / (double) fxtal ) * hsdiv * n1, regs );
	return err;
}

byte Si570::findDividers(unsigned long fout, byte &hsdiv, unsigned int &n1) {
	const unsigned char HS_DIV[6] = {11, 9, 7, 6, 5, 4};
	unsigned long fout_kHz = fout / 1000;
	int maxDivider = floor( (float) SI570_FDCO_MAX_KHZ / fout_kHz );

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
	byte r135;

	r137 = readRegister(137) & ~SI570_R137_FREEZE_DCO;
	r135 = SI570_R135_NEWFREQ;

	err |= writeRegister(137, r137);
	err |= writeRegister(135, r135);

	do {
		r135 = readRegister(135);
	} while (r135 != SI570_R135_DEFAULT);

//	// We don't use writeRegisters() as it seems to be bug-prone,
//	// maybe because of time constraints between unfreeze and
//	// NewFreq bit operations.
//	// TODO: investigate !!!
//	Wire.beginTransmission(i2cAddress);
//	Wire.write(137);
//	Wire.write(r137);
//	err |= Wire.endTransmission();
//	Wire.beginTransmission(i2cAddress);
//	Wire.write(135);
//	Wire.write(r135);
//	err |= Wire.endTransmission();

//	t = micros() -t;
//	Serial.print("delay (us) = ");
//	Serial.println(t);

//	while ( ! (err || i2cBuf[0] == SI570_R135_DEFAULT) ) {
//		err = readRegisters(135, 1);
//	}

	return err;
}

//byte Si570::setHSDiv(byte hsdiv) {
//	return setHSDiv(hsdiv, i2cWriteBuf);
//}

byte Si570::setHSDiv(byte hsdiv, byte * regs) {
	regs[0] &= 0x1f;
	regs[0] |= ((hsdiv - 4) << 5) & 0xe0;
	return SI570_SUCCESS;
}

//byte Si570::setN1(unsigned int n1) {
//	return setN1(n1, i2cWriteBuf);
//}

byte Si570::setN1(unsigned int n1, byte * regs) {
	n1 -= 1;
	regs[0] &= 0xe0;
	regs[0] |= (n1 >> 2) & 0x1f;
	regs[1] &= 0x3f;
	regs[1] |= (n1 << 6) & 0xc0;
	return SI570_SUCCESS;
}

//byte Si570::setRfreqRegisters(unsigned int rfreqInt, unsigned long rfreqFrac) {
//	return setRfreqRegisters(rfreqInt, rfreqFrac, i2cWriteBuf);
//}

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
//	byte n1_b = n1 - 1;

//	i2cWriteBuf[0] = (hsdiv -4) << 5 | n1_b >> 2;
//	i2cWriteBuf[1] = (n1_b & 0x3) << 6;
//	i2cWriteBuf[1] |= rfreq_i >> 4;
//	i2cWriteBuf[2] = (rfreq_i & 0xf) << 4;
//	i2cWriteBuf[2] |= rfreq_f >> 24;
//	i2cWriteBuf[3] = (rfreq_f >> 16) & 0xff;
//	i2cWriteBuf[4] = (rfreq_f >> 8) & 0xff;
//	i2cWriteBuf[5] = rfreq_f & 0xff;

#ifdef SI570_DEBUG
	for (int i = 0 ; i < 6 ; ++i) {
		Serial.print("Si570: WRITE: r");
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

//#ifdef SI570_DEBUG
//	Serial.print("Si570: WRITE: r");
//	Serial.print(byteAddress);
//	Serial.print(": ");
//	Serial.print("0x");
//	Serial.print(value, HEX);
//	if (retcode != 0) {
//		Serial.print(" [WIRE ERROR=");
//		Serial.print(retcode);
//		Serial.println("]");
//	}
//	else Serial.println();
//#endif

	if (retcode != 0) return SI570_I2C_ERROR;

	return SI570_SUCCESS;
}



byte Si570::readRegister(byte byteAddress) {
	byte resp;
	byte retcode;

	Wire.beginTransmission(i2cAddress);
	Wire.write(byteAddress);
	retcode = Wire.endTransmission(false);
	Wire.requestFrom((byte) i2cAddress, (byte) 1);
	resp = Wire.read();
	retcode = Wire.endTransmission();

#ifdef SI570_DEBUG
	Serial.print("Si570: READ: r");
	Serial.print(byteAddress);
	Serial.print(": ");
	Serial.print("0x");
	Serial.print(resp, HEX);
	if (retcode != 0) {
		Serial.print(" [WIRE ERROR=");
		Serial.print(retcode);
		Serial.println("]");
	}
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
		Serial.print("Si570: READ: r");
		Serial.print(regAddr + i);
		Serial.print(": ");
		Serial.print("0x");
		Serial.println(i2cReadBuf[i], HEX);
#endif

//		Serial.print("Read: r");
//		Serial.print(byteAddress + i);
//		Serial.print(": 0x");
//		Serial.println(i2cWriteBuf[i], HEX);
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
			Serial.println("Si570: ERROR: Register error");
			Serial.print(i + regAddr);
			Serial.print(": ");
			Serial.print(i2cReadBuf[i], HEX);
			Serial.print(" != ");
			Serial.println(i2cWriteBuf[i], HEX);

			Serial.println("Si570: ERROR: Register error");
			debugWriteRegisters();
			debugReadRegisters();
#endif
			return SI570_CHECKREG_ERROR;
		}
	}
	return SI570_SUCCESS;
}

void Si570::debugReadRegisters() {
	Serial.println("Si570: Read register values:");
	debugRegisters(i2cReadBuf);
}

void Si570::debugWriteRegisters() {
	Serial.println("Si570: Write register values:");
	debugRegisters(i2cWriteBuf);
}

void Si570::debugRegisters(const byte * regs) {
#ifdef SI570_DEBUG
	Serial.print("\tFxtal      ");
	Serial.print(fxtal);
	Serial.println(" Hz");

	Serial.print("\tHS_DIV     ");
	Serial.println((int) getHSDiv(regs));

	Serial.print("\tN1         ");
	Serial.println((int) getN1(regs));

	Serial.print("\tRfreq      ");
	Serial.print(getRfreq(regs));
	Serial.print(" (");
	Serial.print(getRfreqInt(regs));
	Serial.print(" + ");
	Serial.print(getRfreqFrac(regs));
	Serial.println(" / 2^28)");

	Serial.print("\tFdco       ");
	Serial.print(getFdco(regs));
	Serial.println(" kHz");

	Serial.print("\tFout       ");
	Serial.print(getFrequency(regs));
	Serial.println(" Hz");
	Serial.println();
#endif
}

