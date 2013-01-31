/*
 * Si570.h
 *
 * Copyright (c) 2012, Gabriel Fournier <gabriel@gaftech.fr>
 *
 * This file is part of ArduinoSi570 project.
 * Please read attached LICENSE file. 
 *
 *  Created on: 28 août 2012
 *      Author: Gabriel Fournier
 */

#include <Arduino.h>

#ifndef SI570_H_
#define SI570_H_

//#define SI570_DEBUG

/****************
 * Some constants
 */

// Return codes
#define SI570_SUCCESS			0
#define SI570_ERROR				1
#define SI570_I2C_ERROR			1 << 1
#define SI570_CHECKREG_ERROR	1 << 2
#define SI570_VALUE_ERROR		1 << 3
#define SI570_TIMEOUT_ERROR		1 << 4

// Control bits
#define SI570_R135_DEFAULT		0
#define SI570_R135_RESET		(1 << 7)
#define SI570_R135_NEWFREQ		(1 << 6)
#define SI570_R135_FREEZE_M		(1 << 4)
#define SI570_R135_RECALL		1
#define SI570_R137_FREEZE_DCO	(1 << 4)

// Precalculated values
#define POW_2_16              	65536
#define POW_2_22				4194304
#define POW_2_24           		16777216
#define POW_2_28				268435456

/*******************
 * Hardware settings
 */

// Default Si570 values
#ifndef SI570_I2C_ADDRESS
#define SI570_I2C_ADDRESS		0x55
#endif
#ifndef SI570_STARTUP_FREQ
#define SI570_STARTUP_FREQ		10000000	// Hz
#endif
#ifndef SI570_7PPM_STABILITY
#define SI570_7PPM_STABILITY	0			// 1 for Si570 with "A" second option code
#endif
#ifndef SI570_OE_PIN
#define SI570_OE_PIN			NOT_A_PIN
#endif
#ifndef SI570_OE_HIGH
#define SI570_OE_HIGH			1
#endif
#define	SI570_FDCO_MIN_KHZ 		4850000		// kHz
#define	SI570_FDCO_MAX_KHZ 		5670000		// kHz
#define SI570_SMALL_CHANGE_PPM	3500

/******************************
 * Behavior flags and constants
 */

// Debug flag
//#define SI570_DEBUG							// Debug, using serial

// Tuning check and retry
#define SI570_CHECK_REGISTERS	1				// After writing new frequency,
												// check if it has been applied
												// then retry if not.

#ifndef SI570_R135_RETRIES						// Number of retries when waiting for the register 135
#define SI570_R135_RETRIES		10				// to be cleared
#endif
#ifndef SI570_TUNE_RETRIES						// Number of retries in case of register check error
#define SI570_TUNE_RETRIES		0				// after applying new frequency
#endif
#ifndef SI570_SMALL_CHANGE						// Try the small frequency change algorithm when
#define SI570_SMALL_CHANGE		0				// it can be apply.
#endif

class Si570
{
public:
	byte init();
	byte init(byte addr, unsigned long startupFreq, bool stability_7ppm, byte oepin=NOT_A_PIN, byte oehigh=1);
	byte reset();
	byte hardReset();

	/* Register cache access (no I2C reading) */
	unsigned long getFrequency(void);

	/* Device registers writing (those methods update cache AND device) */
	byte tune(unsigned long);

	/*************************
	 * Output Enable accessors
	 */
	boolean isEnabled();
	void enable(boolean on=1);

	/************
	 * Debugging
	 */

	/*
	 * Compare device values with local write cache
	 *
	 * Can be used to check if previous frequency write was successful.
	 */
	byte checkFrequencyRegisters();
#ifdef SI570_DEBUG
	void debugReadRegisters();
	void debugWriteRegisters();
	void debugRegisters(const byte * regs);
#endif

private:
	/* Device factory settings */
	byte i2cAddress;
	unsigned long startFreq;
	byte oepin;
	boolean oehigh;
	unsigned long fxtal;
	byte regAddr;					// First register address (7 or 13)

	/* I2C buffer */
	byte i2cReadBuf[6];
	byte i2cWriteBuf[6];

	/* Value calculations from registers */
	byte getHSDiv(const byte * regs);
	unsigned int getN1(const byte * regs);
	unsigned int getRfreqInt(const byte * regs);
	unsigned long getRfreqFrac(const byte * regs);
	double getRfreq(const byte * regs);
	unsigned long getFrequency(const byte * regs);
	unsigned long getFdco(const byte * regs);

	/* Write register cache setters */
	byte setRfreqSmallChange(unsigned long foutNew, byte * regs);
	byte setRfreq(unsigned long foutNew, byte * regs);
	byte findDividers(unsigned long foutNew, byte &hsdiv, unsigned int &n1);
	byte setRfreqInts(double rfreq, byte * regs);
	byte setHSDiv(byte hsdiv, byte * regs);
	byte setN1(unsigned int n1, byte * regs);
	byte setRfreqRegisters(unsigned int rfreqInt, unsigned long rfreqFrac, byte * regs);

	/* Frequency setting (cache AND device register update) */
	byte setFrequency(unsigned long);

	/* device 135 and 137 registers accessors */
	byte freezeDCO(void);
	byte unfreezeDCO(void);
	byte waitR135(void);
	byte writeAndWaitR135(byte val)		{ return writeRegister(135, val) | waitR135(); }

	/*
	 * Directly read/write value from/to device register.
	 *
	 * Doesn't update cache, so only use those methods for regs 135 and 137.
	 */
	byte readRegister(byte byteAddress);
	byte writeRegister(byte byteAddress, byte value);

	/*
	 * Device frequency registers read/write from/to cache
	 */
	byte writeFrequencyRegisters();
	byte readFrequencyRegisters();



};

#endif /* SI570_H_ */
