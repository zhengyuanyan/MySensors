/*
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2022 Sensnology AB
 * Full contributor list: https://github.com/mysensors/MySensors/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

#include "MyLeds.h"

// #define MY_DEFAULT_LED_PCA9685
#if defined(MY_DEFAULT_LED_PCA9685)
#include <Wire.h>
// #include "../drivers/Adafruit_PWM_Servo_Driver_Library/Adafruit_PWMServoDriver.h"

#define LED_COMMON_CATHODE 0
#define LED_COMMON_ANODE 1

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);
#endif

#define LED_ON_OFF_RATIO        (4)       // Power of 2 please
#define LED_PROCESS_INTERVAL_MS (MY_DEFAULT_LED_BLINK_PERIOD/LED_ON_OFF_RATIO)

// these variables don't need to be volatile, since we are not using interrupts
static uint8_t countRx;
static uint8_t countTx;
static uint8_t countErr;
static unsigned long prevTime;

inline void ledsWrite(uint8_t pin, bool state, bool uesePwm = false, uint8_t pwmch = 0)
{
#if defined(MY_DEFAULT_LED_PCA9685)
	if (uesePwm) {
		if (LED_POLARITY == LED_COMMON_CATHODE)
		{
			pca.setPWM(pwmch, state ? 4096 : 0,  0);
		}else {
			pca.setPWM(pwmch, 4096, state ? 0 : 4096);
		}
		return;
	}	
#else
	hwDigitalWrite(pin, state);
#endif
}

inline void ledsInit()
{
	// initialize counters
	countRx = 0;
	countTx = 0;
	countErr = 0;

#if defined(MY_DEFAULT_LED_PCA9685)

    Wire.begin(PCA9685_I2C_SDA_PIN, PCA9685_I2C_SCL_PIN, PCA9685_I2C_FREQUENCY);
	pca.begin();
	pca.setPWMFreq(PCA9685_PWM_FREQUENCY);  // This is the maximum PWM frequency
	delay(10);
#if defined (MY_DEFAULT_POWER_PCA9685_PIN)
{
	// ledsWrite(0, 1, true, MY_DEFAULT_POWER_PCA9685_PIN);
	pca.setPWM(0, 4095,  0);
}
	for (int i = 1; i < 16; i++)
	{
		ledsWrite(0, 0, true, i);
		delay(200);
		ledsWrite(0, 1, true, i);
	}
#endif

#else
	// Setup led pins
#if defined(MY_DEFAULT_RX_LED_PIN)
	hwPinMode(MY_DEFAULT_RX_LED_PIN,  OUTPUT);
#endif
#if defined(MY_DEFAULT_TX_LED_PIN)
	hwPinMode(MY_DEFAULT_TX_LED_PIN,  OUTPUT);
#endif
#if defined(MY_DEFAULT_ERR_LED_PIN)
	hwPinMode(MY_DEFAULT_ERR_LED_PIN, OUTPUT);
#endif
#endif
	prevTime = hwMillis() -
	           LED_PROCESS_INTERVAL_MS;     // Subtract some, to make sure leds gets updated on first run.
	ledsProcess();

}

void ledsProcess()
{
	// Just return if it is not the time...
	if ((hwMillis() - prevTime) < LED_PROCESS_INTERVAL_MS) {
		return;
	}
	prevTime = hwMillis();

#if defined(MY_DEFAULT_RX_LED_PIN) || defined(MY_DEFAULT_TX_LED_PIN) || defined(MY_DEFAULT_ERR_LED_PIN)
	uint8_t state;
#endif

	// For an On/Off ratio of 4, the pattern repeated will be [on, on, on, off]
	// until the counter becomes 0.

#if defined(MY_DEFAULT_RX_LED_PIN)
	if (countRx) {
		--countRx;
	}
	state = (countRx & (LED_ON_OFF_RATIO-1)) ? LED_ON : LED_OFF;
#if defined(MY_DEFAULT_LED_PCA9685)
	ledsWrite(0, state, true, MY_DEFAULT_RX_LED_PIN);
#else
	ledsWrite(MY_DEFAULT_RX_LED_PIN, state, false);
#endif
#endif


#if defined(MY_DEFAULT_TX_LED_PIN)
	if (countTx) {
		--countTx;
	}
	state = (countTx & (LED_ON_OFF_RATIO-1)) ? LED_ON : LED_OFF;

#if defined(MY_DEFAULT_LED_PCA9685)
		ledsWrite(0, state, true, MY_DEFAULT_TX_LED_PIN);
#else
		ledsWrite(MY_DEFAULT_TX_LED_PIN, state, false);
#endif
#endif

#if defined(MY_DEFAULT_ERR_LED_PIN)
	if (countErr) {
		--countErr;
	}
	state = (countErr & (LED_ON_OFF_RATIO-1)) ? LED_ON : LED_OFF;

#if defined(MY_DEFAULT_LED_PCA9685)
		ledsWrite(0, state, true, MY_DEFAULT_ERR_LED_PIN);
#else
		ledsWrite(MY_DEFAULT_ERR_LED_PIN, state, false);
#endif
#endif
}

void ledsBlinkRx(uint8_t cnt)
{
	if (!countRx) {
		countRx = cnt*LED_ON_OFF_RATIO;
	}
	ledsProcess();
}

void ledsBlinkTx(uint8_t cnt)
{
	if(!countTx) {
		countTx = cnt*LED_ON_OFF_RATIO;
	}
	ledsProcess();
}

void ledsBlinkErr(uint8_t cnt)
{
	if(!countErr) {
		countErr = cnt*LED_ON_OFF_RATIO;
	}
	ledsProcess();
}

bool ledsBlinking()
{
	return countRx || countTx || countErr;
}
