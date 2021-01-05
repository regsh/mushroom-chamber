// EnvLCD.h

#ifndef _ENVLCD_h
#define _ENVLCD_h

#include "EnvFactor.h"
#include <Adafruit_RGBLCDShield.h> //LCD shield

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif
class EnvLCD{
	private:
		uint8_t count;
		uint8_t state;
		EnvFactor* factors;
		String homeMessage;
		Adafruit_RGBLCDShield lcd;

		void nextState();
		void updateDisplay();
		void displayHome();
	public:
		EnvLCD(EnvFactor* fctrs, uint8_t cnt, String homeMsg);
		void CheckButtons();

};


#endif

