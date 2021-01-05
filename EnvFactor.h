// EnvFactor.h

#ifndef _ENVFACTOR_h
#define _ENVFACTOR_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif
class EnvFactor {
private:
	String name;
	int setPtLow;
	int setPtHigh;
	int setInterval;
	int (*func)(void); //function to get current value of factor from sensor
public:
	EnvFactor(String n, int(*current)(), int low, int high, int intvl);
	String GetName();
	int GetLow();
	void SetLow(bool raise);
	int GetHigh();
	void SetHigh(bool raise);
	int GetCurrent();
	bool CheckValue(int v);
};


#endif

