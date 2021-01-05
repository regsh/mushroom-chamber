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
	char* name;
	int setPtLow;
	int setPtHigh;
public:
	EnvFactor(char* n, int low, int high);
	String GetName();
	int GetLow();
	void SetLow(int l);
	int GetHigh();
	void SetHigh(int h);
	bool CheckValue(int v);
};


#endif

