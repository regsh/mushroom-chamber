// 
// 
// 

#include "EnvFactor.h"

EnvFactor::EnvFactor(String n, int(*current)(), int low, int high, int intvl)
{
	name = n;
	func = current;
	setPtLow = low;
	setPtHigh = high;
	setInterval = intvl;
}

String EnvFactor::GetName()
{
	return name;
}

int EnvFactor::GetLow()
{
	return setPtLow;
}

void EnvFactor::SetLow(bool raise)
{
	if (raise) setPtLow += setInterval;
	else setPtLow -= setInterval;
}

int EnvFactor::GetHigh()
{
	return setPtHigh;
}

void EnvFactor::SetHigh(bool raise)
{
	if (raise) setPtHigh += raise;
	else setPtHigh -= raise;
}

int EnvFactor::GetCurrent()
{
	return func();
}

bool EnvFactor::CheckValue(int v)
{
	return (v >= setPtLow && v <= setPtHigh);
}


