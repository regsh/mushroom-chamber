// 
// 
// 

#include "EnvFactor.h"

EnvFactor::EnvFactor(char* n, int low, int high)
{
	name = n;

	setPtLow = low;
	setPtHigh = high;
}

String EnvFactor::GetName()
{
	return *name;
}

int EnvFactor::GetLow()
{
	return setPtLow;
}

void EnvFactor::SetLow(int l)
{
	setPtLow = l;
}

int EnvFactor::GetHigh()
{
	return setPtHigh;
}

void EnvFactor::SetHigh(int h)
{
	setPtHigh = h;
}

bool EnvFactor::CheckValue(int v)
{
	return (v >= setPtLow && v <= setPtHigh);
}
