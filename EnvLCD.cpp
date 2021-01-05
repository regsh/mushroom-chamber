// 
// 
// 

#include "EnvLCD.h"

void EnvLCD::nextState()
{
	if (state < count * 3) state++;
	else state = 0;
}


void EnvLCD::updateDisplay()
{
	if (state == 0) displayHome();
	else {
		lcd.clear();
		uint8_t factorIdx = (state - 1) / 3;
		lcd.print(factors[factorIdx].GetName);
		lcd.setCursor(0, 1);
		if ((state - 1) % 3 == 0) {
			lcd.print("MN: ");
			lcd.print(factors[factorIdx].GetLow);
		}
		else if((state -1)%3 == 1){
			lcd.print("MX: ");
			lcd.print(factors[factorIdx].GetHigh);
		}
		else {
			lcd.print("CURR: ");
			lcd.print(factors[factorIdx].GetCurrent);
		}
	}
}

void EnvLCD::displayHome()
{
	lcd.clear();
	lcd.print(homeMessage);
}

EnvLCD::EnvLCD(EnvFactor* fctrs, uint8_t cnt, String homeMsg)
{
	count = cnt;
	factors = fctrs;
	homeMessage = homeMsg;
	lcd = Adafruit_RGBLCDShield();
}

void EnvLCD::CheckButtons()
{
	uint8_t buttons = lcd.readButtons();
	if (buttons) {
		bool changed = false;
		if (BUTTON_RIGHT) {
			nextState();
			changed = true;
		}
		else if (state != 0) {
			uint8_t factorIdx = (state - 1) / 3;
			uint8_t screen = (state - 1) % 3;
			if (!screen == 2) {
				if (BUTTON_UP) {
					if (screen == 0) {
						factors[factorIdx].SetLow(true);
					}
					else factors[factorIdx].SetHigh(true);
					changed = true;
				}
				else if (BUTTON_DOWN) {
					if (screen == 0) {
						factors[factorIdx].SetLow(false);
					}
					else factors[factorIdx].SetHigh(false);
					changed = true;
				}
			}
		}
		if (changed) {
			updateDisplay();
		}
	}
}
