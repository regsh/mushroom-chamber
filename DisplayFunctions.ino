//Function for displaying the appropriate info on screen of LCD
//based on current state of program
void displayState(int state) { //change back to switch case?
    switch (state) {
    case MAIN:
        lcd.clear();
        lcd.print(("CO2: "));
        lcd.print(co2Current);
        lcd.print(("ppm"));
        lcd.setCursor(0, 1);
        lcd.print(F("Temp:"));
        lcd.print((int)convertCtoF(tempCurrent));
        lcd.print(F("F"));
        lcd.print(F(" RH:"));
        lcd.print(rhCurrent);
        lcd.print("%");
        break;
    case SET_CO2_MAX:
        lcd.clear();
        lcd.print(F("CO2 MAX:"));
        lcd.setCursor(0, 1);
        lcd.print(co2Max * 100);
        break;
    case SET_RH_MIN:
        lcd.clear();
        lcd.print(F("RH MIN:"));
        lcd.setCursor(0, 1);
        lcd.print(rhMin);
        break;
    case SET_RH_MAX:
        lcd.clear();
        lcd.print(F("RH MAX:"));
        lcd.setCursor(0, 1);
        lcd.print(rhMax);
        break;
    case SET_TEMP_MIN:
        lcd.clear();
        lcd.print(F("TEMP MIN:"));
        lcd.setCursor(0, 1);
        lcd.print(convertCtoF(tempMin));
        break;
    case SET_TEMP_MAX:
        lcd.clear();
        lcd.print(F("TEMP MAX:"));
        lcd.setCursor(0, 1);
        lcd.print(convertCtoF(tempMax));
        break;
    case PAUSED:
        lcd.clear();
        lcd.print(F("PAUSED"));
        break;
    }  
}

void checkButtons(void){
  //Checks for input on LCD buttons
  uint8_t buttons = lcd.readButtons();
 
  //add error handling for if min is greater than max, less than 0, etc.
  if (buttons) {
    if (buttons & BUTTON_UP) {
      if (state == SET_CO2_MAX && co2Max < 100) {
        co2Max = co2Max + 1;
      }
      else if (state == SET_RH_MIN) {
        if (rhMin < 95 && rhMax > rhMin + 5) rhMin = rhMin + 1;
      }
      else if (state == SET_RH_MAX) {
        if (rhMax < 100)rhMax = rhMax + 1;
      }
      else if (state == SET_TEMP_MIN) {
        if (tempMin < tempMax - 5) tempMin = tempMin + 1;
      }
      else if (state == SET_TEMP_MAX) {
        tempMax += 1;
      }      
      displayState(state);
    }
    if (buttons & BUTTON_DOWN) {
      if (state == SET_CO2_MAX && co2Max > 4) {
        co2Max = co2Max - 1;
      }
      if (state == SET_RH_MIN && rhMin > 0) {
        rhMin = rhMin - 1;
      }
      if (state == SET_RH_MAX && rhMax > rhMin + 5) {
        rhMax = rhMax - 1;
      }
      if (state == SET_TEMP_MIN && rhMin > 0) {
        tempMin -= 1;
      }
      if (state == SET_TEMP_MAX && tempMax > rhMin + 5) {
        rhMax = rhMax - 1;
      }
      displayState(state);
    }
    //toggles the backlight
    if (buttons & BUTTON_LEFT) {
      if (backlightOn == false) {
        lcd.setBacklight(VIOLET);
        backlightOn = true;
      }
      else {
        lcd.setBacklight(OFF);
        backlightOn = false;
      }
    }
    if (buttons & BUTTON_RIGHT) {
      if (state == MAIN) {
        state = SET_CO2_MAX;
      }
      else if (state == SET_CO2_MAX) {
        EEPROM.update(CO2_MEM_LOC, co2Max);
        state = SET_RH_MIN;
      }
      else if (state == SET_RH_MIN) {
        EEPROM.update(RH_MIN_MEM_LOC, rhMin);
        state = SET_RH_MAX;
      }
      else if (state == SET_RH_MAX) {
        EEPROM.update(RH_MAX_MEM_LOC, rhMax);
        state = SET_TEMP_MIN;
      }
      else if (state == SET_TEMP_MIN) {
        EEPROM.update(TEMP_MIN_MEM_LOC, tempMin);
        state = SET_TEMP_MAX;
      }
      else if (state == SET_TEMP_MAX) {
        EEPROM.update(TEMP_MAX_MEM_LOC, tempMax);
        state = MAIN;
      }
      displayState(state);
    }
    if (buttons & BUTTON_SELECT) {
        if (!pause) {
            pauseRelay();
            pause = true;
            pauseStart = millis();
        }
        else {
            pause = false;
            state = MAIN;
            displayState(state);
        }        
      //pause relay behavior for 5 minutes or until SELECT button is pressed
    }
    delay(200);
  }
}

void toggleRelays(void){
        //Serial.println(F("checking relays"));
        if (fanOn == false && 
          (co2Current > (co2Max * 100) || 
          rhCurrent < rhMin /*|| tempShortAvg > tempMax || tempShortAvg < tempMin */
          )) {
            //Serial.println(F("turning on fan"));
            fanOn = true;
            digitalWrite(FAN_RELAY,LOW);
        }
        //add 10 sec delay after humidifier goes off
        else if (fanOn == true && 
          co2Current < 600 && 
          humOn == false &&
          millis() - humOffTime >= 10000) //10 sec lag between fan turning off and humidifier 
         {
              //Serial.println(F("turning off fan"));
              fanOn = false;
              digitalWrite(FAN_RELAY,HIGH);
        }
        if (humOn == false && 
          rhCurrent < rhMin) {
            //Serial.println(F("turning ON hum"));
            humOn = true;
            digitalWrite(HUM_RELAY,LOW);
        }
        else if (humOn == true && 
          rhCurrent >= rhMax) {
            //Serial.println(F("turning OFF hum"));
            humOn = false;
            digitalWrite(HUM_RELAY,HIGH);
            humOffTime = millis();
        }
}
void pauseRelay() {
    digitalWrite(FAN_RELAY, HIGH);
    digitalWrite(HUM_RELAY, HIGH);
    digitalWrite(RELAY_4, HIGH);
    fanOn = false;
    humOn = false;
    state = PAUSED;
    displayState(state);   
}
