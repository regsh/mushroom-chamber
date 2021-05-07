//Controller for environmental chamber built for cultivationg mushrooms
//Reads CO2, temp, humidity data and modulates power to outlet for humidifier/fans accordingly
//Logs conditions at time intervals specified
//Allows for user manipulation of set points via LCD interface

#include <MemoryFree.h> //for monitoring free RAM, diagnosing memory leaks
#include <pgmStrToRAM.h>
#include <SPI.h> //data logging shield SPI communication
#include <SD.h> //SD card data logging
#include <Wire.h> //I2C communication shared by LCD and SCD30 sensor
#include <RTClib.h> //RTC on data logging shield
#include <Adafruit_RGBLCDShield.h> //LCD shield
#include <SparkFun_SCD30_Arduino_Library.h> //for CO2/temp/RH sensor (Sensiron SCD30)
#include <EEPROM.h> //for persistent data storage of set points

// how many milliseconds between retrieving data and logging it (ms).
#define DATA_INTERVAL 15000 //mills between collection of data points
#define LOG_INTERVAL  300000 // mills between entries (reduce to take more/faster data)
#define SYNC_INTERVAL 300000 // mills between calls to flush() - to write data to the card 
#define PAUSE_INTERVAL 300000 //mills to pause relay behavior upon user click of 'SELECT' button

#define ECHO_TO_SERIAL   1 // echo data to serial port

//digital pins for controlling relay
#define FAN_RELAY 3 //Relay controlling the fan (IN2)
#define HUM_RELAY 4   //Relay controlling the humidifier (IN3)
#define RELAY_4 2

//Digital pins connected to data logger LEDs
#define redLEDpin 5
#define greenLEDpin 6

// color definitions for setting the backlight
#define OFF 0x0
#define VIOLET 0x5
#define WHITE 0x7

//representation of GUI state for toggling through the display
#define MAIN 0
#define SET_CO2_MAX 1
#define SET_RH_MIN 2
#define SET_RH_MAX 3
#define SET_TEMP_MIN 4
#define SET_TEMP_MAX 5
#define PAUSED 6

#define CO2_MEM_LOC 0
#define TEMP_MIN_MEM_LOC 1
#define TEMP_MAX_MEM_LOC 2
#define RH_MIN_MEM_LOC 3
#define RH_MAX_MEM_LOC 4

// The LCD shield connected to UNO using I2C bus (A4 and A5)
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
//RTC for data logger
RTC_PCF8523 rtc;

//SCD30 Co2,Temp,RH sensor
SCD30 airSensor;

// for the data logging shield, we use digital pin 10 for the SD cs line
const int chipSelect = 10;
char filename[] = "LOGGER01.CSV";
// the logging file
File logfile;

DateTime now;
unsigned long syncTime = 0; // time of last sync()
unsigned long lastReading = 0; //time of last data reading
unsigned long lastLogging = 0; //time of last data log to SD card
unsigned long pauseStart = 0; //time relay was paused 

bool backlightOn = false;
bool fanOn = false;
bool humOn = false;
bool pause = false;
long humOffTime = 0; //variable for storing timepoint when humidifier turns off 
uint8_t co2Max;
uint8_t rhMin;
uint8_t rhMax;
uint8_t tempMin;
uint8_t tempMax;
uint8_t state = MAIN;

uint8_t rhData[20];
uint8_t rhCurrent;

uint8_t tempData[20];
uint8_t tempCurrent;

int co2Data[20];
int co2Current;

uint8_t currentIdx = 0;

//Function for printing error information for debugging
//Turns on Red LED and stops program
void error(const char *str)
{
  Serial.print(F("error: "));
  Serial.println(str);
  // red LED indicates error
  digitalWrite(redLEDpin, HIGH);
  while (1);
}
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
        lcd.print(tempCurrent);
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

void addData(uint8_t rh, uint8_t temp, int co2) {
    rhData[currentIdx] = rh;
    tempData[currentIdx] = temp;
    co2Data[currentIdx] = co2;
    currentIdx = (currentIdx + 1) % 20;
}

int convertCtoF(float tempC){
   float product = tempC * 9;
   product = product/5;
   product += 32;
   return (int)product + 0.5;  //casts are truncated, so without adding the 0.5 never rounds up (i.e. 3.6 becomes 3)
}

//initializes co2 and rh data queues with values specified
//could use expected environmental values or initial readings from sensor
void initializeQueues(uint8_t rh, uint8_t temp, int co2) {
  for (int i = 0; i < 20; i++) {
        rhData[i] = rh;
        tempData[i] = temp;
        co2Data[i] = co2;
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

void printRoot() {
  File root = SD.open("/");
  if (root) {
    root.rewindDirectory();
    while (true) {
      File entry =  root.openNextFile();
      if (! entry) {
        Serial.println(F("EOF"));
        // no more files
        break;
      }
      Serial.print(entry.name());
      Serial.print(F("\t\t"));
      Serial.println(entry.size(), DEC);//need to check if directory first
      entry.close();
    }
  }
  root.close();
}

void setup(void)
{
  Serial.begin(115200);
  digitalWrite(SDA,1);
  digitalWrite(SCL,1);

  // Debugging LEDs for data logger
  pinMode(redLEDpin, OUTPUT);
  pinMode(greenLEDpin, OUTPUT);
  //Signal for toggling relays
  pinMode(FAN_RELAY, OUTPUT);
  pinMode(HUM_RELAY, OUTPUT);
  digitalWrite(FAN_RELAY,HIGH);
  digitalWrite(HUM_RELAY,HIGH);

  co2Max = EEPROM.read(CO2_MEM_LOC);
  rhMin = EEPROM.read(RH_MIN_MEM_LOC);
  rhMax = EEPROM.read(RH_MAX_MEM_LOC);
  tempMin = EEPROM.read(TEMP_MIN_MEM_LOC);
  tempMax = EEPROM.read(TEMP_MAX_MEM_LOC);

  // initialize the SD card
  Serial.print(F("Initializing SD card..."));
  // sets SD chip select pin to output
  pinMode(10, OUTPUT);
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    error("Card not present or unable to be initialized");
  }
  Serial.println(F("card initialized."));
  // create a new file

  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i / 10 + '0';
    filename[7] = i % 10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE);
      break;  // leave the loop!
    }
  }
  if (! logfile) {
    error("couldnt create file");
  }

  Serial.print(F("Logging to: "));
  Serial.println(filename);

  // connect to RTC
  Wire.begin();

  if (! rtc.begin()) {
    error("RTC failed");
  }

  if (! rtc.initialized() || rtc.lostPower()) {
    Serial.println("RTC is NOT initialized, let's set the time!");
    //Sets RTC to time program was compiled if new device or power lost
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    //
    // Note: allow 2 seconds after inserting battery or applying external power
    // without battery before calling adjust(). This gives the PCF8523's
    // crystal oscillator time to stabilize. If you call adjust() very quickly
    // after the RTC is powered, lostPower() may still return true.
  }

  // When time needs to be re-set on a previously configured device, the
  // following line sets the RTC to the date & time this sketch was compiled
  //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  // This line sets the RTC with an explicit date & time, for example to set
  // January 21, 2014 at 3am you would call:
  // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));

  // When the RTC was stopped and stays connected to the battery, it has
  // to be restarted by clearing the STOP bit. Let's do this to ensure
  // the RTC is running.
  rtc.start();
  //NOTE: RTC can be offset to adjust for temperature, age etc. See documentation and example
  //PCF8523 sketches for doing so
  logfile.println(F("Time, Co2, Temp(C),Temp(F), RH, FanOn, HumOn, Co2Max, RHMin, RHMax, FreeMem"));

#if ECHO_TO_SERIAL
  Serial.println(F("Time, Co2Current, TempCurrent(F), RHCurrent, FanOn, HumOn, Co2Max, RHMin, RHMax, FreeMem"));
#endif //ECHO_TO_SERIAL

  //LCD SET-UP
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  lcd.setBacklight(OFF);
  
  //SCD30 temp/humidity/CO2 sensor set-up
  //if(airSensor.begin(Wire,true) == false) when you want auto-calibration turned on
  if (airSensor.begin(Wire, false) == false)
  {
    error("Sensor error");
  }
  else {
    Wire.setWireTimeout(3000, true); //sensor can stretch clock up to 150ms
    //airSensor.setMeasurementInterval(5);
    //https://www.fpaynter.com/tag/i2c-freeze/
    while (!airSensor.dataAvailable()) {
    }
    initializeQueues(airSensor.getHumidity(),convertCtoF(airSensor.getTemperature()), airSensor.getCO2());
    displayState(state);
  }
}

void loop(void)
{
  /*
  if(humOn){digitalWrite(HUM_RELAY, LOW);}
  else {digitalWrite(HUM_RELAY, HIGH);}
  if(fanOn){digitalWrite(FAN_RELAY,LOW);}
  else{digitalWrite(FAN_RELAY,HIGH);}
  */
  
  //The SCD30 has data ready every two seconds, can reconfigure for more/less frequent data collection
  if (Serial.available() > 0) {
    logfile.close();
    char c = Serial.read();
    Serial.println(c);
    int numData = 0;
    char fn[13] = "LOGGER00.CSV";
    String input;
    while (Serial.available()) Serial.readString();
    switch (c) {
      //prints directory from SD card on Serial monitor
      case 'r':
        printRoot();
        if (Serial.available()) Serial.read();
        break;
      //initiates file open function. file number must follow
      case 'o':
        if (Serial.available() > 0) Serial.readString();
        while (Serial.available() == 0) {}
        input = Serial.readString();
        fn[6] = input[0];
        fn[7] = input[1];
        Serial.println(fn);
        File dataFile = SD.open(fn);
        // if the file is available, write contents to Serial monitor
        if (dataFile) {
          while (dataFile.available()) {
            Serial.write(dataFile.read());
          }
          Serial.println(F("EOF"));
          dataFile.close();
        }
        // if the file isn't open, pop up an error:
        else {
          Serial.println(F("error opening file"));
        }
        break;
    }
    logfile = SD.open(filename, FILE_WRITE);
  }

  //Checks to see if LOG_INTERVAL has passed since last reading and logs data if so
  if (millis() - lastReading >= DATA_INTERVAL) {
    // fetch the time
    now = rtc.now();
    if (airSensor.dataAvailable()) {
      //green LED indicates data is being collected
      digitalWrite(greenLEDpin, HIGH);
      rhCurrent = airSensor.getHumidity();
      tempCurrent = convertCtoF(airSensor.getTemperature());
      co2Current = airSensor.getCO2();
      //gets current sensor data
      addData(rhCurrent, tempCurrent, co2Current);
      displayState(state);
    }

    //LOGIC FOR RELAY
    if (pause && (millis() - pauseStart >= PAUSE_INTERVAL)) {
        pause = false;
        state = MAIN;
        displayState(state);
    }
    if (!pause) {
        //Serial.println(F("checking relays"));
        if (fanOn == false && 
          (co2Current > (co2Max * 100) || 
          rhCurrent < rhMin /*|| tempCurrent > tempMax || tempCurrent < tempMin */
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
#if ECHO_TO_SERIAL
//"Time, Co2Current, TempCurrent(F), RHCurrent, FanOn, HumOn, Co2Max, RHMin, RHMax, FreeMem"
    Serial.print('"');
    Serial.print(now.year(), DEC);
    Serial.print("/");
    Serial.print(now.month(), DEC);
    Serial.print("/");
    Serial.print(now.day(), DEC);
    Serial.print(" ");
    Serial.print(now.hour(), DEC);
    Serial.print(":");
    Serial.print(now.minute(), DEC);
    Serial.print(":");
    Serial.print(now.second(), DEC);
    Serial.print('"');
    Serial.print(", ");
    Serial.print(co2Current);
    Serial.print(", ");
    Serial.print((tempCurrent));
    Serial.print(", ");          
    Serial.print(rhCurrent);
    Serial.print(", ");
    Serial.print(fanOn);
    Serial.print(", ");
    Serial.print(humOn);
    Serial.print(", ");
    Serial.print(co2Max * 100);
    Serial.print(", ");
    Serial.print(rhMin);
    Serial.print(", ");
    Serial.print(rhMax);
    Serial.print(", ");
    Serial.print(freeMemory());
    Serial.println();
#endif //ECHO_TO_SERIAL
    lastReading = millis();
  }
  if (millis() - lastLogging >= LOG_INTERVAL) {
    //TODO: reset state and backlight
    lastLogging = millis();
    long co2Avg = 0;
    float tempAvg = 0;
    float rhAvg = 0;
    for(uint8_t i = 0; i < 20; i ++){
      co2Avg += co2Data[i];
      tempAvg += tempData[i];
      rhAvg += rhData[i];
    }
    co2Avg = co2Avg/20;
    tempAvg = tempAvg/20;
    rhAvg = rhAvg/20;
    
    // log time
    logfile.print('"');
    logfile.print(now.year(), DEC);
    logfile.print("/");
    logfile.print(now.month(), DEC);
    logfile.print("/");
    logfile.print(now.day(), DEC);
    logfile.print(" ");
    logfile.print(now.hour(), DEC);
    logfile.print(":");
    logfile.print(now.minute(), DEC);
    logfile.print(":");
    logfile.print(now.second(), DEC);
    logfile.print('"');
    logfile.print(", ");

    // log sensor data
    logfile.print(co2Avg);
    logfile.print(", ");
    logfile.print((tempAvg));
    logfile.print(", ");
    logfile.print(rhAvg);
    logfile.print(", ");
    logfile.print(fanOn);
    logfile.print(", ");
    logfile.print(humOn);
    logfile.print(", ");
    logfile.print(co2Max * 100);
    logfile.print(", ");
    logfile.print(rhMin);
    logfile.print(", ");
    logfile.print(rhMax);
    logfile.print(", ");
    logfile.print(freeMemory());
    logfile.println();

    //Green LED off when data collection is complete
    digitalWrite(greenLEDpin, LOW);

  }
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
  digitalWrite(greenLEDpin, LOW);

  // Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
  // which uses a bunch of power and takes time
  if ((millis() - syncTime) < SYNC_INTERVAL) return;
  syncTime = millis();

  // blink LED to show we are syncing data to the card & updating FAT!
  digitalWrite(redLEDpin, HIGH);
  logfile.flush();
  digitalWrite(redLEDpin, LOW);
  state = MAIN;
  displayState(state);

}
