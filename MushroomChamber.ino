//Controller for environmental chamber built for cultivationg mushrooms
//Reads CO2, temp, humidity data and modulates power to outlet for humidifier/fans accordingly
//Logs conditions at time intervals specified
//Allows for user manipulation of set points via LCD interface

//TO-DO- change logging behavior to take average from all 20 readings
//Use variable to determine number of readings to average for relay behavior

#include <MemoryFree.h> //for monitoring free RAM, diagnosing memory leaks
#include <pgmStrToRAM.h>
#include <SPI.h> //data logging shield SPI communication
#include <SD.h> //SD card data logging
#include <Wire.h> //I2C communication shared by LCD and SCD30 sensor
#include <RTClib.h> //RTC on data logging shield
#include <Adafruit_RGBLCDShield.h> //LCD shield
#include <SparkFun_SCD30_Arduino_Library.h> //for CO2/temp/RH sensor (Sensiron SCD30)

// how many milliseconds between retrieving data and logging it (ms).
#define DATA_INTERVAL 15000 //mills between collection of data points
#define LOG_INTERVAL  300000 // mills between entries (reduce to take more/faster data)
#define SYNC_INTERVAL 300000 // mills between calls to flush() - to write data to the card 
#define PAUSE_INTERVAL 300000 //mills to pause relay behavior upon user click of 'SELECT' button

#define ECHO_TO_SERIAL   1 // echo data to serial port

//digital pins for controlling relay
#define FAN_RELAY 2 //Relay controlling the fan (IN2)
#define HUM_RELAY 3 //Relay controlling the humidifier (IN3)
#define RELAY_4 4

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
#define PAUSED 4

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
int co2Max = 1000;
uint8_t rhMin = 70;
uint8_t rhMax = 95;
uint8_t state = MAIN;

uint8_t rhData[20];
uint8_t rhShortAvg; //average value over the last 4 readings

uint8_t tempData[20];
uint8_t tempShortAvg;

int co2Data[20];
int co2ShortAvg;

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
        lcd.print(co2ShortAvg);
        lcd.print(("ppm"));
        lcd.setCursor(0, 1);
        lcd.print(F("Temp:"));
        lcd.print(tempShortAvg);
        lcd.print(F("C"));
        lcd.print(F(" RH:"));
        lcd.print(rhShortAvg);
        lcd.print("%");
        break;
    case SET_CO2_MAX:
        lcd.clear();
        lcd.print(F("CO2 MAX:"));
        lcd.setCursor(0, 1);
        lcd.print(co2Max);
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
    getAvgs();
}

void getAvgs() {
    int sum = 0;
    for (int i = 0; i < 4; i++) {
        sum += rhData[(currentIdx + 20 - i) % 20];
    }
    rhShortAvg = sum / 4;

    sum = 0;
    for (int i = 0; i < 4; i++) {
        sum += tempData[(currentIdx + 20 - i) % 20];
    }
    tempShortAvg = sum / 4;
    sum = 0;
    for (int i = 0; i < 4; i++) {
        uint8_t idx = (currentIdx + 20 - i) % 20;
        sum += co2Data[idx];
    }
    co2ShortAvg = sum / 4;
}

//initializes co2 and rh data queues with values specified
//could use expected environmental values or initial readings from sensor
void initializeQueues(uint8_t rh, uint8_t temp, int co2) {
  for (int i = 0; i < 20; i++) {
        rhData[i] = rh;
        tempData[i] = temp;
        co2Data[i] = co2;

        rhShortAvg = rh;
        tempShortAvg = temp;
        co2ShortAvg = co2;
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
  Serial.begin(9600);

  // Debugging LEDs for data logger
  pinMode(redLEDpin, OUTPUT);
  pinMode(greenLEDpin, OUTPUT);
  //Signal for toggling relays
  pinMode(FAN_RELAY, OUTPUT);
  pinMode(HUM_RELAY, OUTPUT);
  pinMode(RELAY_4, OUTPUT);

  digitalWrite(RELAY_4, HIGH);

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
  logfile.println(F("Time, Co2, Temp, RH, FanOn, HumOn, Co2Max, RHMin, RHMax, FreeMem"));

#if ECHO_TO_SERIAL
  Serial.println(F("Time, Co2, Temp, RH, FanOn, HumOn, Co2Max, RHMin, RHMax, FreeMem"));
#endif //ECHO_TO_SERIAL

  //LCD SET-UP
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  lcd.setBacklight(OFF);
  

  //SCD30 temp/humidity/CO2 sensor set-up
  if (airSensor.begin(Wire, false) == false)
  {
    error("Sensor error");
  }
  else {
    Wire.setWireTimeout(3000, true); //sensor can stretch clock up to 150ms
    //https://www.fpaynter.com/tag/i2c-freeze/
    while (!airSensor.dataAvailable()) {
    }
    initializeQueues(airSensor.getHumidity(),airSensor.getTemperature(), airSensor.getCO2()); //should confirm that this works with the returned data types
    displayState(state);
  }
}

void loop(void)
{
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
      //gets current sensor data
      addData(airSensor.getHumidity(), airSensor.getTemperature(), airSensor.getCO2());
    }

    //LOGIC FOR RELAY
    if (pause && (millis() - pauseStart >= PAUSE_INTERVAL)) {
        pause = false;
        state = MAIN;
        displayState(state);
    }
    if (!pause) {
        if (fanOn == false && (co2ShortAvg > co2Max || rhShortAvg < rhMin)) {
            fanOn = true;
            digitalWrite(FAN_RELAY, LOW);
        }
        if (fanOn == true && co2ShortAvg < ((co2Max + 400) / 2) && rhShortAvg >= (rhMin + rhMax) / 2) {
            fanOn = false;
            digitalWrite(FAN_RELAY, HIGH);
        }
        if (humOn == false && rhShortAvg < rhMin) {
            humOn = true;
            digitalWrite(HUM_RELAY, LOW);
        }
        if (humOn == true && rhShortAvg >= (rhMin + rhMax) / 2) {
            humOn = false;
            digitalWrite(HUM_RELAY, HIGH);
        }
    }

#if ECHO_TO_SERIAL
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
    Serial.print(co2ShortAvg);
    Serial.print(", ");
    Serial.print(tempShortAvg);
    Serial.print(", ");
    Serial.print(rhShortAvg);
    Serial.print(", ");
    Serial.print(fanOn);
    Serial.print(", ");
    Serial.print(humOn);
    Serial.print(", ");
    Serial.print(co2Max);
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
    lastLogging = millis();

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
    logfile.print(co2ShortAvg);
    logfile.print(", ");
    logfile.print(tempShortAvg);
    logfile.print(", ");
    logfile.print(rhShortAvg);
    logfile.print(", ");
    logfile.print(fanOn);
    logfile.print(", ");
    logfile.print(humOn);
    logfile.print(", ");
    logfile.print(co2Max);
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
      if (state == SET_CO2_MAX && co2Max < 10000) {
        co2Max = co2Max + 100;
      }
      else if (state == SET_RH_MIN) {
        if (rhMin < 95 && rhMax > rhMin + 10) rhMin = rhMin + 5;
      }
      else if (state == SET_RH_MAX) {
        if (rhMax < 100)rhMax = rhMax + 5;
      }
      displayState(state);
    }
    if (buttons & BUTTON_DOWN) {
      if (state == SET_CO2_MAX && co2Max > 400) {
        co2Max = co2Max - 100;
      }
      if (state == SET_RH_MIN && rhMin > 0) {
        rhMin = rhMin - 5;
      }
      if (state == SET_RH_MAX && rhMax > rhMin + 5) {
        rhMax = rhMax - 5;
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
        state = SET_RH_MIN;
      }
      else if (state == SET_RH_MIN) {
        state = SET_RH_MAX;
      }
      else if (state == SET_RH_MAX) {
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
  //logfile.flush();
  digitalWrite(redLEDpin, LOW);

}
