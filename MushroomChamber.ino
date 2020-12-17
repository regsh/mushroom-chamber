//Data offloading functionality
//Separate into files: http://www.gammon.com.au/forum/?id=12625
//http://cse230.artifice.cc/lecture/splitting-code.html
//Humidity: 80-95%, drop to 60-70% before harvest
//CO2: 500-1000


//#include <MemoryFree.h> //for monitoring free RAM, diagnosing memory leaks
//#include <pgmStrToRAM.h>
#include <SPI.h> //data logging shield SPI communication
#include <SD.h> //SD card data logging
#include <Wire.h> //I2C communication
#include <RTClib.h> //RTC on data logging shield
#include <Adafruit_RGBLCDShield.h> //LCD shield
#include <utility/Adafruit_MCP23017.h> //microchip in LCD?
#include <SparkFun_SCD30_Arduino_Library.h> //for CO2/temp/RH sensor (Sensiron SCD30)
#include <stdint.h>
#include <ArduinoQueue.h> //for averaging data points to debounce relay behavior


// how many milliseconds between grabbing data and logging it. 1000 ms is once a second
#define DATA_INTERVAL 5000 //mills between collection of data points
#define LOG_INTERVAL  60000 // mills between entries (reduce to take more/faster data)
#define SYNC_INTERVAL 60000 // mills between calls to flush() - to write data to the card 


#define ECHO_TO_SERIAL   1 // echo data to serial port
#define SENSOR_EXISTS   0 //whether the program is running on real (true) or simulated (false) sensor data

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

// The LCD shield connected to UNO using I2C bus (A4 and A5)
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
//RTC for data logger
RTC_PCF8523 rtc;

#if SENSOR_EXISTS
//SCD30 Co2,Temp,RH sensor
SCD30 airSensor;
#endif //SENSOR_EXISTS

// for the data logging shield, we use digital pin 10 for the SD cs line
const int chipSelect = 10;
// the logging file
File logfile;
File root;
DateTime now;
unsigned long syncTime = 0; // time of last sync()
unsigned long lastReading = 0; //time of last data reading
unsigned long lastLogging = 0; //time of last data log to SD card
unsigned long backlight = 0;

bool backlightOn = false;
bool fanOn = false;
bool humOn = false;
int co2Max = 1000;
uint8_t rhMin = 70;
uint8_t rhMax = 95;
uint8_t state = MAIN;

int co2_current = 0;
uint8_t tempC = 0;
uint8_t relHum = 0;

ArduinoQueue<int> co2Readings(12);
ArduinoQueue<uint8_t> rhReadings(12);
long co2Sum = 0;
int rhSum = 0;

void setup(void);
void loop(void);
//Function for printing error information for debugging
//Turns on Red LED and stops program
void error(const char *str)
{
  Serial.print("error: ");
  Serial.println(str);

  // red LED indicates error
  digitalWrite(redLEDpin, HIGH);
  while (1);
}
//Function for displaying the appropriate info on screen of LCD
//based on current state of program
void displayState(int state) {
  switch (state) {
    case MAIN:
      lcd.clear();
      lcd.print("Hello mushrooms!");
      break;
    case SET_CO2_MAX:
      lcd.clear();
      lcd.print("CO2 MAX:");
      lcd.setCursor(0, 1);
      lcd.print(co2Max);
      break;
    case SET_RH_MIN:
      lcd.clear();
      lcd.print("RH MIN:");
      lcd.setCursor(0, 1);
      lcd.print(rhMin);
      break;
    case SET_RH_MAX:
      lcd.clear();
      lcd.print("RH MAX:");
      lcd.setCursor(0, 1);
      lcd.print(rhMax);
      break;
  }
}
//Adds new reading to queue of co2 data points
//Returns the new average over the last 12 readings
int addCo2Data(int newData) {
  co2Sum -= co2Readings.dequeue();
  co2Readings.enqueue(newData);
  co2Sum += newData;
  return co2Sum / 12;
}
//Adds new reading to queue of rh data points
//Returns the new average over last 12 readings
uint8_t addRHData(uint8_t newData) {
  rhSum -= rhReadings.dequeue();
  rhReadings.enqueue(newData);
  rhSum += newData;
  return rhSum / 12;
}
//initializes co2 and rh data queues with values specified
//could use expected environmental values or initial readings from sensor
void initializeQueues(int co2, uint8_t rh) {
  for (int i = 0; i < 12; i++) {
    rhReadings.enqueue(rh);
    co2Readings.enqueue(co2);
  }
  rhSum = rh * 12;
  co2Sum = co2 * 12;
  relHum = rh;
  co2_current = co2;
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
  Serial.print("Initializing SD card...");
  // sets SD chip select pin to output
  pinMode(10, OUTPUT);

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    error("Card not present or unable to be initialized");
  }
  Serial.println("card initialized.");

  // create a new file
  char filename[] = "LOGGER01.CSV";
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

  Serial.print("Logging to: ");
  Serial.println(filename);
  //Serial.println("Enter \"read\" to read file data or any character to simulate sensor data");

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
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  // This line sets the RTC with an explicit date & time, for example to set
  // January 21, 2014 at 3am you would call:
  // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));

  // When the RTC was stopped and stays connected to the battery, it has
  // to be restarted by clearing the STOP bit. Let's do this to ensure
  // the RTC is running.
  rtc.start();
  //NOTE: RTC can be offset to adjust for temperature, age etc. See documentation and example
  //PCF8523 sketches for doing so
  logfile.println("Time, Co2, Temp, RH, FanOn, HumOn, Co2Max, RHMin, RHMax");

#if ECHO_TO_SERIAL
  Serial.println("Time, Co2, Temp, RH, FanOn, HumOn, Co2Max, RHMin, RHMax");
#endif //ECHO_TO_SERIAL

  //LCD SET-UP
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  lcd.print("Hello, mushrooms!");
  lcd.setBacklight(OFF);

#if SENSOR_EXISTS
  //SCD30 temp/humidity/CO2 sensor set-up
  if (airSensor.begin(Wire, true) == false)
  {
    error("Air sensor not detected. Please check wiring. Freezing...");
  }
  else {
    while (!airSensor.dataAvailable()) {
    }
    initializeQueues(airSensor.getCO2(), airSensor.getHumidity()); //should confirm that this works with the returned data types
    tempC = airSensor.getTemperature();
  }

#else
  initializeQueues(400, 40);
  tempC = 25;
#endif //SENSOR_EXISTS
}



void loop(void)
{
  //The SCD30 has data ready every two seconds, can reconfigure for more/less frequent data collection


#if !(SENSOR_EXISTS)
  //Allows Serial manipulation of mock sensor data in the case the sensor is not connected
  /*
  if (Serial.available() > 0) {
    Serial.println(Serial.read());
    Serial.println("Enter value for CO2");
    while (Serial.available() == 0) {}

    String data = Serial.readString();
    Serial.println("You entered: " + data);
    int numData = data.toInt();
    Serial.println("as a number: " + numData);
    co2_current = addCo2Data(numData);
    /*
      Serial.println(Serial.readString());
      Serial.println("Enter value for Temp");
      while(Serial.available() == 0){
      }
      data = Serial.readString();
      numData = data.toInt();
      tempC = numData;
    
    Serial.println(Serial.readString());
    Serial.println("Enter value for RH");
    while (Serial.available() == 0) {
    }
    data = Serial.readString();
    numData = data.toInt();
    relHum = addRHData(numData);
  }
  */
#endif
  while(Serial.available() > 0){
    Serial.println(Serial.readStringUntil('\n'));
  }

  //Checks to see if LOG_INTERVAL has passed since last reading and logs data if so
  if (millis() - lastReading >= DATA_INTERVAL) {
    // fetch the time
    now = rtc.now();
#if SENSOR_EXISTS
    if (airSensor.dataAvailable()) {
      //green LED indicates data is being collected
      digitalWrite(greenLEDpin, HIGH);
      //gets current sensor data
      co2_current = addCo2Data(airSensor.getCO2());
      relHum = addRHData(airSensor.getHumidity());

    }
#endif //SENSOR_EXISTS
    //LOGIC FOR RELAY
    if (fanOn == false && (co2_current > co2Max || relHum < rhMin)) {
      fanOn = true;
      digitalWrite(FAN_RELAY, LOW);
    }
    if (fanOn == true && co2_current < ((co2Max + 400) / 2) && relHum >= (rhMin + rhMax) / 2) {
      fanOn = false;
      digitalWrite(FAN_RELAY, HIGH);
    }
    if (humOn == false && relHum < rhMin) {
      humOn = true;
      digitalWrite(HUM_RELAY, LOW);
    }
    if (humOn == true && relHum >= (rhMin + rhMax) / 2) {
      humOn = false;
      digitalWrite(HUM_RELAY, HIGH);
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
    Serial.print(co2_current);
    Serial.print(", ");
    Serial.print(tempC);
    Serial.print(", ");
    Serial.print(relHum);
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
    logfile.print(co2_current);
    logfile.print(", ");
    logfile.print(tempC);
    logfile.print(", ");
    logfile.print(relHum);
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
    logfile.println();

    //Green LED off when data collection is complete
    digitalWrite(greenLEDpin, LOW);
    //Uncomment to monitor amount of free memory throughout program
    /*
      Serial.print("Free memory: ");
      Serial.print(freeMemory());
      Serial.println();
    */
  }
  if (backlightOn && (millis() - backlight) > 15000) {
    backlightOn = false;
    lcd.setBacklight(OFF);
  }
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
        backlight = millis();
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
      lcd.clear();
      lcd.print("CO2: ");
      lcd.print(co2_current);
      lcd.print("ppm");
      lcd.setCursor(0, 1);
      lcd.print("Temp:");
      lcd.print(tempC);
      lcd.print("C");
      lcd.print(" RH:");
      lcd.print(relHum);
      lcd.print("%");
      delay(3000);
      state = MAIN;
      lcd.clear();
      lcd.print("Hello, mushrooms!");
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
}

void printDirectory(File dir, int numTabs) {
  while (true) {
    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++) {
      Serial.print('\t');
    }
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println("/");
      printDirectory(entry, numTabs + 1);
    } else {
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
    entry.close();
  }
}
