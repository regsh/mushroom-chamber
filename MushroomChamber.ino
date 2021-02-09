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
uint8_t rhShortAvg; //average value over the last 4 readings
uint8_t rhCurrent;

uint8_t tempData[20];
uint8_t tempShortAvg;
float tempCurrent;

int co2Data[20];
int co2ShortAvg;
int co2Current;

uint8_t currentIdx = 0;

void setup(void)
{
  Serial.begin(115200);
  digitalWrite(SDA,1);
  digitalWrite(SCL,1);

  // Debugging LEDs for data logger
  pinMode(redLEDpin, OUTPUT);
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
  }
  // When time needs to be re-set on a previously configured device, the
  // following line sets the RTC to the date & time this sketch was compiled
  //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  rtc.start();
  //NOTE: RTC can be offset to adjust for temperature, age etc. See documentation and example
  //PCF8523 sketches for doing so
  
  logfile.println(F("Time, Co2, Temp(C),Temp(F), RH, FanOn, HumOn, Co2Max, RHMin, RHMax, FreeMem"));
#if ECHO_TO_SERIAL
  Serial.println(F("Time, Co2MinAvg, TempCurrent(C), TempCurrent(F), TempMinAvg(F), RHCurrent, FanOn, HumOn, Co2Max, RHMin, RHMax, FreeMem"));
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
    rhCurrent = (uint8_t)airSensor.getHumidity();
    tempCurrent = airSensor.getTemperature();
    co2Current = airSensor.getCO2();
    initializeQueues(rhCurrent,tempCurrent,co2Current);
    displayState(state);
  }
}
void loop(void)
{
  //checks for input from serial monitor and provides appropriate data to user
  checkSerial();
  
  //Checks to see if DATA_INTERVAL has passed since last reading and adds sensor data if so
  if (millis() - lastReading >= DATA_INTERVAL) {
    getData();
    
    if (pause && (millis() - pauseStart >= PAUSE_INTERVAL)) {
        pause = false;
        state = MAIN;
        displayState(state);
    }
    if (!pause) {
        toggleRelays();
    }
    lastReading = millis();
  }
  
  //logs data after specified interval
  if (millis() - lastLogging >= LOG_INTERVAL) {
    logData();    
  }
  
  //checks for user input on LCD buttons and changes set points
  checkButtons();
  
  if ((millis() - syncTime) < SYNC_INTERVAL) return;
  syncTime = millis();

  // blink LED to show we are syncing data to the card & updating FAT!
  digitalWrite(redLEDpin, HIGH);
  logfile.flush();
  digitalWrite(redLEDpin, LOW);
  //resets the state to home screen
  state = MAIN;
  displayState(state);
}
