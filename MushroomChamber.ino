//Separate into files: http://www.gammon.com.au/forum/?id=12625
//http://cse230.artifice.cc/lecture/splitting-code.html
//Humidity: 80-95%, drop to 60-70% before harvest
//CO2: 500-1000

//Manipulate memory: https://www.arduino.cc/reference/en/language/variables/utilities/progmem/

#include <Vector.h>
#include "EnvLCD.h"
#include "EnvFactor.h"
#include "ChamberLCD.h"
#include <cppQueue.h>
#include <MemoryFree.h> //for monitoring free RAM, diagnosing memory leaks
#include <pgmStrToRAM.h>
#include <SPI.h> //data logging shield SPI communication
#include <SD.h> //SD card data logging
#include <Wire.h> //I2C communication
#include <RTClib.h> //RTC on data logging shield
#include <Adafruit_RGBLCDShield.h> //LCD shield
#include <utility/Adafruit_MCP23017.h> //microchip in LCD?
#include <SparkFun_SCD30_Arduino_Library.h> //for CO2/temp/RH sensor (Sensiron SCD30)
#include <stdint.h>
#include "SensorData.h"

// how many milliseconds between grabbing data and logging it. 1000 ms is once a second
#define DATA_INTERVAL 5000 //mills between collection of data points
#define LOG_INTERVAL  60000 // mills between entries (reduce to take more/faster data)
#define SYNC_INTERVAL 60000 // mills between calls to flush() - to write data to the card 

#define ECHO_TO_SERIAL   1 // echo data to serial port

//digital pins for controlling relay
#define FAN_RELAY 2 //Relay controlling the fan (IN2)
#define HUM_RELAY 3 //Relay controlling the humidifier (IN3)
#define RELAY_4 4

//Digital pins connected to data logger LEDs
#define redLEDpin 5
#define greenLEDpin 6

//SCD30 Co2,Temp,RH sensor
SCD30 airSensor;

// The LCD shield connected to UNO using I2C bus (A4 and A5)
EnvLCD* elcd;

EnvFactor** factors;
int factorCnt;

SensorData co2Data(2,12);
SensorData rhData(1,12);

long lastCheck;

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

int getTemp() {
    return (int)airSensor.getTemperature();
}

int getHum() {
    return (int)airSensor.getHumidity();
}

int getCo2() {
    return (int)airSensor.getCO2();
}

void setup(void)
{
  Serial.begin(9600);

  //SCD30 temp/humidity/CO2 sensor set-up
  if (airSensor.begin(Wire, true) == false)
  {
    error(("Air sensor error."));
  }
  
  factorCnt = 3;
  EnvFactor* temp = new EnvFactor(String("TEMP (C)"), getTemp, 15, 25, 1);
  EnvFactor *rh = new EnvFactor(String("RH (%)"), getHum, 0, 90, 5);
  EnvFactor *co2 = new EnvFactor(String("CO2 (ppm)"), getCo2, 0, 4000, 100);
  factors = new EnvFactor* [factorCnt];
  factors[0] = temp;
  factors[1] = rh;
  factors[2] = co2;

  const String msg = "Hello mushrooms!";
  elcd = new EnvLCD(factors, factorCnt, msg);
  Serial.println("first factor:");
  Serial.println((factors[0])->GetName());
  Serial.println((factors[0])->GetLow());
  Serial.println((factors[0])->GetHigh());
  Serial.println((factors[0])->GetCurrent());

}
void loop(void)
{
    if (millis() - lastCheck > 1000) {
        elcd->CheckButtons();
    }
    
}
