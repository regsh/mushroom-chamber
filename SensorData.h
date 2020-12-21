#include <ArduinoQueue.h>

#ifndef SENSORDATA_H
#define SENSORDATA_H
//template<class T>
//Ideally this would be  templated class for less memory intensive data types
class SensorData{
  private: 
    ArduinoQueue<int> data;
    int sum;
    uint8_t numPoints;
  public:
    SensorData(uint8_t points);
    //SensorData(uint8_t points, int initValue);
    int addData(int newData);
    int getCurrentAvg();
};

#endif
