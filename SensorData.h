#include <cppQueue.h>

#ifndef SENSORDATA_H
#define SENSORDATA_H

//Class to average multiple data points from sensory input
//Uses ArduinoQueue class to implement FIFO data management
//template<class T>
//Ideally this would be  templated class for less memory intensive data types
class SensorData{
  private: 
    cppQueue *data;
    int capacity;
    int sum;
    uint8_t numPoints;
  public:
    SensorData(uint8_t recSize, uint8_t points);
    //SensorData(uint8_t points, int initValue);
    int addData(int newData);
    int getCurrentAvg();
    int getSum();
};

#endif
