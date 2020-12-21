#include <ArduinoQueue.h> //for averaging data points to debounce relay behavior
#include "SensorData.h"

SensorData::SensorData(uint8_t points){
    data = new ArduinoQueue<int>(points);
}

/*
SensorData::SensorData(uint8_t points, int initValue){
    data = new ArduinoQueue<int>(points);
    for(int i = 0; i < numPoints; i++){
      data.enqueue(initValue);
    }
    sum = initValue * numPoints;
}
*/

int SensorData::addData(int newData){
  if(data.isFull()) data.dequeue();
  data.enqueue(newData);
  sum += newData;
  return getCurrentAvg();
}

int SensorData::getCurrentAvg(){
  return sum/data.itemCount();
}
