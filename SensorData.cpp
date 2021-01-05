#include <cppQueue.h>
#include "SensorData.h"

/// <summary>Constructor</summary>
/// <param name="recSize">Size in bytes of each reading from sensor</param>
/// <param name="points">Number of points to be averaged</param>
SensorData::SensorData(uint8_t recSize, uint8_t points){
    data = new cppQueue(recSize, points, FIFO, true);
    capacity = points;
    sum = 0;
}

int SensorData::addData(int newData){
  uint8_t rec;
  data->push(&newData);
  sum += newData;
  return getCurrentAvg();
}
int SensorData::getSum() {
    return sum;
}

int SensorData::getCurrentAvg(){
    if (data->getCount() == 0)return 0;
    else return sum / data->getCount();
}


