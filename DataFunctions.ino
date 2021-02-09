void addData(uint8_t rh, uint8_t temp, int co2) {
    rhData[currentIdx] = rh;
    tempData[currentIdx] = temp;
    co2Data[currentIdx] = co2;
    currentIdx = (currentIdx + 1) % 20;
    getAvgs();
}

float convertCtoF(float tempC){
   float product = tempC * 9;
   product = product/5;
   product += 32;
   return product;
}

void getAvgs() {
    int sum = 0;
    for (int i = 1; i < 5; i++) {
        //Serial.print((currentIdx + 20 - i) % 20);
        //Serial.print(": ");
        uint8_t d = rhData[(currentIdx + 20 - i) %20];
        //Serial.println(d);
        sum += rhData[(currentIdx + 20 - i) % 20];
    }
    rhShortAvg = sum / 4;

    sum = 0;
    for (int i = 1; i < 5; i++) {
        sum += tempData[(currentIdx + 20 - i) % 20];
    }
    tempShortAvg = sum / 4;
    sum = 0;
    for (int i = 1; i < 5; i++) {
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

void getData(){
      // fetch the time
    now = rtc.now();
    if (airSensor.dataAvailable()) {
      rhCurrent = airSensor.getHumidity();
      tempCurrent = airSensor.getTemperature();
      co2Current = airSensor.getCO2();
      //gets current sensor data
      addData(rhCurrent, tempCurrent, co2Current);
      displayState(state);
    }
}
