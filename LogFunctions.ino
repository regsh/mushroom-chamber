

void logData(void){
  lastLogging = millis();
    long co2Avg = 0;
    unsigned int tempAvg = 0;
    unsigned int rhAvg = 0;
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
    logfile.print(convertCtoF(tempAvg));
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

}
