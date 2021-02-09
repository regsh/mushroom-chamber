void checkSerial(){
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
}

void printToSerial(){
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
    Serial.print((tempCurrent));
    Serial.print(", ");    
    Serial.print(convertCtoF(tempCurrent));
    Serial.print(", ");    
    Serial.print((int)convertCtoF(tempShortAvg));
    Serial.print(", ");   
    Serial.print(rhCurrent);
    Serial.print(", ");
    Serial.print(fanOn);
    Serial.print(", ");
    Serial.print(humOn);
    Serial.print(", ");
    Serial.print(co2Max * 100);
    Serial.print(", ");
    Serial.print(rhMin);
    Serial.print(", ");
    Serial.print(rhMax);
    Serial.print(", ");
    Serial.print(freeMemory());
    Serial.println();
#endif //ECHO_TO_SERIAL
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
