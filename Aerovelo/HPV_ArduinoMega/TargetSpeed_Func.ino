/* Target Speed Functions */
/*
Updates profileNum, profileFilename, logFilename and starting GPS location if the yellow button is pressed
 */
void toggle() {
  File dataFile;

  static char _word[32] = {0};
  //Read button
  Toggle = !digitalRead(8);
  //If button was pressed, update "profile"
  if (Toggle != lastToggle) {
    Serial.println("Toggle!");
    lastToggle = Toggle;

    // Reset the state
    distance = 0;
    velocity = 0;
    GPS_totalDistance = 0;
    
    if (simulation_mode) {
      profileNum++;
  
      if (profileNum > (MAX_PROFILE_NUM)) {
        profileNum = 1;
      }
      
      simulation_mode = false;
      Serial.print("Updated Profile #: ");
      Serial.println(profileNum);
      // Update profileFilename based on profileNum; 0 -> PAUSED
      if (profileNum != 0) {
        // Figure out SD card log iteration.
        sprintf(logFilename, "LG%02u%02u%02u.txt", GPS.UTC.hour, GPS.UTC.day, GPS.UTC.month);//, GPS.UTC.hour, GPS.UTC.minute, GPS.UTC.second);
        sprintf(profileFilename, "PF%02d.txt", profileNum);
        //sprintf(profileFilename, "LALALA");
        Serial.println(profileFilename);
        Serial.println(logFilename);
  
        dataFile = SD.open(profileFilename, FILE_READ);
        if (dataFile) {
          Serial.println("File opened");
          int n = sd_ReadWord(dataFile, _word);
          memcpy(profileName, _word, n);
          profileName[n] = 0;
          Serial.println(profileName);
  
          Serial.print("Coefficients: ");
          for (int i = 0; i < 7; ++i) {
            sd_ReadWord(dataFile, _word);
            coeff[i] = atof(_word);
            Serial.println(coeff[i]);
          }
  
          dataFile.close();
        } else {
          Serial.println("SD Read Error (Coeffs)");
        }
  
        // Check accuracy first?
        
        GPS_setStart();
  
        /*
        dataFile = SD.open("gpsRef.txt", FILE_READ);
  
        sd_ReadWord(dataFile, _word);
        LattitudeStart = atof(_word) * 10000000;
        sd_ReadWord(dataFile, _word);
        LongitudeStart = atof(_word) * 10000000;
        //AltidudeStart = GPS.Altitude;
        dataFile.close();
        */
      } else {
  
        sprintf(logFilename, "LGdflt.txt");
  
        dataFile = SD.open("PFdflt.txt", FILE_READ);
        if (dataFile) {
          Serial.println("File opened");
          int n = sd_ReadWord(dataFile, _word);
          memcpy(profileName, _word, n);
          Serial.println(profileName);
  
          Serial.print("Coefficients: ");
          for (int i = 0; i < 7; ++i) {
            sd_ReadWord(dataFile, _word);
            coeff[i] = atof(_word);
            Serial.println(coeff[i]);
          }
          
  //          coeff[0] = 0;
  //          coeff[1] = -0.3705;
  //          coeff[2] = 4.104;
  //          coeff[3] = -16.399;
  //          coeff[4] = 17.941;
  //          coeff[5] = -121.1;
  //          coeff[6] = 3557.8;
  
          dataFile.close();
        }
        
        GPS_setStart();
  
        /*
        dataFile = SD.open("gpsTest.txt", FILE_READ);
        if (dataFile) {
          sd_ReadWord(dataFile, _word);
          LattitudeStart = atof(_word) * 10000000;
          sd_ReadWord(dataFile, _word);
          LongitudeStart = atof(_word) * 10000000;
          //AltidudeStart = GPS.Altitude;
          dataFile.close();
        }
        */
      }
    } else {
      simulation_mode = true;
    }
  }
}

/*
Calculate target speed from a 6-degree polynomial given distance
 */
int32_t calcSpeed(double distance, double *coeff) {
  double _speed = 0;
  distance = distance / 1000000.0; // Convert to km
  distance = 8 - distance; // Function expects distance from end
  if (distance > 5 * 1.6) distance = 5 * 1.6;
  
  for (int i = 0; i < 7; ++i) {
    _speed += pow(distance, 6 - i) * coeff[i];
  }
  
  if (_speed > 200 / .036) _speed = 200 / .036;

  return _speed;
}
