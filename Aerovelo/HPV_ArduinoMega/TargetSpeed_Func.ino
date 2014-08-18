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
    sd_Write("Profile changed. ", logFilename);
    Serial.println("Toggle!");
    lastToggle = Toggle;

    // Reset the state
    distance = 0;
    velocity = 0;
    GPS_totalDistance = 0;
    
    if (simulation_mode) {
      profileNum++;
  
      if (profileNum > (MAX_PROFILE_NUM)) {
        profileNum = 0;
      }
      
      simulation_mode = false;
      Serial.print("Updated Profile #: ");
      Serial.println(profileNum+1);
      // Update profileFilename based on profileNum; 0 -> PAUSED
      if (profileNum >= 0) {
        // Figure out SD card log iteration.
        //sprintf(logFilename, "LG%02u%02u%02u.txt", GPS.UTC.hour, GPS.UTC.day, GPS.UTC.month);//, GPS.UTC.hour, GPS.UTC.minute, GPS.UTC.second);
        sprintf(profileFilename, "PF%02d.txt", profileNum+1);
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
          sd_Log("Could not open profile with name \"");
          sd_Log(profileFilename);
          sd_Log("\". ");
          Serial.println("SD Read Error (Coeffs)");
        }
        
        GPS_setStart();
      } /*else {
        //sprintf(logFilename, "LGdflt.txt");
  
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
  
          dataFile.close();
        } else {
          sd_Log("Could not open profile with name \"");
          sd_Log(profileFilename);
          sd_Log("\". ");
          Serial.println("SD Read Error (Coeffs)");
        }
        
        GPS_setStart();
      }*/
    } else {
      simulation_mode = true;
    }
  }
}

/*
 * Returns true if successful, false if not
*/
bool loadFinishCoordinates() {
  static char _word[32] = {0};
  File dataFile = SD.open("FinCoord.txt", FILE_READ);
  
  if (!dataFile) {
    sd_Log("Could not open FinCoord.txt. ");
    return false;
  }
  
  sd_ReadWord(dataFile, _word);
  LattitudeFinish = atof(_word) * 10000000;
  sd_ReadWord(dataFile, _word);
  LongitudeFinish = atof(_word) * 10000000;
  sd_ReadWord(dataFile, _word);
  AltitudeFinish = atof(_word) * 1000;
  dataFile.close();
  
  Serial.println(LattitudeFinish);
  Serial.println(LongitudeFinish);
  Serial.println(AltitudeFinish);
}

/*
 * Calculate target speed from a 6-degree polynomial given distance
 */
int32_t calcSpeed(double distance, double *coeff) {
  double _speed = 0;
  distance = distance / 1000.0; // Convert to km
  distance = distance; // Function expects distance from end
  if (distance > 5 * 1.6) distance = 5 * 1.6;
  
  for (int i = 0; i < 7; ++i) {
    _speed += pow(distance, 6 - i) * coeff[i];
  }
  
  if (_speed > 200 / .036) _speed = 200 / .036;
  if (_speed < 0) _speed = -1337;

  return _speed;
}
