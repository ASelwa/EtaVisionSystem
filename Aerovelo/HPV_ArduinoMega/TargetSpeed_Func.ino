/* Target Speed Functions */
/*
 * Updates profileNum, profileFilename, logFilename and starting GPS location if the yellow button is pressed
 */
void toggle() {
  File dataFile;

  static char _word[32] = {0};
  //Read button
  Toggle = !digitalRead(TOGGLE_PIN);
  //If button was pressed, update "profile"
  if (Toggle != lastToggle) {
    sd_Write("Profile changed. ", logFilename);
    //Serial.println("Toggle!");
    lastToggle = Toggle;

    // Reset the state
    distance = 0;
    velocity = 0;
    GPS_totalDistance = 0;
    averagePower = 0;
    power10s = 0;
    // Change target power display to 0
    targetPower = 0;
    *((uint8_t*)slipBuffer + 0) = ID_TPOWER;
    *((uint16_t*)(slipBuffer + 1 + 0)) = targetPower;
    *((uint8_t*)slipBuffer + 1 + 2) = 0;
    SlipPacketSend(3, (char*)slipBuffer, &Serial3);
    
    if (simulation_mode) {
      profileNum++;
      
      sprintf(profileFilename, "PF%02d.txt", profileNum+1);
      if (!SD.exists(profileFilename)) {
        profileNum = 0;
      }
      
      simulation_mode = false;
      //Serial.print("Updated Profile #: ");
      //Serial.println(profileNum+1);
      sprintf(profileFilename, "PF%02d.txt", profileNum+1);
      //Serial.println(profileFilename);
      //Serial.println(logFilename);

      dataFile = SD.open(profileFilename, FILE_READ);
      if (dataFile) {
        //Serial.println("File opened");
        // Display no OSD message if successful
        *((uint8_t*)slipBuffer + 0) = ID_SDCOMM;
        *((uint8_t*)(slipBuffer + 1 + 0)) = 1;
        *((uint8_t*)slipBuffer + 1 + 1) = 0;
        SlipPacketSend(2, (char*)slipBuffer, &Serial3);
        
        int n = sd_ReadWord(dataFile, _word);
        memcpy(profileName, _word, n);
        profileName[n] = 0;
        //Serial.println(profileName);
        sd_Log(profileName);

        //Serial.print("Coefficients: ");
        for (int i = 0; i < 7; ++i) {
          sd_ReadWord(dataFile, _word);
          coeff[i] = atof(_word);
        }

        dataFile.close();
      } else {
        sd_Log("Could not open profile with name \"");
        sd_Log(profileFilename);
        sd_Log("\". ");
        
        // Display error message
        *((uint8_t*)slipBuffer + 0) = ID_SDCOMM;
        *((uint8_t*)(slipBuffer + 1 + 0)) = 0;
        *((uint8_t*)slipBuffer + 1 + 1) = 0;
        SlipPacketSend(2, (char*)slipBuffer, &Serial3);
        SlipPacketSend(2, (char*)slipBuffer, &Serial3);
      }
      
      GPS_setStart();
      
    } else {
      simulation_mode = true;
    }
  }
}

/*
 * Loads the coordinates specified for the finish line.
 * Returns true if successful, false if not.
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
  
//  Serial.print("Lat"); Serial.println(LattitudeFinish);
//  Serial.print("Lon"); Serial.println(LongitudeFinish);
//  Serial.print("Alt"); Serial.println(AltitudeFinish);
  
}

/*
 * Calculate target speed from a 6-degree polynomial given distance
 */
int32_t calcSpeed(double distance, double *coeff) {
  double _speed = 0;
  distance -= 45;
  distance = distance / 1000.0; // Convert to km
  if (distance > 5 * 1.6) distance = 5 * 1.6;
  
  for (int i = 0; i < 7; ++i) {
    _speed += pow(distance, 6 - i) * coeff[i];
  }
  
  if (_speed > 200 / .036) _speed = 200 / .036;
  if (_speed < 0) _speed = -1337;

  return _speed;
}

/*
 * Calculate target power given distance
 */
float calcPower(double distance, float startPower, float preSprintPower) {
  double target = 0;
  
  if (distance < 6436) { // More than 1 mile to go
    target = startPower + ((preSprintPower - startPower) / 6436.0) * distance; // y = b + mx
    //Serial.print("Target: "); Serial.println(target);
  } else { // Change to something else if sprint profile is desired
    target = startPower + ((preSprintPower - startPower) / 6436.0) * distance; // y = b + mx
    //Serial.print("Target: "); Serial.println(target);
  }

  return target;
}

/*
 * Calculate target power given displacement
 */
float calcDisplacePower(double displacement, float startPower, float preSprintPower) {
  double target = 0;
  
  displacement /= 1000.0;
  
  //Serial.print("Displacement: "); Serial.println(displacement);
  
  if (displacement > 1609) { // More than 1 mile to go
    target = preSprintPower - ((preSprintPower - startPower) / 6436.0) * displacement; // y = b + mx
    //Serial.print("Target: "); Serial.println(target);
  } else { // Change to something else if real sprint profile is desired
    target = preSprintPower + ((preSprintPower - startPower) / 6436.0) * (1609 - displacement); // y = b + mx
    //Serial.print("Target: "); Serial.println(target);
  }

  return target;
}
