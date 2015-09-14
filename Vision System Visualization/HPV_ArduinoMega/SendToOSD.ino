/* SendToOSD.ino
 *
 *  This file contains all the functions used to send data to the OSD
 *
 */

/**************** POWER ****************/
void sendPower() {
  // Real Power and Cadence
  *((uint8_t*)slipBuffer + 0) = ID_POWER;
  *((uint16_t*)(slipBuffer + 1 + 0)) = power;
  *((uint8_t*)slipBuffer + 1 + 2) = cadence;
  *((uint8_t*)slipBuffer + 1 + 3) = 0;
  SlipPacketSend(4, (char*)slipBuffer, &Serial3);

/*
  // Average Power
  *((uint8_t*)slipBuffer + 0) = ID_AVG_POWER;
  *((uint16_t*)(slipBuffer + 1 + 0)) = averagePower;
  *((uint8_t*)slipBuffer + 1 + 2) = 0;
  SlipPacketSend(3, (char*)slipBuffer, &Serial3);
  
  // 10 second running average power
  *((uint8_t*)slipBuffer + 0) = ID_10S_POWER;
  *((uint16_t*)(slipBuffer + 1 + 0)) = power10s;
  *((uint8_t*)slipBuffer + 1 + 2) = 0;
  SlipPacketSend(3, (char*)slipBuffer, &Serial3);
*/
}

void sendTargetPower(float tarP) {
  *((uint8_t*)slipBuffer + 0) = ID_TPOWER;
  *((uint16_t*)(slipBuffer + 1 + 0)) = tarP;
  *((uint8_t*)slipBuffer + 1 + 2) = 0;
  SlipPacketSend(3, (char*)slipBuffer, &Serial3); 
}


/**************** SPEED ****************/
void sendSpeed() {
  
  velAvgInt = velocity*100;
  // Send Speed through SLIP
  *((uint8_t*)slipBuffer + 0) = ID_SPEED;
  *((int32_t*)(slipBuffer + 1 + 0)) = velocity * 100; //GPS.Ground_Speed; //velAvgInt; 
  *((uint8_t*)slipBuffer + 1 + 4) = 0;
  SlipPacketSend(6, (char*)slipBuffer, &Serial3);
}

void sendTargetSpeed() {
  //Send target speed through SLIP
  *((uint8_t*)slipBuffer + 0) = ID_TSPEED;
  *((int32_t*)(slipBuffer + 1 + 0)) = velocity * 100;
  *((uint8_t*)slipBuffer + 1 + 4) = 0;
  SlipPacketSend(6, (char*)slipBuffer, &Serial3);
}
 
void sendSimSpeed() {
    // Send Simulated Speed through SLIP
  *((uint8_t*)slipBuffer + 0) = ID_SIM_SPEED;
  *((int32_t*)(slipBuffer + 1 + 0)) = velocity * 100; // Will be converted to km / h in OSD_SLIP
  *((uint8_t*)slipBuffer + 1 + 4) = 0;
  SlipPacketSend(6, (char*)slipBuffer, &Serial3);
}



/**************** DISTANCE ****************/
void sendDistance() {
  //Send Distance through SLIP (hijacked for simulated distance)
  *((uint8_t*)slipBuffer + 0) = ID_DISTANCE;
  *((uint32_t*)(slipBuffer + 1 + 0)) = (COURSE_LENGTH - distance) * 1000; //(uint32_t)distance * 1000;
  *((uint8_t*)slipBuffer + 1 + 4) = 0;
  SlipPacketSend(6, (char*)slipBuffer, &Serial3);
}

void sendSimpleDisplacement() {
  //Send simpleDisplacement through SLIP
  *((uint8_t*)slipBuffer + 0) = ID_SIMPLEDISPLACEMENT;
  *((int32_t*)(slipBuffer + 1 + 0)) = simpleDisplacement;
  *((uint8_t*)slipBuffer + 1 + 4) = 0;
  SlipPacketSend(6, (char*)slipBuffer, &Serial3);
}

void sendDisplacement() {
  //Send Displacement through SLIP
  *((uint8_t*)slipBuffer + 0) = ID_DISPLACEMENT;
  if (SIMULATION) {
    *((int32_t*)(slipBuffer + 1 + 0)) = (COURSE_LENGTH - distance) * 1000; // Assume same as distance
  } else {
    *((int32_t*)(slipBuffer + 1 + 0)) = displacement;
  }
  *((uint8_t*)slipBuffer + 1 + 4) = 0;
  SlipPacketSend(6, (char*)slipBuffer, &Serial3);
}


/**************** ACCELERATION ****************/
void sendAccel() {
    
    if (SERIAL_PRINT) { Serial.print("Acceleration: "); Serial.println(accel*105/100); }  
   
    *((uint8_t*)slipBuffer + 0) = ID_ACCEL;
    *((int32_t*)(slipBuffer + 1 + 0)) = accel*-105/100;
    //*((int32_t*)(slipBuffer + 1 + 0)) = (int32_t)(accel*G_BM*100); // 100*m/s^2 scaling, gets adjusted in OSD_SLIP
    *((uint8_t*)slipBuffer + 1 + 4) = 0;
    SlipPacketSend(6, (char*)slipBuffer, &Serial3);
}
/*
void sendAccel() {
  
  if (BRAKE_MODE) {
    dof.readAccel();
    accAvg.addValue( dof.calcAccel(dof.az) );
    accel = (int32_t)((accAvg.getAverage() - abias[2] - 1)*1000*G_BM*105);
    
    if (SERIAL_PRINT) { Serial.print("Acceleration: "); Serial.println(accel/1000); }  
   
    *((uint8_t*)slipBuffer + 0) = ID_ACCEL;
    *((int32_t*)(slipBuffer + 1 + 0)) = accel;
    //*((int32_t*)(slipBuffer + 1 + 0)) = (int32_t)(accel*G_BM*100); // 100*m/s^2 scaling, gets adjusted in OSD_SLIP
    *((uint8_t*)slipBuffer + 1 + 4) = 0;
    SlipPacketSend(6, (char*)slipBuffer, &Serial3);
  }
} */




/**************** MODES & BOOLS ****************/
void sendMode() {
  
  //Serial.print("SIMULATION ");
  //Serial.println(SIMULATION);
  
  // Send real time or simulation mode through SLIP
  *((uint8_t*)slipBuffer + 0) = ID_MODE;
  *((uint8_t*)(slipBuffer + 1 + 0)) = SIMULATION;
  *((uint8_t*)slipBuffer + 1 + 2) = 0;
  SlipPacketSend(2, (char*)slipBuffer, &Serial3);
}

void sendGPSCOMM(int signal) {
  // Sends confirmation that received from GPS
  *((uint8_t*)slipBuffer + 0) = ID_GPSCOMM;
  *((uint8_t*)(slipBuffer + 1 + 0)) = signal; // Received from GPS
  *((uint8_t*)slipBuffer + 1 + 1) = 0;
  SlipPacketSend(2, (char*)slipBuffer, &Serial3);
}

void sendBrakeMode(int signal) {
   *((uint8_t*)slipBuffer + 0) = ID_BRAKE_MODE;
   *((uint8_t*)(slipBuffer + 1 + 0)) = signal; // Change this value
   *((uint8_t*)slipBuffer + 1 + 1) = 0;
   SlipPacketSend(2, (char*)slipBuffer, &Serial3);
}


/**************** MISC ****************/
void sendHeartRate() {
  *((uint8_t*)slipBuffer + 0) = ID_HEART;
  *((uint8_t*)(slipBuffer + 1 + 0)) = Hrt;
  *((uint8_t*)slipBuffer + 1 + 1) = 0;
  SlipPacketSend(2, (char*)slipBuffer, &Serial3);
}

void sendBattery() {
      // Send battery information
      float batteryLevel = getBatteryLevel();
      //Serial.println(batteryLevel);
      *((uint8_t*)slipBuffer + 0) = ID_BATTERY;
      *((uint16_t*)(slipBuffer + 1 + 0)) = batteryLevel * 100;
      *((uint8_t*)slipBuffer + 1 + 2) = lowBattery(batteryLevel);
      *((uint8_t*)slipBuffer + 1 + 3) = 0;
      SlipPacketSend(4, (char*)slipBuffer, &Serial3);
}

void sendTemperature() {
  // Send temperature information
  uint16_t temperature = readTemp();
  *((uint8_t*)slipBuffer + 0) = ID_TEMPERATURE;
  *((int16_t*)(slipBuffer + 1 + 0)) = temperature;
  *((uint8_t*)slipBuffer + 1 + 2) = highTemp(temperature);
  *((uint8_t*)slipBuffer + 1 + 3) = 0;
  SlipPacketSend(4, (char*)slipBuffer, &Serial3);
}

void receiveOSD() {
  // Receive any messages from the OSD over SLIP?
  int8_t slipLen;
  slipLen = SlipReceive(slipBuffer, &Serial);
  if (slipLen > 0) {
    slipBuffer[slipLen] = 0;
    if (SERIAL_PRINT) { Serial.println(slipBuffer); }
    if (slipBuffer[0] == '!') START = 1;
    else if (slipBuffer[0] == '*') START = 0;
  }
}


/**************** PROFILE ****************/
void sendProfileNum() {
  //Send profileNum through SLIP
  *((uint8_t*)slipBuffer + 0) = ID_PROFNUM;
  *((int8_t*)(slipBuffer + 1 + 0)) = profileNum;
  *((uint8_t*)slipBuffer + 1 + 1) = 0;
  SlipPacketSend(3, (char*)slipBuffer, &Serial3);
}
      
void sendProfileName() {
  //Send profileName through SLIP
  *((uint8_t*)slipBuffer + 0) = ID_PROFNAME;
  memcpy(((int8_t*)(slipBuffer + 1 + 0)), profileName, 16);
  *((uint8_t*)slipBuffer + 1 + 16) = 0;
  SlipPacketSend(18, (char*)slipBuffer, &Serial3);  
}






