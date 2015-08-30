/* SendToOSD.ino
 *
 *  This file contains all the functions used to send data to the OSD
 *
 */
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

void sendSpeed() {
  // Send Speed through SLIP
  *((uint8_t*)slipBuffer + 0) = ID_SPEED;
  *((int32_t*)(slipBuffer + 1 + 0)) = GPS.Ground_Speed;
  *((uint8_t*)slipBuffer + 1 + 4) = 0;
  SlipPacketSend(6, (char*)slipBuffer, &Serial3);
}
      


void sendTargetSpeed() {
  //Send target speed through SLIP
  *((uint8_t*)slipBuffer + 0) = ID_TSPEED;
  *((int32_t*)(slipBuffer + 1 + 0)) = targetSpeed;
  *((uint8_t*)slipBuffer + 1 + 4) = 0;
  SlipPacketSend(6, (char*)slipBuffer, &Serial3);
}
 


void sendHeartRate() {
  *((uint8_t*)slipBuffer + 0) = ID_HEART;
  *((uint8_t*)(slipBuffer + 1 + 0)) = Hrt;
  *((uint8_t*)slipBuffer + 1 + 1) = 0;
  SlipPacketSend(2, (char*)slipBuffer, &Serial3);
}

void sendGPSCOMM() {
  // Sends confirmation that received from GPS
  *((uint8_t*)slipBuffer + 0) = ID_GPSCOMM;
  *((uint8_t*)(slipBuffer + 1 + 0)) = 1; // Received from GPS
  *((uint8_t*)slipBuffer + 1 + 1) = 0;
  SlipPacketSend(2, (char*)slipBuffer, &Serial3);
}

void sendBrakeMode(int signal) {
   *((uint8_t*)slipBuffer + 0) = ID_BRAKE_MODE;
   *((uint8_t*)(slipBuffer + 1 + 0)) = signal; // Change this value
   *((uint8_t*)slipBuffer + 1 + 1) = 0;
   SlipPacketSend(2, (char*)slipBuffer, &Serial3);
}

void sendAccel() {
  // Send acceleration (averaged in the 3 term array like lat, lon, alt)
  *((uint8_t*)slipBuffer + 0) = ID_ACCEL;
  *((int32_t*)(slipBuffer + 1 + 0)) = average(accel, accelTerms); // m/s^2 right now, and then gets adjusted in OSD_SLIP
  *((uint8_t*)slipBuffer + 1 + 4) = 0;
  SlipPacketSend(6, (char*)slipBuffer, &Serial3);
}

void receiveOSD() {
  slipLen = SlipReceive(slipBuffer, &Serial);
  if (slipLen > 0) {
    slipBuffer[slipLen] = 0;
    Serial.println(slipBuffer);
    if (slipBuffer[0] == '!') START = 1;
    else if (slipBuffer[0] == '*') START = 0;
  }
}



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

void sendMode() {
  // Send real time or simulation mode through SLIP
  *((uint8_t*)slipBuffer + 0) = ID_MODE;
  *((uint8_t*)(slipBuffer + 1 + 0)) = simulation_mode;
  *((uint8_t*)slipBuffer + 1 + 2) = 0;
  SlipPacketSend(2, (char*)slipBuffer, &Serial3);
}




