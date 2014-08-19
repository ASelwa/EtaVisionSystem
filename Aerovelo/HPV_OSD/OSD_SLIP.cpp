#include "OSD_SLIP.h"

extern float cadence, power;
extern uint32_t GPS_Time, GPS_Distance, GPS_Displacement;
extern uint8_t GPS_NumSats;
extern int32_t GPS_Altitude, GPS_Heading;
extern double  GPS_Speed, simulatedSpeed, targetSpeed;
extern UTC_t UTC;
extern uint8_t START, heartRate;
extern int8_t profile_Num;
extern char profileName[16];
extern uint8_t calibrationState, mode;
extern int16_t offset;
extern float batteryLevel;
extern uint8_t GPSComm, SDComm;


void OSD_SlipParse(char *slipBuffer) {
  int MsgID = slipBuffer[0];

  Serial.print("MsgID = ");
  Serial.println(MsgID);

  switch(MsgID) {
  case ID_CADENCE:
    Serial.print("Cadence:\n    N = ");
    Serial.print(*((uint8_t*) slipBuffer+3));
    Serial.print(" T = ");
    Serial.println(*((uint16_t*)((uint8_t*)slipBuffer+1)));
    cadence = (*((uint8_t*) slipBuffer+3)*1000.0)/ (1.0* (*((uint16_t*)((uint8_t*)slipBuffer+1))));
    Serial.println(cadence);
    break;
  case ID_HEART:
    Serial.print("Hrt:\n    N = ");
    Serial.print(*((uint8_t*) slipBuffer+3));
    Serial.print(" T = ");
    Serial.println(*((uint16_t*)((uint8_t*)slipBuffer+1)));
    //heartRate = (*((uint8_t*) slipBuffer+3)*1000.0)/ (1.0* (*((uint16_t*)((uint8_t*)slipBuffer+1))));
    Serial.println(heartRate);
    heartRate = *((uint8_t*) (slipBuffer+1));
    break;
  case ID_POWER:
    power = *((uint16_t*)((uint8_t*)slipBuffer+1));
    cadence = *((uint8_t*)(slipBuffer+3));
    break;
  case ID_DISTANCE:
    GPS_Distance = (*((uint32_t*)(slipBuffer+1)))/1000;
    break;
  case ID_DISPLACEMENT:
    GPS_Displacement = (*((uint32_t*)(slipBuffer+1)))/1000;
    break;
  case ID_GPSTIME:
    GPS_Time = *((uint32_t*)(slipBuffer+1));
    break;
  case ID_NUMSATS:
    GPS_NumSats = *((uint8_t*)(slipBuffer+1));
    break;
  case ID_ALTITUDE:
    GPS_Altitude = *((int32_t*)(slipBuffer+1))/1000;
    break;
  case ID_SPEED:
    GPS_Speed = ((double)(*((int32_t*)(slipBuffer+1))))*0.036; // km/hr
    break;
  case ID_SIM_SPEED:
    simulatedSpeed = ((double)(*((int32_t*)(slipBuffer+1))))*0.036; // km/hr
    break;
  case ID_HEADING:
    GPS_Heading = *((int32_t*)(slipBuffer+1));
    break;
  case ID_UTC:
    UTC = *((UTC_t*)(slipBuffer+1));
    break;
  case ID_START:
    START = *((uint8_t*)(slipBuffer+1));
    break;
  case ID_PROFNUM:
    profile_Num = *((int8_t*)(slipBuffer+1));
    break;
  case ID_TSPEED:
    targetSpeed = ((double)(*((int32_t*)(slipBuffer+1))))*.036;
    break;
  case ID_PROFNAME:
    memcpy(profileName, ((char*)(slipBuffer + 1 + 0)), 16);
    break;
  case ID_CALIBRATION:
    calibrationState = *((uint8_t*) (slipBuffer+1));
    offset = *((int16_t*) (slipBuffer+1+1));
    break;
  case ID_BATTERY:
    batteryLevel = *((uint16_t*)((uint8_t*)slipBuffer+1)) * 1.0 / 100;
    break;
  case ID_GPSCOMM:
    GPSComm = *((uint8_t*) (slipBuffer+1));
    break;
  case ID_SDCOMM:
    SDComm = *((uint8_t*) (slipBuffer+1));
    break;
  case ID_MODE:
    mode = *((uint8_t*) (slipBuffer+1));
    break;
  default:;
  }
}




