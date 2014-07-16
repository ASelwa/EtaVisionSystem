#include "OSD_SLIP.h"

extern float cadence;
extern uint32_t GPS_Time, GPS_Distance;
extern uint8_t GPS_NumSats;
extern int32_t GPS_Altitude, GPS_Heading;
extern double  GPS_Speed, targetSpeed;
extern UTC_t UTC;
extern uint8_t START, heartRate;
extern int8_t profile_Num;
extern char profileName[16];


void OSD_SlipParse(char *slipBuffer) {
  int MsgID = slipBuffer[0];

  Serial.print("MsgID = ");
  Serial.println(MsgID);

  switch(MsgID)
  {
  case ID_CADENCE:
    Serial.print("Cadence:\n    N = ");
    Serial.print(*((uint8_t*) slipBuffer+3));
    Serial.print(" T = ");
    Serial.println(*((uint16_t*)((uint8_t*)slipBuffer+1)));
    cadence = (*((uint8_t*) slipBuffer+3)*1000.0)/ (1.0* (*((uint16_t*)((uint8_t*)slipBuffer+1))));
    Serial.println(cadence);
    /*
		INSERT CODE HERE
     		*/

    break;

  case ID_HEART:
    Serial.print("Hrt:\n    N = ");
    Serial.print(*((uint8_t*) slipBuffer+3));
    Serial.print(" T = ");
    Serial.println(*((uint16_t*)((uint8_t*)slipBuffer+1)));
    heartRate = (*((uint8_t*) slipBuffer+3)*1000.0)/ (1.0* (*((uint16_t*)((uint8_t*)slipBuffer+1))));
    Serial.println(heartRate);
    /*
		INSERT CODE HERE
     		*/

    break;


  case ID_DISTANCE:
    GPS_Distance = (*((uint32_t*)(slipBuffer+1)))/1000;
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
  default:;
  }
}




