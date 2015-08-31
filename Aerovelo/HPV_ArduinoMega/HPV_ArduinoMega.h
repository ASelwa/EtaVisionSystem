/* HPV Arduino Mega Header */

#ifndef HPV_H_
#define HPV_H_

#include "ANT_interface.h"
#include "GPS_UBLOX.h"
#include "slip.h"

#define _SS 53
#define SS 53

// Message IDs
#define ID_CADENCE	1
#define ID_DISTANCE	2
#define ID_DISPLACEMENT 16
#define ID_SPEED	3
#define ID_SIM_SPEED    20
#define ID_GPSTIME      4
#define ID_NUMSATS      5
#define ID_FIX          6
#define ID_ALTITUDE	7
#define ID_HEADING	8
#define ID_UTC          9
#define ID_START        10
#define ID_HEART        11
#define ID_PROFNUM      12
#define ID_TSPEED       13
#define ID_PROFNAME     14
#define ID_POWER        15
#define ID_TPOWER       23
#define ID_10S_POWER    24
#define ID_AVG_POWER    25
#define ID_CALIBRATION  17
#define ID_BATTERY      18
#define ID_TEMPERATURE  26
#define ID_GPSCOMM      19
#define ID_SDCOMM       22
#define ID_MODE         21
#define ID_SIMPLEDISPLACEMENT  30
#define ID_BRAKE_MODE 31
#define ID_ACCEL 32

extern uint8_t buffer[128];
extern uint8_t rxBuffer[32];
extern char slipBuffer[N_SLIP];

extern uint32_t TIME;
extern uint32_t TEMPTIME;

void sd_Init();
void sd_Raw_Write(char *data, char *filename);
void sd_Log(char *data);
char *dtoa(char *s, double n);

#endif
