#include "slip.h"

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
#define ID_MODE         21
#define ID_SDCOMM       22
#define ID_SIMPLEDISPLACEMENT  30
#define ID_BRAKE_MODE 31
#define ID_ACCEL 32

typedef struct {
          uint16_t year;
          uint8_t month;
          uint8_t day;
          uint8_t hour;
          uint8_t minute;
          uint8_t second;
        } UTC_t;


// Receive and parse SLIP msg
void OSD_SlipParse(char *slipBuffer);



