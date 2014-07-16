#include "slip.h"

// Message IDs
#define ID_CADENCE	1
#define ID_DISTANCE	2
#define ID_SPEED	3
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



