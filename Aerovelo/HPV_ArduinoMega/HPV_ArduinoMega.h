/* HPV Arduino Mega Header */

#ifndef HPV_H_
#define HPV_H_

#include "ANT_interface.h"
#include "GPS_UBLOX.h"
#include "slip.h"


#define SS 53

extern uint8_t buffer[128];

extern uint8_t rxBuffer[32];

extern uint32_t TIME;

extern char slipBuffer[N_SLIP];

void sd_Init();
void sd_Raw_Write(char *data, char *filename);
void sd_Log(char *data);
char *dtoa(char *s, double n);

#endif
