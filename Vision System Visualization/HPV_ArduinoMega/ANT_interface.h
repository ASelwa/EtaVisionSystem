/* ANT+ Interface Header */
#ifndef ANT_HEADER_H
  #define ANT_HEADER_H
#include "antmessage.h"
#include <inttypes.h>

#define SYNC 0xA4

#define NET_KEY_ANTP {0xB9, 0xA5, 0x21, 0xFB, 0xBD, 0x72, 0xC3, 0x45, 0x00}
#define NET_KEY_PUBLIC {0xE8, 0xE4, 0x21, 0x3B, 0x55, 0x7A, 0x67, 0xC1, 0x00}


void ANT_SendMessage(uint8_t *buf, uint8_t n);

void ANT_SetupChannel(uint8_t *buf, uint8_t channel, uint8_t Dev_T, uint8_t Trans_T, uint16_t period, uint16_t Dev_Num);

void receiveInterrupt();

uint8_t receiveANT(uint8_t *rxBuf);
#endif
