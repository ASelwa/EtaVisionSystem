/*
 * user.h
 *
 * Created: 02/05/2012 8:25:27 PM
 *  Author: Oleksiy Ryndin
 */ 


#ifndef SLIP_H_
#define SLIP_H_

#include <inttypes.h>
#include <Arduino.h>
#define N_SLIP 32

int8_t SlipReceive(char *slipBuffer, HardwareSerial *serial);
void SlipPacketSend(uint8_t len, char *buffer, HardwareSerial *serial);
//int8_t NmeaReceive(void);


#endif /* SLIP_H_ */
