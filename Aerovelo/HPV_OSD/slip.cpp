/*
 * slip.c
 *
 * Created: 20/05/2012 7:24:19 PM
 *  Author: Oleksiy Ryndin
 */

//#include <avr/io.h>
//#include <avr/interrupt.h>
//#include <avr/pgmspace.h>

//#include <util/delay.h>

//#include <stdio.h> 
//#include <stdlib.h>

#include "slip.h"
//#include "uart.h"
//#include <Arduino.h>

#define END (char)192
#define ESC (char)219
#define ESC_END (char)220
#define ESC_ESC (char)221

uint8_t slipState = 0;

int8_t slipIndex = 0;

int8_t SlipReceive(char *slipBuffer, FastSerial *serial) {
    char data;
    int8_t slipLen;

    while (serial->available()) {
        data = (char) (serial->read() & 0xFF);
        
        switch (slipState) {
        case 0: // Default state, just keep receiving data.
            switch (data) {
            case END:
                
                //Serial.println("END received.");
                if (slipIndex > 0) { // Packet fully received.;
                    slipLen = slipIndex;
                    slipIndex = 0;
                    return slipLen;
                }
                // Else ignore the empty packet.
                break;
            case ESC:
                slipState = 1; // Escape sequence initiated.
                break;
            default:
                slipBuffer[slipIndex++] = data;
                break;
            }

            // TODO: Handle buffer overflow.
            break;
        case 1: // Escape sequence was initiated by last character received.
            switch (data) {
            case ESC_END:
                data = END;
                break;
            case ESC_ESC:
                data = ESC;
                break;
            default:
                break;
            }
            slipBuffer[slipIndex++] = data;
            if (slipIndex >= N_SLIP){
             slipLen = slipIndex;
             slipIndex = 0;
             slipState = 0;
             return slipLen; 
            }
            slipState = 0; // Go back to normal data receive;
            break;
        }
    }
    return -1;
}

void SlipPacketSend(uint8_t len, char *buffer, FastSerial *serial) {
    serial->write(END);

    // Send data.
    while (len--) {

        // Special escape sequences in case data is the same as the END or ESC char.
        switch (*buffer) {
        case END:
            serial->write(ESC);
            serial->write(ESC_END);
            break;
        case ESC:
            serial->write(ESC);
            serial->write(ESC_ESC);
            break;
        default:
            serial->write(*buffer);
            break;
        }
        ++buffer;
    }
    serial->write(END); // Send packet end.

}
