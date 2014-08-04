#ifndef SIMULATION_H
#define SIMULATION_H

#include <math.h>
#include <inttypes.h>
#include "Arduino.h"

float getElevation(float distance);

uint16_t cB(const uint8_t x, const uint8_t y);

void readPowerMeter(uint8_t *pwrRx, uint8_t print, uint16_t *time_interval, float* power, float* cadence_out, bool* coast);

void simulate(float power, uint16_t time_interval, uint8_t print, float* velo, float* dist);

void setOffset(int off);

#endif // SIMULATION_H
