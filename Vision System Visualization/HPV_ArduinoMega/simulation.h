#ifndef SIMULATION_H
#define SIMULATION_H

#include <math.h>
#include <inttypes.h>
#include "Arduino.h"
#include "HPV_ArduinoMega.h" // Only for logging function

float FmultiMap(float val, float * _in, float * _out, uint8_t size);

float powerLookup(float distLocal);

float gearLookup(float v);

float getElevation(float distance);

uint16_t cB(const uint8_t x, const uint8_t y);

void readPowerMeter(uint8_t *pwrRx, uint8_t print, uint16_t *time_interval, float* power, float* cadence_out, bool* coast);

void simulate(float power, uint16_t time_interval, uint8_t print, float* velo, float* dist);

void setOffset(int off);

float pwrAvg(float pwrIn);

float tenSecPower(float pwrIn);

#endif // SIMULATION_H
