#ifndef SIMULATION_H
#define SIMULATION_H

float elevations[19][2] = {
	{0, 1470.75144}, 
	{182.88, 1468.8312},
	{563.88, 1463.25336},
	{792.48, 1461.42456},
	{960.12, 1460.72352},
	{1310.64, 1458.28512},
	{1630.68, 1456.60872},
	{1950.72, 1454.68848},
	{2240.28, 1453.56072},
	{2697.48, 1450.45176},
	{2910.84, 1449.35448},
	{3215.64, 1447.22088},
	{4114.8, 1439.84472},
	{4465.32, 1436.5224},
	{5760.72, 1427.25648},
	{6141.72, 1424.20848},
	{7025.64, 1418.6916},
	{7467.6, 1416.80184},
	{8046.72, 1412.93088}
};

float getElevation(float distance);

uint16_t cB(const uint8_t x, const uint8_t y);

void readPowerMeter(uint8_t *pwrRx, uint8_t print, uint16_t *time_interval, float* power, float* cadence_out, bool* coast);

void simulate(float power, uint16_t time_interval, uint8_t print, float* velo, float* dist);

#endif // SIMULATION_H