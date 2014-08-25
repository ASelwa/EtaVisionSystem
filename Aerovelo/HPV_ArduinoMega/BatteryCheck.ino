// Can handle up to 15.818 volts
const float R1 = 326000;
const float R2 = 22000;
// determine by voltage divider resistors, see text
const float resistorFactor = 1023.0 * (R2/(R1 + R2));  // 1023*0.348
const int batteryPin = 0;         // +V from battery is connected to analog pin 2

float potential() {
  int val = analogRead(batteryPin);  // read the value from the sensor
  float volts = 1.1*(val / resistorFactor); // calculate the ratio
  
  return volts;
}

float getBatteryLevel() {
  return potential();
}
