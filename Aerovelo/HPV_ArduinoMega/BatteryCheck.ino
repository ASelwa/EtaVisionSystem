// Can handle up to 15.818 volts
const float R1 = 367000; // 326000
const float R2 = 20400; // 22000
// determine by voltage divider resistors, see text
const float resistorFactor = 1023.0 * (R2/(R1 + R2));  // 1023*0.348
const int BATTERY_PIN = 0;         // +V from battery is connected to analog pin 2
const int TEMPERATURE_PIN = A5;

float potential() {
  int val = analogRead(BATTERY_PIN);  // read the value from the sensor
  float volts = 2.56*(val / resistorFactor); // calculate the ratio
  
  return volts;
}

float getBatteryLevel() {
  return potential();
}

/*
 * Low battery only if value read is consistently lower than threshold
 */
bool lowBattery(float potential) {
  static int lastPotentials[5] = {0};
  static int index = 0;
  bool low = true;
  
  lastPotentials[index++] = potential * 100;
  
  if (index >= 5)
    index = 0;
  
  for (int i = 0; i < 5; i++) {
    if (lastPotentials[index] > 11.25)
      low = false;
  }
  
  return low;
}

float readTemp () {
 
 // Input voltage for temperature block is 3.3V
 // Configure arduino mega to use 2.56V internal reference 
 // lowest temperature reading possible is -4C (when temperature block outputs 2.56V on terminal 
 
 float res1 = 10000;
 float refVolt = 3.3;
 float B = 3435;
 float refTemp = 25;
 
 float Vin = analogRead(TEMPERATURE_PIN)*refVolt/1024;
 float res2 = res1*(1/(refVolt/Vin - 1));
 float temperature = 1/((log(res2/res1)/B) + (1/(refTemp+273.15))) - 273.15;
 
 Serial.print("Vin = ");Serial.print(Vin);Serial.print(" V");
 Serial.print("\tR2 = ");Serial.print(res2);Serial.print(" ohms");
 Serial.print("\tTemp = ");Serial.print(temperature);Serial.println(" C");
 
 return Vin;
}
