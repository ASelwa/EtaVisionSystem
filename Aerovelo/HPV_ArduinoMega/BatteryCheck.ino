// Can handle up to 15.818 volts with the breadboard values
const float R1 = 326000; // 326000 measured on breadboard, 267000 measured after soldering
const float R2 = 22000; // 22000 measured on breadboard, 20400 measured after soldering
const float resistorFactor = 1023.0 * (R2/(R1 + R2));  // 1023*0.348
const int BATTERY_PIN = 0;         // +V from battery is connected to analog pin 0
const int TEMPERATURE_PIN = A2;

float potential() {
  int val = analogRead(BATTERY_PIN);  // read the value from the sensor
  float volts = 2.56*(val / resistorFactor); // calculate the ratio
  
  return volts;
}

float getBatteryLevel() {
  return potential();
}

/*
 * Low battery only if value read is consistently lower than threshold (at least 5 readings in a row)
 */
bool lowBattery(float potential) {
  static int lastPotentials[5] = {9999, 9999, 9999, 9999, 9999}; // Placeholder values
  static int index = 0;
  static bool lowFlag = false;
  bool low = true;
  
  for (int i = 0; i < 5; i++) {
    Serial.print(lastPotentials[i]); Serial.print("\t");
  } Serial.println();
  
  if (!lowFlag) { // Low battery stays low
    lastPotentials[index++] = potential * 100;
    
    if (index >= 5)
      index = 0;
    
    for (int i = 0; i < 5; i++) {
      if (lastPotentials[i] > 963 - 80) // -0.8 = offset from multimeter reading
        low = false;
    }
    
    if (low) // Once set, can't be reversed
      lowFlag = true;
    
    return low;
  } else
      return true;
}

/*
 * Measure the current temperature
 */
float readTemp () {
 
 // Input voltage for temperature block is 3.3V
 // Configure arduino mega to use 2.56V internal reference 
 // lowest temperature reading possible is -4C (when temperature block outputs 2.56V on terminal 
 
 float res1 = 10000;
 float refVolt = 2.56, Vdd = 3.3;
 float B = 3435;
 float refTemp = 25;
 
 float Vin = analogRead(TEMPERATURE_PIN)*refVolt/1024;
 float res2 = res1*(1/(Vdd/Vin - 1));
 float temperature = 1/((log(res2/res1)/B) + (1/(refTemp+273.15))) - 273.15;
 
 Serial.print("Vin = ");Serial.print(Vin);Serial.print(" V");
 Serial.print("\tR2 = ");Serial.print(res2);Serial.print(" ohms");
 Serial.print("\tTemp = ");Serial.print(temperature);Serial.println(" C");
 
 return temperature;
}

/*
 * Check if the temperature is high
 */
bool highTemp(float temp) {
  if (temp > 55)
    return true;
    
  return false;
}
