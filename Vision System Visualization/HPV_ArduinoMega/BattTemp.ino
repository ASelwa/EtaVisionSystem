// Can handle up to 15.818 volts with the breadboard values
const float R1 = 330000; // 326000 measured on breadboard, 267000 measured after soldering
const float R2 = 21700; // 22000 measured on breadboard, 20400 measured after soldering
const float resistorFactor = (R2/(R1 + R2));  // 1023*0.348
const int BATTERY_PIN = 0;         // +V from battery is connected to analog pin 0
const int TEMPERATURE_PIN = A2;

float potential() {
  
  int val = analogRead(BATTERY_PIN);  // read the value from the sensor
  //float volts = 2.56*(val / resistorFactor); // calculate the ratio
  
//  Serial.print("analogRead ");
//  Serial.println(val);
//  
//  Serial.print("readVcc ");
//  Serial.println(readVcc());
  
  float volts = val / 1023.0 * readVcc() / 1000.0 / resistorFactor;

//  Serial.print("Volts ");
//  Serial.println(volts);

  
  return volts;
}

float getBatteryLevel() {
  return potential();
}


long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
 
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  long result = (high<<8) | low;
  //result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  
  result = 1.1*9.8/9.5*1023*1000 / result; 
  
  return result; // Vcc in millivolts
}


/*
 * Low battery only if value read is consistently lower than threshold (at least 5 readings in a row)
 */
bool lowBattery(float potential) {
  static int lastPotentials[5] = {9999, 9999, 9999, 9999, 9999}; // Placeholder values
  static int index = 0;
  static bool lowFlag = false;
  bool low = true;
  
  /* PRINTING
  for (int i = 0; i < 5; i++) {
    Serial.print(lastPotentials[i]); Serial.print("\t");
  } Serial.println();*/
  
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
 float refVolt = readVcc() / 1000.0, Vdd = 3.3;
 float B = 3435;
 float refTemp = 25;
 
 float Vin = analogRead(TEMPERATURE_PIN)*refVolt/1024;
 float res2 = res1*(1/(Vdd/Vin - 1));
 float temperature = 1/((log(res2/res1)/B) + (1/(refTemp+273.15))) - 273.15;
 
 /* PRINTING
 Serial.print("Vin = ");Serial.print(Vin);Serial.print(" V");
 Serial.print("\tR2 = ");Serial.print(res2);Serial.print(" ohms");
 Serial.print("\tTemp = ");Serial.print(temperature);Serial.println(" C");*/
 
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
