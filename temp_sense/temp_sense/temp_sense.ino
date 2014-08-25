#include <math.h>

void setup() {
  Serial.begin(9600);
  analogReference(DEFAULT);
}


void loop() {
  readTemp(A0, 5);
  delay(500);
}

void readTemp (int aPin, float refVolt) {
 
 int res1 = 11000;
 float Vin = analogRead(aPin)*refVolt/1024;
 float res2 = res1*(1/(refVolt/Vin));
 //res = 33880*exp(-0.036*temp);
 //float temp = log(res2/32379)/(-0.036);
 
 float steinhart;
 steinhart = res2 / 10000;     // (R/Ro)
 steinhart = log(steinhart);                  // ln(R/Ro)
 steinhart /= 3435;                   // 1/B * ln(R/Ro)
 steinhart += 1.0 / (25 + 273.15); // + (1/To)
 steinhart = 1.0 / steinhart;                 // Invert
 steinhart -= 273.15;                         // convert to C
 
 Serial.print("Vin = ");Serial.print(Vin);Serial.print(" V");
 Serial.print("\tR2 = ");Serial.print(res2);Serial.print(" ohms");
 Serial.print("\tTemp = ");Serial.print(steinhart);Serial.println(" C");
 
}
