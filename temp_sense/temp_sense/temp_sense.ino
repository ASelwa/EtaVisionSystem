#include <math.h>

void setup() {
  Serial.begin(9600);
  analogReference(INTERNAL2V56);
}


void loop() {
  readTemp(A0);
  delay(500);
}



void readTemp (int aPin) {
 
 // Input voltage for temperature block is 3.3V
 // Configure arduino mega to use 2.56V internal reference 
 // lowest temperature reading possible is -4C (when temperature block outputs 2.56V on terminal 
 
 float res1 = 10000;
 float refVolt = 3.3;
 float B = 3435;
 float refTemp = 25;
 
 float Vin = analogRead(aPin)*refVolt/1024;
 float res2 = res1*(1/(refVolt/Vin - 1));
 float temperature = 1/((log(res2/res1)/B) + (1/(refTemp+273.15))) - 273.15;
 
 Serial.print("Vin = ");Serial.print(Vin);Serial.print(" V");
 Serial.print("\tR2 = ");Serial.print(res2);Serial.print(" ohms");
 Serial.print("\tTemp = ");Serial.print(temperature);Serial.println(" C");
 
}



