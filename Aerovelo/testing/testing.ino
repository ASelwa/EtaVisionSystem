/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.
 
  This example code is in the public domain.
 */
 
// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
int led = 13;

// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);     
  
  
  
  Serial.begin(115200);
  
  

  
  
}


  // For acceleration calc
int32_t prevSpeed = 0;
int32_t currSpeed = 100;
uint32_t prevGPSTime = 0;
uint32_t currGPSTime = 0;
int32_t accel = 0;

// the loop routine runs over and over again forever:
void loop() {
  
  prevGPSTime = (millis());
  
    delay(500);
       
 currGPSTime = (millis());
 
 accel = (currSpeed - prevSpeed)*1000 / (currGPSTime-prevGPSTime);
 
 float sender = float(accel);
 
  Serial.println(prevGPSTime);
  
    Serial.println(currGPSTime);
    Serial.print("sender: ");    
    Serial.println(sender/100);
    
      Serial.println("end");
  

}
