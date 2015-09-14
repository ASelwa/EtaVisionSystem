#include <Servo.h>

Servo escServo;

volatile uint32_t speedCount = 0;

void speedInterrupt(void){
  ++speedCount;
  //Serial.println("Hi!");
}

uint32_t pauseTimer;

void setup() {
  // put your setup code here, to run once:
  pinMode(2, OUTPUT);
  pinMode(3, INPUT);
  
  //pinMode(4, OUTPUT);

  Serial.begin(115200);
  
  escServo.attach(2);
  
  attachInterrupt(0, speedInterrupt, RISING);
  pauseTimer = millis() + 500;
}


int servoUs;
void loop() {
  // put your main code here, to run repeatedly: 
  
  Serial.println(servoUs);
  Serial.print("  ");
  Serial.println((120.0*speedCount)/6.0);
  speedCount = 0;
  while (millis() < pauseTimer){
    servoUs = map(analogRead(0), 0, 1024, 1000, 2000);
    escServo.writeMicroseconds(servoUs);
  }
  pauseTimer += 500;
  //Serial.println(millis());
}
