/*
 Data Acquisition V2
 
 Created 25 August 2015
 by Alex Selwa
 
 */
 
// Set up infrared
#include <Wire.h>
#include <Adafruit_MLX90614.h>
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

// Set up thermocouple instance with software SPI on any three digital IO pins.
#include <SPI.h>
#include "Adafruit_MAX31855.h" 
#define DO   4
#define CS   5
#define CLK  6
Adafruit_MAX31855 thermocouple(CLK, CS, DO);
 
// Set up O2 and CO2 sensor
#include <math.h>
#include <SoftwareSerial.h>
int const pinO2 = A0; // analog pin used to connect the O2 sensor
const int pinRx = 8;
const int pinTx = 7;
SoftwareSerial K_30_Serial(pinTx,pinRx);
int valMultiplier = 1; 
byte readCO2[] = 
  {0xFE, 0X44, 0X00, 0X08, 0X02, 0X9F, 0X25};
byte response[] = {0,0,0,0,0,0,0};  //create an array to store the response 

// Open Log Setup   
//int resetOpenLog = 2;
#include <SoftwareSerial.h>
//Connect TXO of OpenLog to pin 3, RXI to pin 2
SoftwareSerial OpenLog(3, 2); //Soft RX on 3, Soft TX out on 2
//SoftwareSerial(rxPin, txPin)
char buff[80];
//int resetOpenLog = 1;
//pinMode(resetOpenLog, OUTPUT);

      // ADJUST THESE PARAMETERS    
      int period = 300; // ms      
      float number_of_points = 10; //for initial O2 average

// Flow controlling variables
unsigned long time;
int counter = 0; // For averaging while loop

// Temporary variables used
float O2_temp;
float CO2_temp;
float caliper_temp;
float disk_temp;
float disk_amb_temp;
unsigned long valCO2 = 0;
float meanO2 = 0;
    
    
    
void setup() {
  // Open Log setup
  //pinMode(resetOpenLog, OUTPUT);
  //Serial.begin(115200);
  OpenLog.begin(9600);
  
//  //Reset OpenLog
//  digitalWrite(resetOpenLog, LOW);
//  delay(100);
//  digitalWrite(resetOpenLog, HIGH);
//
//  //Wait for OpenLog to respond with '<' to indicate it is alive and recording to a file
//  while(1) {
//    if(Serial.available())
//      if(Serial.read() == '<') break;
//  }
  
  

  //start the reading of the co2 sensor
  K_30_Serial.begin(9600);
  
  // IR temperature sensor setup
  mlx.begin();
  
//  // Calibrate O2 Sensor
//  for (int i=0; i < number_of_points; i++){
//    meanO2 = meanO2 + analogRead(pinO2);
//    // print out the values to the serial monitor
//    //Serial.println(analogRead(pinO2));
//    delay(400);
//  } 
//  meanO2 = meanO2/number_of_points;
//    
//  sprintf(buff, "Initial average O2: ");
//  ftoa(buff, meanO2, 3);
//  OpenLog.print(buff);
//  OpenLog.println(",");
//  Serial.print(buff);
//  Serial.println(",");
  
  sprintf(buff, "Time [s], O2 [raw], CO2 [ppm], Caliper [deg C], Disk[deg C], Ambient [deg C],");
  OpenLog.println(buff);
  //Serial.println(buff);
}


int valO2 = 0;



// Measurement and Printout Loop
void loop() {
  
  // Reset these "accumulator" variables
  O2_temp = 0;
  CO2_temp = 0;
  caliper_temp = 0;
  disk_temp = 0;
  disk_amb_temp = 0;
  counter = 0;
  time = millis();
  
   

  
  while ((millis() - time) < period) {
 
    
    //sprintf(buff, " Just entered while loop, time: %d", time);
    //OpenLog.println(buff); //regular println works with v2.51 and above
    
    
    // O2
    valO2 = analogRead(pinO2);
    O2_temp = 0; // Commented since sensor removed: O2_temp + valO2; // read the value of the O2 sensor
    
    // CO2  
    sendRequest(readCO2); 
    delay(50);
    if (getValue(response) < 10001) {
      valCO2 = getValue(response);
    }
    else {
      //valCO2 = CO2_average[e.datapoint]/i; // Divide by zero?
      valCO2 = 10000; 
    }
    CO2_temp = CO2_temp + valCO2;   
    
    
    // Caliper temp
    caliper_temp = caliper_temp + thermocouple.readCelsius();
    // Disk temp
    disk_temp = disk_temp + mlx.readObjectTempC();
    // Ambient temp
    disk_amb_temp = disk_amb_temp + mlx.readAmbientTempC();
    // Averaging counter
    counter++;
    //sprintf(buff, "End of while, counter: %d.txt", counter);
    //OpenLog.println(buff); //regular println works with v2.51 and above
  }
  

  //sprintf(buff, "Time+period: %d, ", (int)(time+period));
  //sprintf(buff, "%d, %d, %d, %d, %d, %d,", (int)(time+period), (int)(O2_temp/counter*100), (int)(CO2_temp/counter*100), (int)(caliper_temp/counter*100), (int)(disk_temp/counter*100), (int)(disk_amb_temp/counter*100));
  //OpenLog.println(buff);

  // Write to Open Logger  
  ftoa(buff, (time+period)/1000, 3);
  OpenLog.print(buff);
  OpenLog.print(", ");
  
  //Serial.print(buff);
  //Serial.print(", ");

  ftoa(buff, O2_temp/counter, 3);
  OpenLog.print(buff);
  OpenLog.print(", ");

  //Serial.print(buff);
  //Serial.print(", ");  

  ftoa(buff, CO2_temp/counter, 3);
  OpenLog.print(buff);
  OpenLog.print(", ");
  
  //Serial.print(buff);
  //Serial.print(", ");

  ftoa(buff, caliper_temp/counter, 3);
  OpenLog.print(buff);
  OpenLog.print(", ");
  
  //Serial.print(buff);
  //Serial.print(", ");

  ftoa(buff, disk_temp/counter, 3);
  OpenLog.print(buff);
  OpenLog.print(", ");
  
  //Serial.print(buff);
  //Serial.print(", ");

  ftoa(buff, disk_amb_temp/counter, 3);
  OpenLog.print(buff);
  OpenLog.println(", ");
  
  //Serial.print(buff);
  //Serial.println(", ");
  
  
 }
 
 
/**************************************************************
 *                      FUNCTIONS                             *
 **************************************************************/


char *ftoa(char *a, double f, int precision)
{
 long p[] = {0,10,100,1000,10000,100000,1000000,10000000,100000000};
 
 char *ret = a;
 long heiltal = (long)f;
 itoa(heiltal, a, 10);
 while (*a != '\0') a++;
 *a++ = '.';
 long desimal = abs((long)((f - heiltal) * p[precision]));
 itoa(desimal, a, 10);
 return ret;
}



//function for reading the UART co2 sensor
void sendRequest(byte packet[]) 
{ 
  while(!K_30_Serial.available())  //keep sending request until we start to get a response 
  { 
    K_30_Serial.write(readCO2,7); 
    delay(25); 
  } 
   
  int timeout=0;  //set a timeout counter 
  while(K_30_Serial.available() < 7 )    //Wait to get a 7 byte response 
  { 
    timeout++;   
    if(timeout > 10)    //if it takes to long there was probably an error 
      { 
        while(K_30_Serial.available())  //flush whatever we have 
          K_30_Serial.read(); 
           
          timeout = 0;   
          break;                        //exit and try again 
      } 
      delay(25); 
  } 
   
  for (int i=0; i < 7; i++) 
  { 
    response[i] = K_30_Serial.read(); 
  }   
} 

unsigned long getValue(byte packet[]) 
{ 
    int high = packet[3];                        //high byte for value is 4th byte in packet in the packet 
    int low = packet[4];                         //low byte for value is 5th byte in the packet 

    unsigned long val = high*256 + low;                //Combine high byte and low byte with this formula to get value 
    return val* valMultiplier; 
} 
