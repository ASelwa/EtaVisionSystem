/*
 Data Acquisition V1
 
 Created 23 August 2015
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

// Declare variables used in loops
float meanO2 = 0;
int valO2 = 0;
float number_of_points = 2; //for initial average

// ADJUST THESE PARAMETERS 
      #define SAVE_O2 true
      #define SAVE_CO2 true
      #define SAVE_CALIPER false
      #define SAVE_DISK false
      #define SAVE_DISK_AMB false
      
      #define DATA_SIZE 250 // DATA_SIZE = 660ish / # variables enabled above

      int n_seconds_saved = 210; // How long to collect data for
      
      // Main data arrays: set the size as 1 if "SAVE" flags are false 
      int O2_average[DATA_SIZE];
      int CO2_average[DATA_SIZE];
      unsigned int caliper_average[1];
      unsigned int disk_average[1];
      unsigned int disk_amb_average[1];  
      
      // Flow controlling variables
      unsigned long time;
      int period = n_seconds_saved * 1000 / DATA_SIZE; // ms      
      int counter = 0; // For averaging while loop
      int datapoint = 0; // For indexing data arrays
      
      // Temporary variables used
      float O2_temp;
      float CO2_temp;
      float caliper_temp;
      float disk_temp;
      float disk_amb_temp;
      unsigned long valCO2 = 0;
    
    
    
void setup(){
  Serial.begin(115200);
  K_30_Serial.begin(9600);//start the reading of the co2 sensor
  
  // Calibrate O2 Sensor
  if (SAVE_O2) {
    for (int i=0; i < number_of_points; i++){
      meanO2 = meanO2 + analogRead(pinO2);
      // print out the values to the serial monitor
      Serial.println(analogRead(pinO2));
      delay(1000);
    } 
    meanO2 = meanO2/number_of_points;
    
    Serial.print("Final meanO2: ");
    Serial.println(meanO2);
  }
  
  // IR temperature sensor setup
  mlx.begin();
}

// Measurement and Printout Loop
void loop(){
  
  // Reset these "accumulator" variables
  O2_temp = 0;
  CO2_temp = 0;
  caliper_temp = 0;
  disk_temp = 0;
  disk_amb_temp = 0;
  counter = 0;
  time = millis();
  
  while (millis() - time < period) {
    if (SAVE_O2) {
      valO2 = analogRead(pinO2); // read the value of the O2 sensor
      O2_temp = O2_temp + valO2;
    }
    if (SAVE_CO2) {
      sendRequest(readCO2); 
      if (getValue(response) < 10001) {
        valCO2 = getValue(response);
      }
      else {
        //valCO2 = CO2_average[datapoint]/i; // Divide by zero?
        valCO2 = 10000; 
      }
      CO2_temp = CO2_temp + valCO2;      
    }
    if (SAVE_CALIPER) {
      caliper_temp = caliper_temp + thermocouple.readCelsius();
    }
    if (SAVE_DISK) {
          disk_temp = disk_temp + mlx.readObjectTempC();
    }
    if (SAVE_DISK_AMB) {
          disk_amb_temp = disk_amb_temp + mlx.readAmbientTempC();
    }
    counter++;
  }
  
  // Store the data in the array
  if (SAVE_O2) {
    O2_average[datapoint] = O2_temp/counter;
  }
  if (SAVE_CO2) {
    CO2_average[datapoint] = CO2_temp/counter;
  }
  if (SAVE_CALIPER) {
    caliper_average[datapoint] = caliper_temp/counter*100;
  }
  if (SAVE_DISK) {
    disk_average[datapoint] = disk_temp/counter*100;
  }
  if (SAVE_DISK_AMB) {
    disk_amb_average[datapoint] = disk_amb_temp/counter*100;
  }  

  // Log to Serial Monitor  
  if (Serial.available() > 0) { 
    print_function(true);
  }
  datapoint++;
  
  // STAY IN THIS PRINTING LOOP FOREVER WHEN RUN OUT ROOM IN THE DATA ARRAYS
  if (datapoint > DATA_SIZE-1) {
    datapoint--;
    while (true) {
      print_function(true);
      delay(10000); // It will print the entire log every 10 seconds
    } 
  }    
}


/**************************************************************
 *                      FUNCTIONS                             *
 **************************************************************/
void print_function(bool finished) {
      Serial.println("*");
      if (SAVE_O2) {
        Serial.print("mO2: ");
        Serial.println(meanO2);  
      }
      for (int j = 0; j <datapoint; j++) // print all the datapoints so far
      {   
        Serial.print(j+1); 
        if (SAVE_O2) {
          Serial.print(", O2: ");
          Serial.print(O2_average[j]/meanO2*100);
        }
        if (SAVE_CO2) {       
          Serial.print(", CO2: ");
          Serial.print(CO2_average[j]);
        }
        if (SAVE_CALIPER) {
          Serial.print(", Cal: ");
          Serial.print((float)caliper_average[j]/100);
        }
        if (SAVE_DISK) {
          Serial.print(", Disk: ");
          Serial.print((float)disk_average[j]/100);
        }
        if (SAVE_DISK_AMB) {
          Serial.print(", Ambient: ");
          Serial.print((float)disk_amb_average[j]/100);
        } 
        Serial.println(" ");
      }
      if (finished) {
          Serial.print("s: ");
          Serial.println(time/1000);
        }
      Serial.println("*");
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

