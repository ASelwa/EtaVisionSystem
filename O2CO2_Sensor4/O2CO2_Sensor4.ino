/*
  O2 and C02 Sensor Sketch
 
 Created 14 January 2015
 by Cameron Robertson
 
 Parts required:
 O2 Sensor - Figaro SK-25F
 CO2 Sensor - Figaro TGS 4161
 */
 
// Set up infrared
#include <Wire.h>
#include <Adafruit_MLX90614.h>
Adafruit_MLX90614 mlx = Adafruit_MLX90614();



// Set up thermocouple
#include <SPI.h>
#include "Adafruit_MAX31855.h" 
// Example creating a thermocouple instance with software SPI on any three
// digital IO pins.
#define DO   3
#define CS   4
#define CLK  5
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

////Declare variables used in loops
float meanO2 = 0;
float gainO2 = 100;
float meanVoltO2 = 0;
float baseVoltO2 = 0;
int valO2 = 0;
float valVoltO2 = 0;
float valPercentO2 = 0;
float CO2ppm = 0;
float number_of_points = 10; //for initial average
int threshold = 10;
float temp_value = 0;

// ADJUST THESE PARAMETERS 

      // How long to collect data for
      //int n_seconds_saved = 3600;
      int n_average = 140; // 90 gives roughly 0.33 Hz sampling frequency
      // 23 gave 60 points in 196s = 0.306 Hz
      // 3 gave 270 points in 171s = 1.579 Hz
      // 50 gives...270 points in 491s
      // 100 gives 270 points in 990s
      // 180 gives 270 points in 1794ish seconds for 2 variable
      #define SAVE_O2 true
      #define SAVE_CO2 true
      #define SAVE_CALIPER true
      #define SAVE_DISK true
      #define SAVE_DISK_AMB false
      #define SAVE_TIME false
      // DATA_SIZE = 660 / # variables
      #define DATA_SIZE 135
      int data_size = DATA_SIZE;
      float O2_temp;
      float CO2_temp;
      float caliper_temp;
      float disk_temp;
      float disk_amb_temp;
      
      // Set the size as 1 if "SAVE" flags are false 
      int O2_average[DATA_SIZE];
      int CO2_average[DATA_SIZE];
      unsigned int caliper_average[DATA_SIZE];
      unsigned int disk_average[DATA_SIZE];
      unsigned int disk_amb_average[1];
      int time[1];
      
      int datapoint = 0;
      unsigned long timeOffset;
      unsigned long totalTime;
      
      unsigned long valCO2 = 0;

      // For printing data as it's collected
      #define PRINT_FLAG false


void setup(){
  Serial.begin(115200);
  K_30_Serial.begin(9600);//start the reading of the co2 sensor

  // Calibrate O2 Sensor
  if (SAVE_O2) {
    for (int i=0; i < number_of_points; i++){
      meanO2 = meanO2 + analogRead(pinO2);
      //
      // print out the values to the serial monitor
      Serial.print("Calib: ");
      Serial.print(i);
      Serial.print("  ");
      Serial.print("O2: ");
      Serial.println(analogRead(pinO2));
      delay(1000);
    //
    } 
    meanO2 = meanO2/number_of_points;
    
    Serial.print("Final meanO2: ");
    Serial.println(meanO2);
  }
  
  // IR temperature sensor setup
  mlx.begin();

  timeOffset = millis(); // To track the starting time  
}

// Measurement and Printout Loop
void loop(){
  
  // Reset these "accumulator" variables
  O2_temp = 0;
  CO2_temp = 0;
  caliper_temp = 0;
  disk_temp = 0;
  disk_amb_temp = 0;
  
  for (int i = 0; i<n_average; i++){
    
    if (SAVE_O2) {
      valO2 = analogRead(pinO2); // read the value of the O2 sensor
      O2_temp = O2_temp + valO2;
    }
    if (SAVE_CO2) {
        sendRequest(readCO2); 
      if (getValue(response) < 10001)
      {
        valCO2 = getValue(response); 
      }
      else
      {
        valCO2 = 10000; 
        //valCO2 = CO2_average[datapoint]/i; // Divide by zero?
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
      
    /*if (PRINT_FLAG) {
      Serial.print(" , O2 = ");
      Serial.println(valO2);
      Serial.print(" , CO2 ppm = ");
      Serial.println(valCO2);
      Serial.print("Time (ms): ");
      Serial.println(millis()-timeOffset);
    }*/
  }
  
  // Store the data in the array
  if (SAVE_O2) {
    O2_average[datapoint] = O2_temp/n_average;
  }
  if (SAVE_CO2) {
    CO2_average[datapoint] = CO2_temp/n_average;
  }
  if (SAVE_CALIPER) {
    caliper_average[datapoint] = caliper_temp/n_average*100;
  }
  if (SAVE_DISK) {
    disk_average[datapoint] = disk_temp/n_average*100;
  }
  if (SAVE_DISK_AMB) {
    disk_amb_average[datapoint] = disk_amb_temp/n_average*100;
  }  
  if (SAVE_TIME) {
    time[datapoint] = (millis()-timeOffset)/1000;
  }

  // Log to Serial Monitor  
  if (Serial.available() > 0) { 
    totalTime = millis()-timeOffset; // Save the end time
    print_function(true);
  }
  datapoint++;
  
  // STAY IN THIS PRINTING LOOP FOREVER WHEN RUN OUT ROOM IN THE DATA ARRAYS
  if (datapoint > data_size-1) {
    datapoint--;
    totalTime = millis()-timeOffset; // Save the end time
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
      Serial.println("START");
      if (SAVE_O2) {
        Serial.print("Initial O2: ");
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
        if (SAVE_TIME) {
          Serial.print(",  Time: ");
          Serial.print(time[j]);
        }
        Serial.println(" ");
      }
      if (finished) {
          Serial.print("s: ");
          Serial.println(totalTime/1000);
        }
      Serial.println("END");
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

