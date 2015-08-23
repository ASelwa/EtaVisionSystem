/*
  O2 and C02 Sensor Sketch
 
 Created 14 January 2015
 by Cameron Robertson
 
 Parts required:
 O2 Sensor - Figaro SK-25F
 CO2 Sensor - Figaro TGS 4161
 
 */

// include the math library
#include <math.h>
#include <SoftwareSerial.h>
//
int const pinO2 = A0; // analog pin used to connect the O2 sensor

const int pinRx = 8;
const int pinTx = 7;
SoftwareSerial K_30_Serial(pinTx,pinRx);
unsigned char flg_get = 0;              // if get sensor data
int valMultiplier = 1; 

byte readCO2[] = 
  {0xFE, 0X44, 0X00, 0X08, 0X02, 0X9F, 0X25};
byte response[] = {0,0,0,0,0,0,0};  //create an array to store the response 

//function for reading the UART co2 sensor
void sendRequest(byte packet[]) 
{ 
  while(!K_30_Serial.available())  //keep sending request until we start to get a response 
  { 
    K_30_Serial.write(readCO2,7); 
    delay(25); 
  } 
   
  int timeout=0;  //set a timeoute counter 
  while(K_30_Serial.available() < 7 )    //Wait to get a 7 byte response 
  { 
    timeout++;   
    if(timeout > 10)    //if it takes to long there was probably an error 
      { 
        while(K_30_Serial.available())  //flush whatever we have 
          K_30_Serial.read(); 
           
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
//float change = 100;
int seconds_per_datapoint = 5;
int n_average = 18*seconds_per_datapoint;//this should give us 30 seconds per datapoint 
float O2_average[100];
float CO2_average[100];
int datapoint = 0;

void setup(){
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);
  K_30_Serial.begin(9600);//start the reading of the co2 sensor
//  while (change > threshold){// wait for the O2 sensor to reach steady state value
//    temp_value = analogRead(pinO2);
//    delay(1000);
//    change = analogRead(pinO2) - temp_value;
//    Serial.print("waiting for O2 sensor...");
//    Serial.println(change);
//  }
  //  // Calibrate O2 Sensor 
  //
  for (int i=0; i < number_of_points; i++){
    meanO2 = meanO2 + analogRead(pinO2);
    //
    // print out the values to the serial monitor
    Serial.print("Calibration Point: ");
    Serial.print(i);
    Serial.print("  ");
    Serial.print("meanO2: ");
    Serial.println(analogRead(pinO2));
    delay(1000);
  //
  } 
  meanO2 = meanO2/number_of_points;
  
  Serial.print("Final meanO2: ");
  Serial.println(meanO2);
  //
  // Assume typical atmospheric concentration of O2 (20.95%), determine 0% O2 Voltage
//  meanVoltO2 = map(meanO2, 0, 1023, 0, 500);
//  baseVoltO2 = (meanVoltO2/100/gainO2);//-0.0067319;
  Serial.print("Value outside O2: ");
  Serial.println(meanO2);
}


// Measurement and Printout Loop
void loop(){
  O2_average[datapoint] = 0;
  CO2_average[datapoint] = 0;
  //  
  for (int i = 0; i<n_average; i++){
    valO2 = analogRead(pinO2); // read the value of the O2 sensor
    valPercentO2 = valO2/meanO2*100;
    delay(5);
    // read the value of the CO2 sensor. this is done directly using the uart
    Serial.println("********************************************************");// DO NOT COMMENT OUT
    Serial.println(); // THIS LINE AND THE ONE ABOVE SEEM TO BE CRITICAL TO GET THE CO2 SENSOR WORKINg...
    flg_get = 0;
    int gas, temp;
    sendRequest(readCO2); 
    unsigned long valCO2 = 0;
    if (getValue(response) < 10001)
    {
      valCO2 = getValue(response); 
    }
    else
    {
      valCO2 = CO2_average[datapoint]/i;
    }
    Serial.print(" , O2 percentage = ");
    Serial.println(valPercentO2);
  Serial.print(" , CO2 ppm = ");
    Serial.println(valCO2);
    O2_average[datapoint] = O2_average[datapoint] + valPercentO2;
    CO2_average[datapoint] = CO2_average[datapoint] + valCO2;
    delay(10);//got to figure out the actual timing here so we can match the duration of the run to the data
  } 
  O2_average[datapoint] = O2_average[datapoint]/n_average;
  CO2_average[datapoint] = CO2_average[datapoint]/n_average;
  Serial.println("***************** DATA ************");
  Serial.print("initial O2 reading : ");
  Serial.println(meanO2);  
  for (int j = 0; j <datapoint; j++) // print all the datapoints so far
  {   
     Serial.print("O2 Level [%]: ");
     Serial.print(O2_average[j]);
     Serial.print(", CO2 Level [ppm]: ");
     Serial.println(CO2_average[j]);
  }
  Serial.println("*********** END DATA ************");
  datapoint++;
  //
}


