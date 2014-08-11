/* Arduino Mega Main Program
 * By Oleksiy Ryndin and Sherry Shi
 */

#include <SD.h>
#include <SPI.h>
#include "inttypes.h"
#include "ANT_interface.h"
#include "GPS_UBLOX.h"
#include "slip.h"
#include "simulation.h"


#define _SS 53

// Message IDs
#define ID_CADENCE	1
#define ID_DISTANCE	2
#define ID_DISPLACEMENT 16
#define ID_SPEED	3
#define ID_GPSTIME      4
#define ID_NUMSATS      5
#define ID_FIX          6
#define ID_ALTITUDE	7
#define ID_HEADING	8
#define ID_UTC          9
#define ID_START        10
#define ID_HEART        11
#define ID_PROFNUM      12
#define ID_TSPEED       13
#define ID_PROFNAME     14
#define ID_POWER        15
#define ID_CALIBRATION  17
#define ID_BATTERY      18
#define ID_GPSCOMM      19

#define MAX_PROFILE_NUM   1

char slipBuffer[128]; //SLIP
uint8_t antBuffer[64]; // ANT+
char sdBuffer[128]; //SD
char gpsFilename[32];
char antFilename[32];
uint8_t Hrt;
uint32_t TIME;

int32_t LattitudeStart = 436585166, LongitudeStart = -793934912, AltitudeStart = 117689;
int32_t LattitudePrev, LongitudePrev, AltitudePrev;
int8_t startSet = 0;
uint32_t GPS_totalDistance = 0;

int8_t lastToggle;
int8_t Toggle;
int8_t profileNum = 0;
char profileFilename[32] = {
  0
};
char logFilename[32] = "GPSlog.txt"; // "LGdflt.txt"
char profileName[16];
double coeff[7] = {0};

bool simulation_mode = false;

void setup() {

  // Select serial port.
  // Serial2 is GPS
  Serial.begin(115200);	// COM port
  Serial3.begin(115200); // OSD
  Serial1.begin(9600);	// ANT+
  Serial.println("Program start!");
  //delay(1000);

  // For checking the battery voltage
  analogReference(INTERNAL1V1);
  
  // Set up CTS interrupt.
  //pinMode(2, INPUT);
  //attachInterrupt(0, receiveInterrupt, RISING);

  // Reset ANT+
  pinMode(3, OUTPUT);
  digitalWrite(3, LOW);
  delayMicroseconds(1000);
  digitalWrite(3, HIGH);
  delayMicroseconds(1000);

  pinMode(8, INPUT);
  digitalWrite(8, HIGH);


  // Initialize GPS
  GPS.Init();


  // Set up SD
  sd_Init();

  // Figure out SD card log iteration.

  //for (int i = 0; i < 16; ++i){
  //  sprintf(gpsFilename, "gpslog%02d.txt", i);
  //  if (!SD.exists(gpsFilename)){
  //    sprintf(antFilename, "antlog%02d.txt", i);
  //    break;
  //  }
  //}

  // Crank Torque Frequency
  ANT_SetupChannel(antBuffer, 0, 11, 0, 8182, 12131);
  // Heart rate channel.
  //ANT_SetupChannel(antBuffer, 1, 0x78, 1, 8086); // 8070
  ANT_SetupChannel(antBuffer, 1, 120, 0, 16140, 45063);

  Serial.println("ANT+ setup complete.");

  TIME = millis() + PERIOD;
  
  coeff[0] = 0;
  coeff[1] = -0.3705;
  coeff[2] = 4.104;
  coeff[3] = -16.399;
  coeff[4] = 17.941;
  coeff[5] = -121.1;
  coeff[6] = 3557.8;

  //calibrate();

  return;
}

int8_t START = 0;

//void loopTestPower() {
//  float power = 65000.5678;
//
//  *((uint8_t*)slipBuffer + 0) = ID_POWER;
//  *((uint16_t*)(slipBuffer + 1 + 0)) = power;
//  //*((uint8_t*)slipBuffer + 1 + 2) = cadence;
//  *((uint8_t*)slipBuffer + 1 + 4) = 1;
//
//  for (uint8_t counter = 0; counter < 6; counter++) {
//    Serial.print(slipBuffer[counter], HEX);
//    Serial.print(" ");
//  }
//  Serial.println();
//
//  SlipPacketSend(4, (char*)slipBuffer, &Serial3);
//  delay(10000);
//}
//
//void loopTestCadence() {
//  static uint16_t cadenceTime = 1000;
//
//  *((uint8_t*)slipBuffer + 0) = ID_CADENCE;
//  *((uint16_t*)(slipBuffer + 1 + 0)) = cadenceTime; //cadenceTime - cadenceTimePrev;
//  *((uint8_t*)slipBuffer + 1 + 2) = 2; //cadenceRev - cadenceRevPrev;
//  *((uint8_t*)slipBuffer + 1 + 4) = 1;
//  //Serial.println("Cadence:");
//  //Serial.println(*((uint16_t*)slipBuffer + 1 + 0));
//  //Serial.println(*((uint8_t*)slipBuffer + 1 + 2));
//
//  for (int i = 0; i < 4; i++) {
//    Serial.print(i);
//    Serial.print(": ");
//    Serial.println((uint8_t) slipBuffer[i]);
//  }
//
//  //SlipPacketSend(uint8_t len, char *slipBuffer, HardwareSerial *serial);
//  SlipPacketSend(4, (char*)slipBuffer, &Serial3);
//
//  //cadenceTime += 100;
//  delay(1000);
//}

void calibrate() {
  bool calibrated = false;
  int8_t m;
  uint8_t i = 0; // Index for cal_values
  int16_t cal_values[6];
  int16_t calibrationValue;

  Serial.println("Begin calibration... Waiting for calibration messages.");
  *((uint8_t*)slipBuffer + 0) = ID_CALIBRATION;
  *((uint8_t*)(slipBuffer + 1 + 0)) = 1; // Waiting for first calibration message
  *((int16_t*)(slipBuffer + 1 + 1)) = 0; // Calibration value
  *((uint8_t*)slipBuffer + 1 + 3) = 0;
  SlipPacketSend(4, (char*)slipBuffer, &Serial3);

  while (i < 6) {
    if (Serial1.available() && (m = receiveANT(antBuffer)) == 9 && antBuffer[1] == 0x4E && antBuffer[4] == 0x10) {
      cal_values[i++] = antBuffer[9] * 256 + antBuffer[10];

      *((uint8_t*)slipBuffer + 0) = ID_CALIBRATION;
      *((uint8_t*)(slipBuffer + 1 + 0)) = 2; // Waiting for subsequent calibration messages
      *((int16_t*)(slipBuffer + 1 + 1)) = 0; // Calibration value
      *((uint8_t*)slipBuffer + 1 + 3) = 0;
      SlipPacketSend(4, (char*)slipBuffer, &Serial3);
    }
  }

  calibrationValue = average(cal_values, 6);
  setOffset(calibrationValue);

  Serial.print("Calibration complete! Offset is: ");
  Serial.print(calibrationValue);
  Serial.println(" Hz");

  *((uint8_t*)slipBuffer + 0) = ID_CALIBRATION;
  *((uint8_t*)(slipBuffer + 1 + 0)) = 3; // Done calibrating
  *((int16_t*)(slipBuffer + 1 + 1)) = calibrationValue;
  *((uint8_t*)slipBuffer + 1 + 3) = 0;
  SlipPacketSend(4, (char*)slipBuffer, &Serial3);

  delay(2000);

  *((uint8_t*)slipBuffer + 0) = ID_CALIBRATION;
  *((uint8_t*)(slipBuffer + 1 + 0)) = 0; // Clear message
  *((int16_t*)(slipBuffer + 1 + 1)) = calibrationValue;
  *((uint8_t*)slipBuffer + 1 + 3) = 0;
  SlipPacketSend(4, (char*)slipBuffer, &Serial3);
}

float power, cadence, velocity, distance = 0;
bool coast = false;
uint16_t time_int = 0;
uint16_t t2 = 0;
uint32_t lastGPSUpdate = millis();
bool GPSLost = false;
void loop() { // Original loop
  int8_t slipLen;
  //oscIn.packetAvailable(); //SLIP

  uint8_t temp;
  int8_t i, m;

  // put main code here, to run repeatedly:
  while (TIME > millis()) {
    START = !digitalRead(8);

    slipLen = SlipReceive(slipBuffer, &Serial);
    if (slipLen > 0) {
      slipBuffer[slipLen] = 0;
      Serial.println(slipBuffer);
      if (slipBuffer[0] == '!') START = 1;
      else if (slipBuffer[0] == '*') START = 0;
    }

    // Read ANT+ Data
    if (Serial1.available()) {
      //temp = Serial1.read();
      //Serial.print(temp, HEX);
      //Serial.print(' ');
      if ((m = receiveANT(antBuffer)) > 0) {
        if (m > 64)
          Serial.print("\n\nANT Overflow\n\n");
        /*Serial.print("ANT+ Packet Received: ");
        for (i = 0; i < m + 3; ++i) {
          Serial.print(antBuffer[i], HEX);
          Serial.print(' ');
        } Serial.print('\n');*/

        if (m == 9) {
          switch (antBuffer[2]) { // Channel
            case 0: // Power meter
              readPowerMeter(antBuffer, 0, &time_int, &power, &cadence, &coast);
              if (!coast && power != 0 && time_int != 0) {
                Serial.print("Time interval: ");
                Serial.println(time_int);
                simulate(power, time_int, 0, &velocity, &distance);
                t2 = millis();	// last power meter data message received
              } else if (coast) {
                simulate(0, 2 * (millis() - t2) , 0, &velocity, &distance);
                t2 = millis();
              }

              *((uint8_t*)slipBuffer + 0) = ID_POWER;
              *((uint16_t*)(slipBuffer + 1 + 0)) = power;
              *((uint8_t*)slipBuffer + 1 + 2) = cadence;
              *((uint8_t*)slipBuffer + 1 + 3) = 0;

              /*for (uint8_t counter = 0; counter < 5; i++) {
                Serial.print(slipBuffer[i], HEX);
                Serial.print(" ");
              }
              Serial.println();*/

              SlipPacketSend(4, (char*)slipBuffer, &Serial3);

              break;
              //            case 2: // Speed and cadence. SHOULD NOT HAPPEN
              //              uint16_t cadenceTime, cadenceRev, bikeTime, bikeRev;
              //              static uint16_t cadenceTimePrev, cadenceRevPrev, bikeTimePrev, bikeRevPrev;
              //
              //              cadenceTime = *((uint16_t*)(antBuffer + m + 2 - 8));
              //              cadenceRev = *((uint16_t*)(antBuffer + m + 2 - 6));
              //              bikeTime = *((uint16_t*)(antBuffer + m + 2 - 4));
              //              bikeRev = *((uint16_t*)(antBuffer + m + 2 - 2));
              //
              //              // Send Cadence data if different from previous
              //              if (cadenceRev - cadenceRevPrev) {
              //                *((uint8_t*)slipBuffer + 0) = ID_CADENCE;
              //                *((uint16_t*)(slipBuffer + 1 + 0)) = cadenceTime - cadenceTimePrev;
              //                *((uint8_t*)slipBuffer + 1 + 2) = cadenceRev - cadenceRevPrev;
              //                *((uint8_t*)slipBuffer + 1 + 3) = 0;
              //                //Serial.println("Cadence:");
              //                //Serial.println(*((uint16_t*)slipBuffer + 1 + 0));
              //                //Serial.println(*((uint8_t*)slipBuffer + 1 + 2));
              //
              //
              //                //SlipPacketSend(uint8_t len, char *slipBuffer, HardwareSerial *serial);
              //                SlipPacketSend(4, (char*)slipBuffer, &Serial3);
              //                cadenceTimePrev = cadenceTime;
              //                cadenceRevPrev = cadenceRev;
              //                bikeTimePrev = bikeTime;
              //                bikeRevPrev = bikeRev;
              //              }
              //
              //
              //              *((uint8_t*)slipBuffer + 0) = ID_HEART;
              //              *((uint16_t*)(slipBuffer + 1 + 0)) = antBuffer[3 + 7];
              //              *((uint8_t*)slipBuffer + 1 + 1) = 0;
              //              SlipPacketSend(2, (char*)slipBuffer, &Serial3);
              //
              //              break;
            case 1: // Heart rate.
              Hrt = antBuffer[3 + 7];
              *((uint8_t*)slipBuffer + 0) = ID_HEART;
              *((uint8_t*)(slipBuffer + 1 + 0)) = Hrt;
              *((uint8_t*)slipBuffer + 1 + 1) = 0;
              SlipPacketSend(2, (char*)slipBuffer, &Serial3);
              //Serial.println("Hrt:");
              //Serial.println((uint8_t) slipBuffer[1]);
              break;
          }
          //sd_Write(sdBuffer, antFilename);
        }

      }
    }

    //GPS.PrintErrors = 1;

    //Read GPS Data
    GPS.Read(); // Updates buffer

    if (GPS.NewData) {
      GPS.NewData = 0;
      lastGPSUpdate = millis();
      GPSLost = false;
      
      *((uint8_t*)slipBuffer + 0) = ID_GPSCOMM;
      *((uint8_t*)(slipBuffer + 1 + 0)) = 1; // Received from GPS
      *((uint8_t*)slipBuffer + 1 + 1) = 0;
      SlipPacketSend(2, (char*)slipBuffer, &Serial3);
      Serial.println("GPS update."); Serial.println(*((uint8_t*) (slipBuffer+1)));

      //View GPS data
      //char gps_str[120];
      //sprintf((char*)gps_str, "GPS\r\n    Time: %lu\r\n    Lattitude: %li;\r\n    Longitude: %li;\r\n    Altitude: %li;\r\n\n",
      // 	GPS.GPSTime, GPS.Lattitude, GPS.Longitude, GPS.Altitude);

      //Serial.println((char*)gps_str);

      //Get Distance
      /*
      if (START){

       // Send START signal

       *((uint8_t*)slipBuffer + 0) = ID_START;
       *((uint8_t*)(slipBuffer + 1 + 0)) = 1;
       *((uint8_t*)slipBuffer + 1 + 1) = 0;
       SlipPacketSend(3, (char*)slipBuffer, &Serial3);
        */


      // Track the moving average
      const int numTerms = 3; // Moving average of this many values
      static int32_t latitude[numTerms];
      static int32_t longitude[numTerms];
      static int32_t altitude[numTerms];
      static int index = 0;

      // Comment out after getting SD (toggle) to work
      if (startSet == 0) {
        GPS_setStart();
        Serial.println(startSet);
        Serial.println("GPS start set!");

        // Prepare the moving average arrays
        for (int x = 0; x < numTerms; x++) {
          latitude[x] = GPS.Lattitude;
          longitude[x] = GPS.Longitude;
          altitude[x] = GPS.Altitude;
        }
      }
      //else{

      if (index == numTerms)
        index = 0;

      //Serial.print(longitude[0]);
      //Serial.print("\t");
      //Serial.print(longitude[1]);
      //Serial.print("\t");
      //Serial.println(longitude[2]);

      latitude[index] = GPS.Lattitude;
      longitude[index] = GPS.Longitude;
      altitude[index++] = GPS.Altitude;

      int32_t lat = average(latitude, numTerms);
      int32_t lon = average(longitude, numTerms);
      int32_t alt = average(altitude, numTerms);

      //            Serial.print("Current: ");
      //            Serial.print(lat);
      //            Serial.print("\t");
      //            Serial.print(lon);
      //            Serial.print("\t");
      //            Serial.println(alt);
      //
      //            Serial.print("Last: ");
      //            Serial.print(LattitudePrev);
      //            Serial.print("\t");
      //            Serial.print(LongitudePrev);
      //            Serial.print("\t");
      //            Serial.println(AltitudePrev);
      //
      //            Serial.print("Start: ");
      //            Serial.print(LattitudeStart);
      //            Serial.print("\t");
      //            Serial.print(LongitudeStart);
      //            Serial.print("\t");
      //            Serial.println(AltitudeStart);

      uint32_t displacement = GPS_getDistance(LattitudeStart, LongitudeStart, AltitudeStart, lat, lon, alt);
      uint32_t currDistance = GPS_getDistance(LattitudePrev, LongitudePrev, AltitudePrev, lat, lon, alt);

      //      Serial.print("Current Distance: ");
      //      Serial.print(currDistance);
      //      Serial.print("\tDisplacement: ");
      //      Serial.print(displacement);


      if (currDistance >= 0) {
        GPS_totalDistance += currDistance;

        //        Serial.print("\tCumulative Distance: ");
        //        Serial.println(GPS_totalDistance);

        LattitudePrev = lat;
        LongitudePrev = lon;
        AltitudePrev = alt;

        //        Serial.print("Assigned Prev: ");
        //        Serial.print(LattitudePrev);
        //        Serial.print("\t");
        //        Serial.print(LongitudePrev);
        //        Serial.print("\t");
        //        Serial.println(AltitudePrev);
      }


      //Updates profileNum, profileFilename, logFilename and starting GPS location if the yellow button is pressed
      toggle();
      //Send profileNum through SLIP
      *((uint8_t*)slipBuffer + 0) = ID_PROFNUM;
      *((int8_t*)(slipBuffer + 1 + 0)) = profileNum;
      *((uint8_t*)slipBuffer + 1 + 1) = 0;
      SlipPacketSend(3, (char*)slipBuffer, &Serial3);

      //Send profileName through SLIP
      *((uint8_t*)slipBuffer + 0) = ID_PROFNAME;
      memcpy(((int8_t*)(slipBuffer + 1 + 0)), profileName, 16);
      *((uint8_t*)slipBuffer + 1 + 16) = 0;
      SlipPacketSend(18, (char*)slipBuffer, &Serial3);    
      //Serial.println(profileName);
      //Serial.println(slipBuffer);

      //Serial.print("Profile #: ");
      //Serial.println(profileNum);

      //Get target speed
      int32_t targetSpeed = 0;

      if (simulation_mode) {
        targetSpeed = calcSpeed(distance, coeff);
      } else {
        targetSpeed = calcSpeed(GPS_totalDistance, coeff);
      }
      //Send target speed through SLIP
      *((uint8_t*)slipBuffer + 0) = ID_TSPEED;
      *((int32_t*)(slipBuffer + 1 + 0)) = targetSpeed;
      *((uint8_t*)slipBuffer + 1 + 4) = 0;
      SlipPacketSend(6, (char*)slipBuffer, &Serial3);

      //Serial.print("Target Speed: ");
      //Serial.println(targetSpeed);

      // Send Speed through SLIP
      *((uint8_t*)slipBuffer + 0) = ID_SPEED;
      if (simulation_mode) {
        *((int32_t*)(slipBuffer + 1 + 0)) = velocity * 100; // Will be converted to km / h in OSD_SLIP
      } else {
        *((int32_t*)(slipBuffer + 1 + 0)) = GPS.Ground_Speed;
      }
      *((uint8_t*)slipBuffer + 1 + 4) = 0;
      SlipPacketSend(6, (char*)slipBuffer, &Serial3);

      //Serial.print("Speed: ");
      //Serial.println(GPS.Ground_Speed);

      //Send Distance through SLIP
      *((uint8_t*)slipBuffer + 0) = ID_DISTANCE;
      if (simulation_mode) {
        *((uint32_t*)(slipBuffer + 1 + 0)) = distance * 1000;
      } else {
        *((uint32_t*)(slipBuffer + 1 + 0)) = GPS_totalDistance;
      }
      *((uint8_t*)slipBuffer + 1 + 4) = 0;
      SlipPacketSend(6, (char*)slipBuffer, &Serial3);

      //Serial.print("Distance: ");
      //Serial.println(GPS_totalDistance);

      //Send Displacement through SLIP
      *((uint8_t*)slipBuffer + 0) = ID_DISPLACEMENT;
      if (simulation_mode) {
        *((uint32_t*)(slipBuffer + 1 + 0)) = distance * 1000; // Assume same as distance
      } else {
        *((uint32_t*)(slipBuffer + 1 + 0)) = displacement;
      }
      *((uint8_t*)slipBuffer + 1 + 4) = 0;
      SlipPacketSend(6, (char*)slipBuffer, &Serial3);
      
      // Send battery information
      *((uint8_t*)slipBuffer + 0) = ID_BATTERY;
      *((uint16_t*)(slipBuffer + 1 + 0)) = getBatteryLevel() * 100;
      *((uint8_t*)slipBuffer + 1 + 2) = 0;
      SlipPacketSend(3, (char*)slipBuffer, &Serial3);

      //}
      //}

      /*
      else{
        *((uint8_t*)slipBuffer + 0) = ID_START;
        *((uint8_t*)(slipBuffer + 1 + 0)) = 0;
        *((uint8_t*)slipBuffer + 1 + 1) = 0;
        SlipPacketSend(3, (char*)slipBuffer, &Serial3);
      }
      */

      /*
      // Send GPS data through SLIP
      *((uint8_t*)slipBuffer + 0) = ID_GPSTIME;
      *((uint32_t*)(slipBuffer + 1 + 0)) = GPS.GPSTime;
      *((uint8_t*)slipBuffer + 1 + 4) = 0;
      SlipPacketSend(6, (char*)slipBuffer, &Serial3);
      */

      *((uint8_t*)slipBuffer + 0) = ID_NUMSATS;
      *((uint8_t*)(slipBuffer + 1 + 0)) = GPS.NumSats;
      *((uint8_t*)slipBuffer + 1 + 1) = 0;
      SlipPacketSend(3, (char*)slipBuffer, &Serial3);
      /*
            Serial.print("Error: ");
            Serial.println(GPS.vacc);
            Serial.println(GPS.hacc);

            *((uint8_t*)slipBuffer + 0) = ID_FIX;
            *((uint8_t*)(slipBuffer + 1 + 0)) = GPS.Fix;
            *((uint8_t*)slipBuffer + 1 + 1) = 0;
            SlipPacketSend(3, (char*)slipBuffer, &Serial3);


            // Send Altitude through SLIP
            *((uint8_t*)slipBuffer + 0) = ID_ALTITUDE;
            *((int32_t*)(slipBuffer + 1 + 0)) = GPS.Altitude;
            *((uint8_t*)slipBuffer + 1 + 4) = 0;
            SlipPacketSend(6, (char*)slipBuffer, &Serial3);


            Serial.print("Altitude: ");
            Serial.println(GPS.Altitude);


            *((uint8_t*)slipBuffer + 0) = ID_HEADING;
            *((int32_t*)(slipBuffer + 1 + 0)) = GPS.Ground_Course;
            *((uint8_t*)slipBuffer + 1 + 4) = 0;
            SlipPacketSend(6, (char*)slipBuffer, &Serial3);
            Serial.print("Heading: ");
            Serial.println(GPS.Ground_Course);

            // Send UTC time through SLIP
            *((uint8_t*)slipBuffer + 0) = ID_UTC;
            *((int32_t*)(slipBuffer + 1 + 0)) = *((int32_t*)&(GPS.UTC));

            memcpy(slipBuffer+1, (void*)(&GPS.UTC), 7);

            *((uint8_t*)slipBuffer + 1 + 7) = 0;
            SlipPacketSend(9, (char*)slipBuffer, &Serial3);

            //sprintf((char*)gps_str, "GPS\r\n    Distance: %g;\r\n    Speed: %g;\r\n", \
            //	GPS_totalDistance, GPS_Speed);
            //Serial.print((char*)gps_str);

            //sprintf((char*)sdBuffer, "GPS Starting Position\r\n    Lattitude: %li;\r\n    Longitude: %li;\r\n    Altitude: %li;\r\n", \
            //LattitudeStart, LongitudeStart, AltitudeStart);
           //Serial.println((char*)sdBuffer);
            */

      //Store in SD
      //sprintf((char*)sdBuffer, "GPS\r\n    Time: %lu;\r\n    Lattitude: %li;\r\n    Longitude: %li;\r\n    Altitude: %li;\r\n    Distance: %lli\r\n", \
      //sprintf(sdBuffer, "Test0 %lu %li %li %li %li Test1\r\nTest2", GPS.GPSTime, GPS.Lattitude, GPS.Longitude, GPS.Altitude, GPS_totalDistance);


      sprintf(sdBuffer, "%li, %li, %li, %li, %li\r\n", lat, lon, alt, GPS.Ground_Speed, GPS_totalDistance);
      //Serial.println(sdBuffer);
      sd_Write(sdBuffer, logFilename);

      /*
      Serial.print((char*)sdBuffer);
       Serial.print(" Distance: ");
       Serial.print(GPS_totalDistance);
       Serial.print("\r\n Speed: ");
       Serial.print(GPS_Speed);
       Serial.print("\r\n\n");
       */

    } else if (GPSLost && millis() - lastGPSUpdate > 1000) {
      Serial.println("No GPS update for last 1000 ms.");
      Serial.print(lastGPSUpdate); Serial.print("\t"); Serial.println(millis());
      lastGPSUpdate = millis();
      
      //Get target speed
      int32_t targetSpeed = 0;
      
      //Updates profileNum, profileFilename, logFilename and starting GPS location if the yellow button is pressed
      toggle();
      //Send profileNum through SLIP
      *((uint8_t*)slipBuffer + 0) = ID_PROFNUM;
      *((int8_t*)(slipBuffer + 1 + 0)) = profileNum;
      *((uint8_t*)slipBuffer + 1 + 1) = 0;
      SlipPacketSend(3, (char*)slipBuffer, &Serial3);

      //Send profileName through SLIP
      *((uint8_t*)slipBuffer + 0) = ID_PROFNAME;
      memcpy(((int8_t*)(slipBuffer + 1 + 0)), profileName, 16);
      *((uint8_t*)slipBuffer + 1 + 16) = 0;
      SlipPacketSend(18, (char*)slipBuffer, &Serial3);
      
      targetSpeed = calcSpeed(distance, coeff);
      //Send target speed through SLIP
      *((uint8_t*)slipBuffer + 0) = ID_TSPEED;
      *((int32_t*)(slipBuffer + 1 + 0)) = targetSpeed;
      *((uint8_t*)slipBuffer + 1 + 4) = 0;
      SlipPacketSend(6, (char*)slipBuffer, &Serial3);

      // Send Speed through SLIP
      *((uint8_t*)slipBuffer + 0) = ID_SPEED;
      *((int32_t*)(slipBuffer + 1 + 0)) = velocity * 100; // Will be converted to km / h in OSD_SLIP
      *((uint8_t*)slipBuffer + 1 + 4) = 0;
      SlipPacketSend(6, (char*)slipBuffer, &Serial3);

      //Send Distance through SLIP
      *((uint8_t*)slipBuffer + 0) = ID_DISTANCE;
      *((uint32_t*)(slipBuffer + 1 + 0)) = distance * 1000;
      *((uint8_t*)slipBuffer + 1 + 4) = 0;
      SlipPacketSend(6, (char*)slipBuffer, &Serial3);

      //Send Displacement through SLIP
      *((uint8_t*)slipBuffer + 0) = ID_DISPLACEMENT;
      *((uint32_t*)(slipBuffer + 1 + 0)) = distance * 1000; // Assume same as distance
      *((uint8_t*)slipBuffer + 1 + 4) = 0;
      SlipPacketSend(6, (char*)slipBuffer, &Serial3);
      
      // Send battery information
      *((uint8_t*)slipBuffer + 0) = ID_BATTERY;
      *((uint16_t*)(slipBuffer + 1 + 0)) = getBatteryLevel() * 100;
      *((uint8_t*)slipBuffer + 1 + 2) = 0;
      SlipPacketSend(3, (char*)slipBuffer, &Serial3);
      
    } else if (millis() - lastGPSUpdate > 5000) {
      GPSLost = true;
      *((uint8_t*)slipBuffer + 0) = ID_GPSCOMM;
      *((uint8_t*)(slipBuffer + 1 + 0)) = 0; // No RX from GPS
      *((uint8_t*)slipBuffer + 1 + 1) = 0;
      SlipPacketSend(2, (char*)slipBuffer, &Serial3);
      Serial.println("No message for 5 seconds.");
    }
  }
  TIME += PERIOD;
}

int32_t average(int32_t *beg, const int len) {
  int32_t total = 0;

  for (int i = 0; i < len; ++i) {
    total += *(beg + i) / len;
  }

  return total;
}

int16_t average(int16_t *beg, const int len) {
  float total = 0;

  for (int i = 0; i < len; ++i) {
    total += *(beg + i);
  }

  total /= len;

  return (int16_t) (total + 0.5f);
}
