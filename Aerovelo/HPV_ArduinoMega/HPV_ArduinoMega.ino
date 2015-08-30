/* Arduino Mega Main Program
 * By Oleksiy Ryndin and Sherry Shi
 */
 
 /*
  *   TODO:
  *    - store default calibration number (507 Hz, found in simulation.cpp also) in one place
  *    - keep only one sd card logging function (the one that takes in a filename) 
  *      (or do something even smarter than the current method)
  *    
  *    Integer overflow on the simulated distance
  */

#include <SD.h>
#include <SPI.h>
#include "inttypes.h"
#include "ANT_interface.h"
#include "GPS_UBLOX.h"
#include "slip.h"
#include "simulation.h"

#include "RunningAverage.h"

const int accelTerms = 3;
int32_t accel[accelTerms];
int indexAccel = 0;


#define PERIOD 1000 // Main loop period
#define SERIAL_PRINT true

#define _SS 53

// Message IDs
#define ID_CADENCE	1
#define ID_DISTANCE	2
#define ID_DISPLACEMENT 16
#define ID_SPEED	3
#define ID_SIM_SPEED    20
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
#define ID_TPOWER       23
#define ID_10S_POWER    24
#define ID_AVG_POWER    25
#define ID_CALIBRATION  17
#define ID_BATTERY      18
#define ID_TEMPERATURE  26
#define ID_GPSCOMM      19
#define ID_SDCOMM       22
#define ID_MODE         21
#define ID_SIMPLEDISPLACEMENT  30
#define ID_BRAKE_MODE 31
#define ID_ACCEL 32

#define COURSE_LENGTH   8045 // Metres
#define POWER_START      300 // Watts
#define POWER_PRE_SPRINT 337 // Watts
#define CALIBRATION_TIME 15000 // Milliseconds (Should be a few seconds less than the beginningPanels time, so the pedal calibration value can be seen)

const uint8_t TOGGLE_PIN = A1;

char slipBuffer[N_SLIP]; //SLIP.h
static uint8_t antBuffer[64]; // ANT+
char sdBuffer[128]; //SD
char gpsFilename[32];
char antFilename[32];
static uint8_t Hrt;
uint32_t TIME;
uint32_t TEMPTIME;

// Default coordinates as bike room (256 McCaul Street)
int32_t LattitudeStart = 436585166, LongitudeStart = -793934912, AltitudeStart = 117689;
int32_t LattitudeFinish = 436585166, LongitudeFinish = -793934912, AltitudeFinish = 117689;

int32_t LattitudePrev, LongitudePrev, AltitudePrev;
int8_t startSet = 0;
uint32_t GPS_totalDistance = 0;

int8_t lastToggle;
int8_t Toggle;
int8_t profileNum = 0;
char profileFilename[32] = {0};
char logFilename[32] = "LogTest.txt"; // "LGdflt.txt"
//char logFilename[32] = "SRMLogDf.txt";
char profileName[16];
double coeff[7] = {0};

// Old boolean for tracking "simulation mode"
bool simulation_mode = false;

// Revised boolean for enabling simulation during a real run
bool SIMULATION = false;


void setup() {

  // Serial2 is GPS
  Serial.begin(115200);	// COM port
  Serial3.begin(115200); // OSD
  Serial1.begin(57600);	// ANT+
  if (SERIAL_PRINT) { Serial.println("Program start!"); }

  // For checking the battery voltage
  analogReference(INTERNAL2V56);

  // Reset ANT+
  pinMode(3, OUTPUT);
  digitalWrite(3, LOW);
  delayMicroseconds(2000);
  digitalWrite(3, HIGH);
  delayMicroseconds(2000);

  // Toggle
  //pinMode(TOGGLE_PIN, INPUT);
  //digitalWrite(TOGGLE_PIN, HIGH);

  // Initialize GPS and SD
  GPS.Init();
  sd_Init();

  // Figure out SD card log iteration.
  for (int i = 1; i < 100000; ++i){
    sprintf(logFilename, "Log%d.csv", i);
    if (!SD.exists(logFilename)){
      //sprintf(SRMlogFilename, "SLg%d.csv", i);
      break;
    }
  }
  //loadFinishCoordinates();

  // Write the log file headings
  sd_Log("Time (hh:mm:ss), Time (ms), Displacement (m), Sim Distance (m), Ground Speed (km/h), Target Speed (km/h), Sim Speed (km/h), Power (W), Cadence (rpm), Latitude (deg), Longitude (deg), Altitude (m), Heart Rate (bpm), Battery (V), Temperature (C),");
  
  // Crank Torque Frequency
  // ANT_SetupChannel(antBuffer, 0, 0, 0, 8182, 0); UNCOMMENT THIS LINE TO SEARCH FOR ANY DEVICE (ONLY SAFE FOR DEBUGGING)
  ANT_SetupChannel(antBuffer, 0, 11, 0, 8182, 12131);
  // Heart rate channel.
  ANT_SetupChannel(antBuffer, 1, 120, 0, 16140, 45063);
  //Serial.println("ANT+ setup complete.");

  // Calibrate pedals
  calibrate();
  
  TIME = millis() + PERIOD;
}

int8_t START = 0;

static float power, cadence, velocity, distance = 0;
float averagePower, power10s, targetPower;
static bool coast = false;
static uint16_t time_int = 0;
static uint16_t t2 = 0;
static uint32_t lastGPSUpdate = millis();
static uint32_t targetSpeed = 0;
static int32_t displacement;
static int32_t simpleDisplacement = 0;
float logDisplacement = 0;
static int32_t lat, lon, alt;
static bool GPSLost = false;
  int counter = 1; // not used?
  
uint16_t temperature;

// For acceleration calc
int32_t prevSpeed = 0;
int32_t currSpeed = 0;
int32_t prevGPSTime = 500;
int32_t currGPSTime = 1000;

int8_t slipLen;


/********************************************************
 *               MAIN PROGRAM  
 ********************************************************/
void loop() {


  uint8_t temp;
  int8_t i, m; 
  
  // Anything outside this loop is executed every PERIOD milliseconds
  while (TIME > millis()) {
    
    //receiveOSD() // Should print messages received from OSD?

    // Read ANT+ Data
    if (Serial1.available()) {
      if ((m = receiveANT(antBuffer)) > 0) {
        
        if (SERIAL_PRINT) { 
          Serial.print("ANT+ Packet Received: ");
          for (i = 0; i < m + 3; ++i) {
            Serial.print(antBuffer[i], HEX);
            Serial.print(' ');
          } Serial.print('\n');
        }

        if (m == 9) {
          switch (antBuffer[2]) { // Channel
            case 0: // Power meter
              readPowerMeter(antBuffer, 0, &time_int, &power, &cadence, &coast);
              
              if (!coast && power != 0 && time_int != 0) {
                simulate(power, time_int, 2, &velocity, &distance);
                t2 = millis();	// last power meter data message received
              } else if (coast) {
                simulate(0, 2 * (millis() - t2) , 0, &velocity, &distance);
                t2 = millis();
              }
              if (power > 0.1) { // Ignore power values of 0
                averagePower = pwrAvg(power);
                power10s = tenSecPower(power);
              }
              
              sendPower();
			  
              logData(); // Write the updated values to the log file (mainly power updates)
              break;

            case 1: // Heart rate.
              Hrt = antBuffer[3 + 7];
              sendHeartRate();
              break;
          }
        }
      }
    }
    
    GPS.Read(); //Read GPS Data (Updates buffer?)
    if (GPS.NewData) {
      GPS.NewData = 0;
      lastGPSUpdate = millis();
      GPSLost = false;
      
      if (SERIAL_PRINT) { Serial.println("GPS received!"); }

      // Variables for moving average
      const int numTerms = 3;
      static int32_t latitude[numTerms];
      static int32_t longitude[numTerms];
      static int32_t altitude[numTerms];
      static int index = 0;      
 
      if (startSet == 0) {
        GPS_setStart();

        // Prepare the moving average arrays
        for (int x = 0; x < numTerms; x++) {
          latitude[x] = GPS.Lattitude;
          longitude[x] = GPS.Longitude;
          altitude[x] = GPS.Altitude;
          accel[x] = 0;      
        }
      }
      
      if (index == numTerms)
        index = 0;
        indexAccel = 0;

      latitude[index] = GPS.Lattitude;
      longitude[index] = GPS.Longitude;
      altitude[index++] = GPS.Altitude;

      lat = average(latitude, numTerms);
      lon = average(longitude, numTerms);
      alt = average(altitude, numTerms);
            
      //displacement = GPS_getDistance(LattitudeFinish, LongitudeFinish, AltitudeFinish, lat, lon, alt);
      //uint32_t currDistance = GPS_getDistance(LattitudePrev, LongitudePrev, AltitudePrev, lat, lon, alt);
      
      simpleDisplacement = GPS_getDistance(LattitudeStart, LongitudeStart, AltitudeStart, lat, lon, alt);
      //logDisplacement = GPS_getDistance(LattitudeStart, LongitudeStart, AltitudeStart, lat, lon, alt);
      //Serial.print(GPS_getDistance(LattitudeStart, LongitudeStart, AltitudeStart, lat, lon, alt));
      
      // PUT INTO BRAKE MODE
      sendBrakeMode(1);
            
      if (simpleDisplacement < 50000) {
        sendBrakeMode(1);        
      }
      if (simpleDisplacement > 1000000) {
        sendBrakeMode(0); 
      }
      
      // Acceleration section
      currSpeed = GPS.Ground_Speed*1000000;
      currGPSTime = millis();
      accel[indexAccel++] = (currSpeed - prevSpeed) / (currGPSTime-prevGPSTime);
      prevSpeed = currSpeed;
      prevGPSTime = currGPSTime; 
      sendAccel();   
      
      if (SERIAL_PRINT) {Serial.print("Accel: "); Serial.println(float(average(accel, numTerms))/100000*105); }
      
      if (SIMULATION) { 
        // Target power based on simulated distance travelled
        targetPower = calcPower(distance, POWER_START, POWER_PRE_SPRINT);
        sendTargetPower(targetPower);
        
        targetSpeed = calcSpeed(COURSE_LENGTH - distance, coeff);
      }
      else {
        targetSpeed = calcSpeed(displacement / 1000.0, coeff);
      }
        
      sendTargetSpeed();

      sendSpeed();
      
      
      
      
      
      
      if (SIMULATION) {
        // Send Simulated Speed through SLIP
        *((uint8_t*)slipBuffer + 0) = ID_SIM_SPEED;
        *((int32_t*)(slipBuffer + 1 + 0)) = velocity * 100; // Will be converted to km / h in OSD_SLIP
        *((uint8_t*)slipBuffer + 1 + 4) = 0;
        SlipPacketSend(6, (char*)slipBuffer, &Serial3);
      }







      //Send Distance through SLIP (hijacked for simulated distance)
      *((uint8_t*)slipBuffer + 0) = ID_DISTANCE;
      *((uint32_t*)(slipBuffer + 1 + 0)) = (COURSE_LENGTH - distance) * 1000; //(uint32_t)distance * 1000;
/*      
      if (simulation_mode) {
        *((uint32_t*)(slipBuffer + 1 + 0)) = (COURSE_LENGTH - distance) * 1000; //(uint32_t)distance * 1000;
      } else {
        *((uint32_t*)(slipBuffer + 1 + 0)) = (COURSE_LENGTH - distance) * 1000; //(uint32_t)distance * 1000; // Hijacked for simulated distance also
      }
*/
      *((uint8_t*)slipBuffer + 1 + 4) = 0;
      SlipPacketSend(6, (char*)slipBuffer, &Serial3);


      //Send Displacement through SLIP
//      *((uint8_t*)slipBuffer + 0) = ID_DISPLACEMENT;
//      if (simulation_mode) {
//        *((int32_t*)(slipBuffer + 1 + 0)) = (COURSE_LENGTH - distance) * 1000; // Assume same as distance
//      } else {
//        *((int32_t*)(slipBuffer + 1 + 0)) = displacement;
//      }
//      *((uint8_t*)slipBuffer + 1 + 4) = 0;
//      SlipPacketSend(6, (char*)slipBuffer, &Serial3);
      
      //Send simpleDisplacement through SLIP
      *((uint8_t*)slipBuffer + 0) = ID_SIMPLEDISPLACEMENT;
      *((int32_t*)(slipBuffer + 1 + 0)) = simpleDisplacement;
      *((uint8_t*)slipBuffer + 1 + 4) = 0;
      SlipPacketSend(6, (char*)slipBuffer, &Serial3);
      
        // Send battery information
//      float batteryLevel = getBatteryLevel();
//      *((uint8_t*)slipBuffer + 0) = ID_BATTERY;
//      *((uint16_t*)(slipBuffer + 1 + 0)) = batteryLevel * 100;
//      *((uint8_t*)slipBuffer + 1 + 2) = lowBattery(batteryLevel);
//      *((uint8_t*)slipBuffer + 1 + 3) = 0;
//      SlipPacketSend(4, (char*)slipBuffer, &Serial3);
      
//      temperature = readTemp();
//      *((uint8_t*)slipBuffer + 0) = ID_TEMPERATURE;
//      *((int16_t*)(slipBuffer + 1 + 0)) = temperature;
//      *((uint8_t*)slipBuffer + 1 + 2) = highTemp(temperature);
//      *((uint8_t*)slipBuffer + 1 + 3) = 0;
//      SlipPacketSend(4, (char*)slipBuffer, &Serial3);

    } else if (GPSLost && millis() - lastGPSUpdate > 1000) {
      lastGPSUpdate = millis();
      Serial2.end();
      if (!simulation_mode)
        GPS.Init();
      
      
      /* ONCE AGAIN NO PROFILE STUFF
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
      
      // Send real time or simulation mode through SLIP
      *((uint8_t*)slipBuffer + 0) = ID_MODE;
      *((uint8_t*)(slipBuffer + 1 + 0)) = simulation_mode;
      *((uint8_t*)slipBuffer + 1 + 2) = 0;
      SlipPacketSend(2, (char*)slipBuffer, &Serial3);
      */
      
//      targetSpeed = calcSpeed(COURSE_LENGTH - distance, coeff);
//      //Send target speed through SLIP
//      *((uint8_t*)slipBuffer + 0) = ID_TSPEED;
//      *((int32_t*)(slipBuffer + 1 + 0)) = targetSpeed;
//      *((uint8_t*)slipBuffer + 1 + 4) = 0;
//      SlipPacketSend(6, (char*)slipBuffer, &Serial3);

      // Send Speed through SLIP
      *((uint8_t*)slipBuffer + 0) = ID_SPEED;
      *((int32_t*)(slipBuffer + 1 + 0)) = GPS.Ground_Speed;
      *((uint8_t*)slipBuffer + 1 + 4) = 0;
      SlipPacketSend(6, (char*)slipBuffer, &Serial3);
      
      if (SIMULATION) {
        // Send Simulated Speed through SLIP
        *((uint8_t*)slipBuffer + 0) = ID_SIM_SPEED;
        *((int32_t*)(slipBuffer + 1 + 0)) = velocity * 100; // Will be converted to km / h in OSD_SLIP
        *((uint8_t*)slipBuffer + 1 + 4) = 0;
        SlipPacketSend(6, (char*)slipBuffer, &Serial3);
      }

      //Send Distance through SLIP (hijacked for simulated distance)
      *((uint8_t*)slipBuffer + 0) = ID_DISTANCE;
      *((uint32_t*)(slipBuffer + 1 + 0)) = (COURSE_LENGTH - distance) * 1000; //(uint32_t)distance * 1000;
/*      
      if (simulation_mode) {
        *((uint32_t*)(slipBuffer + 1 + 0)) = (COURSE_LENGTH - distance) * 1000; //(uint32_t)distance * 1000;
      } else {
        *((uint32_t*)(slipBuffer + 1 + 0)) = (COURSE_LENGTH - distance) * 1000; //(uint32_t)distance * 1000; // Hijacked for simulated distance also
      }
*/
      *((uint8_t*)slipBuffer + 1 + 4) = 0;
      SlipPacketSend(6, (char*)slipBuffer, &Serial3);
      
      
      
//      //Send Displacement through SLIP
//      *((uint8_t*)slipBuffer + 0) = ID_DISPLACEMENT;
//      *((int32_t*)(slipBuffer + 1 + 0)) = (COURSE_LENGTH - distance) * 1000; // Assume same as distance
//      *((uint8_t*)slipBuffer + 1 + 4) = 0;
//      SlipPacketSend(6, (char*)slipBuffer, &Serial3);
      
      //Send simpleDisplacement through SLIP
      *((uint8_t*)slipBuffer + 0) = ID_SIMPLEDISPLACEMENT;
      *((int32_t*)(slipBuffer + 1 + 0)) = simpleDisplacement;
      *((uint8_t*)slipBuffer + 1 + 4) = 0;
      SlipPacketSend(6, (char*)slipBuffer, &Serial3);
      
      // Send battery information
//      float batteryLevel = getBatteryLevel();
//      *((uint8_t*)slipBuffer + 0) = ID_BATTERY;
//      *((uint16_t*)(slipBuffer + 1 + 0)) = batteryLevel * 100;
//      *((uint8_t*)slipBuffer + 1 + 2) = lowBattery(batteryLevel);
//      *((uint8_t*)slipBuffer + 1 + 3) = 0;
//      SlipPacketSend(4, (char*)slipBuffer, &Serial3);
//      
//      temperature = readTemp();
//      *((uint8_t*)slipBuffer + 0) = ID_TEMPERATURE;
//      *((int16_t*)(slipBuffer + 1 + 0)) = temperature;
//      *((uint8_t*)slipBuffer + 1 + 2) = highTemp(temperature);
//      *((uint8_t*)slipBuffer + 1 + 3) = 0;
//      SlipPacketSend(4, (char*)slipBuffer, &Serial3);
      
    } else if (millis() - lastGPSUpdate > 10000) {
      GPSLost = true;
      // *((uint8_t*)slipBuffer + 0) = ID_GPSCOMM;
      // *((uint8_t*)(slipBuffer + 1 + 0)) = 0; // No receive from GPS
      // *((uint8_t*)slipBuffer + 1 + 1) = 0;
      // SlipPacketSend(2, (char*)slipBuffer, &Serial3);
      
      sd_Open(logFilename);
      sd_Print(" GPS lost connection.");
      sd_Close();
      //Serial.println("No message for 5 seconds.");
    }
  }
  TIME += PERIOD;
  
  logData();  
}




/*********************************************************************************
 *                               FUNCTIONS
 *********************************************************************************/

void logData() {
  // ("Time (hh:mm:ss), Time (ms), Displacement (m), Sim Distance (m), Ground Speed (km/h), Target Speed (km/h), Sim Speed (km/h), Power (W), Cadence (rpm), Latitude (deg), Longitude (deg), Altitude (m), Heart Rate (bpm), Battery (V)\r\n")
  
  sd_Open(logFilename);

  // Times
  sd_Print("\r\n");
  sprintf((char*)sdBuffer, "%u:%u:%u, ", GPS.UTC.hour, GPS.UTC.minute, GPS.UTC.second);
  sd_Print(sdBuffer);
  sd_Print(dtoa(sdBuffer, (double)millis())); sd_Print(", ");
  // Distances
  sprintf((char*)sdBuffer, "%u, ", simpleDisplacement/ 1000);
  sd_Print(sdBuffer);
  
  sd_Print(ftoa(sdBuffer, distance, 3)); sd_Print(", ");
  
  //sprintf((char*)sdBuffer, "%d, ", distance / 1000);
  //sd_Print(sdBuffer);
  
  
  
  //sprintf(sdBuffer, "%5li, ", simpleDisplacement/1000);
  //sd_Print(sdBuffer);
  //sd_Print(dtoa(sdBuffer, distance/1000)); sd_Print(", ");

// TESTING
//  sd_Print(ftoa(sdBuffer, logDisplacement, 3)); sd_Print(", ");
//  sd_Print(dtoa(sdBuffer, simpleDisplacement)); sd_Print(", ");

//  Test to see
//  simpleDisplacement = 134302;
//  Serial.print("dtoa: ");
//  Serial.println(dtoa(sdBuffer, simpleDisplacement)); sd_Print(", ");


  // Speeds
  sd_Print(dtoa(sdBuffer, GPS.Ground_Speed*0.036)); sd_Print(", ");  
  sd_Print(dtoa(sdBuffer, targetSpeed*0.036)); sd_Print(", ");  
  sd_Print(dtoa(sdBuffer, velocity * 3.6)); sd_Print(", ");
 
  // Power and Cadence
  sd_Print(dtoa(sdBuffer, power)); sd_Print(", ");
  sd_Print(dtoa(sdBuffer, cadence)); sd_Print(", ");
 
  // GPS Coordinates
  sd_Print(dtoa(sdBuffer, lat*1.0 / 10000000)); sd_Print(", ");
  sd_Print(dtoa(sdBuffer, lon*1.0 / 10000000)); sd_Print(", ");
  sd_Print(dtoa(sdBuffer, alt*1.0 / 1000));     sd_Print(", ");
 
  // Heart rate and battery
  sprintf(sdBuffer, "%u, ", Hrt);
  sd_Print(sdBuffer);
  sd_Print(ftoa(sdBuffer, getBatteryLevel(), 4)); sd_Print(", ");
  sd_Print(ftoa(sdBuffer, readTemp(), 4)); sd_Print(", ");
  
  sd_Close();
}

// CALIBRATE()
// Calibrate the bike pedals.
void calibrate() {
  bool calibrated = false;
  int8_t m;
  uint8_t i = 0; // Index for cal_values
  int16_t cal_values[6];
  int16_t calibrationValue;

  Serial.println("Begin calibration... Waiting for calibration messages.");
  sd_Log("Begin calibration... Waiting for calibration messages.");
  calibrateMessageOSD(1, 0);

  TEMPTIME = millis(); //To timeout the calibration if taking too long
  
  while (i < 6) {
    if (Serial1.available() && (m = receiveANT(antBuffer)) == 9 && antBuffer[1] == 0x4E && antBuffer[4] == 0x10) {
      cal_values[i++] = antBuffer[9] * 256 + antBuffer[10];
      calibrateMessageOSD(2, 0);
      Serial.print("Receiving... ");
      sprintf(sdBuffer, "Received message %d. ", i);
      sd_Log(sdBuffer);
      
      // Should mean that all the required calibrated values were received
      if (i == 6) {
        calibrated = true; }   
    }
    else { // Nothing available on the ANT serial
      if (millis() - TEMPTIME > CALIBRATION_TIME) { // If more than CALIBRATION_TIME already passed
        break; // exit the while loop
      }
    }
  }
  
  if (calibrated) { 
    calibrationValue = average(cal_values, 6);
    setOffset(calibrationValue);
  
    Serial.print("Calibration complete! Offset is: ");
    Serial.print(calibrationValue);
    Serial.println(" Hz");
    sprintf(sdBuffer, "Calibration complete! Offset is: %d Hz. ", calibrationValue);
    sd_Log(sdBuffer);
    
    // Display the calibration data on the OSD
    calibrateMessageOSD(3, calibrationValue); // The number 3 writes the value to the screen (See calibration state in OSD_panels_HPV) 
  }
  
  else {
    calibrationValue = 507;
    Serial.println("Calibration timed-out...Offset is set as default: 507 Hz");
    sprintf(sdBuffer, "Calibration timed-out...Offset is set as default: 507 Hz");
    sd_Log(sdBuffer);
    
    // Display the calibration data on the OSD
    calibrateMessageOSD(4, calibrationValue); // The number 4 write default to the screen (See calibration state in OSD_panels_HPV)
  }
  // Wait and then clear up the screen (IF REQUIRED)
  //delay(3000); // Not large enough? seeing flicker?
  //calibrateMessageOSD(0, calibrationValue); // The number 0 writes a blank message to the screen 
}

// Sends the specified message and value to the OSD
void calibrateMessageOSD(uint8_t messageNum, int16_t calibrationValue) {
  *((uint8_t*)slipBuffer + 0) = ID_CALIBRATION;
  *((uint8_t*)(slipBuffer + 1 + 0)) = messageNum;
  *((int16_t*)(slipBuffer + 1 + 1)) = calibrationValue;
  *((uint8_t*)slipBuffer + 1 + 3) = 0;
  SlipPacketSend(4, (char*)slipBuffer, &Serial3);
}

/*void logSRM() {
  // Time(hh:mm:ss), Time (ms), GPS Distance (m), GPS Displacement (m), GPS Speed (km/h), Target Speed (km/h), Power (W), Cadence (rpm), Velocity (km/h), Simulated Distance (m), Heart Rate (bpm)
  sd_Open(SRMlogFilename);
  
  sprintf((char*)sdBuffer, "%u:%u:%u, ", GPS.UTC.hour, GPS.UTC.minute, GPS.UTC.second);
  sd_Print(sdBuffer);
  sd_Print(dtoa(sdBuffer, (double)millis())); sd_Print(", ");
  sprintf((char*)sdBuffer, "%u, ", GPS_totalDistance/ 1000);
  sd_Print(sdBuffer);
  sprintf((char*)sdBuffer, "%li, ", displacement / 1000);
  sd_Print(sdBuffer);
  sd_Print(dtoa(sdBuffer, GPS.Ground_Speed*0.036)); sd_Print(", ");
  sd_Print(dtoa(sdBuffer, targetSpeed*0.036)); sd_Print(", ");
  sd_Print(dtoa(sdBuffer, power)); sd_Print(", ");
  sd_Print(dtoa(sdBuffer, cadence)); sd_Print(", ");
  sd_Print(dtoa(sdBuffer, velocity * 3.6)); sd_Print(", ");
  sd_Print(dtoa(sdBuffer, distance)); sd_Print(", ");
  sprintf(sdBuffer, "%u, ", Hrt);
  sd_Print(sdBuffer);
  sd_Print("\r\n");
  
  sd_Close();
}*/

// Calculates the average of int32_t elements in an array of given length
int32_t average(int32_t *beg, const int len) {
  float total = 0;

  for (int i = 0; i < len; ++i) {
    total += *(beg + i) / len;
  }
  
  return (int32_t) (total + 0.5f);
}

//Calculates the average of int16_t elements in an array of given length
int16_t average(int16_t *beg, const int len) {
  float total = 0;

  for (int i = 0; i < len; ++i) {
    total += *(beg + i);
  }

  total /= len;

  return (int16_t) (total + 0.5f);
}



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


// Double to string
char * dtoa(char *s, double n) {
  const double PRECISION = 0.0000001;  
  
  // handle special cases
  if (isnan(n)) {
      strcpy(s, "nan");
  } else if (isinf(n)) {
      strcpy(s, "inf");
  } else if (n == 0.0) {
      strcpy(s, "0");
  } else {
    int digit, m;
    char *c = s;
    int neg = (n < 0);
    if (neg)
      n = -n;
    // calculate magnitude
    m = log10(n);
    if (neg)
      *(c++) = '-';
    if (m < 1.0) {
      m = 0;
    }
    // convert the number
    while (n > PRECISION || m >= 0) {
      double weight = pow(10.0, m);
      if (weight > 0 && !isinf(weight)) {
          digit = floor(n / weight);
          n -= (digit * weight);
          *(c++) = '0' + digit;
      }
      if (m == 0 && n > 0)
          *(c++) = '.';
      m--;
    }
    *(c) = '\0';
  }
  return s;
}
