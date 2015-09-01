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
#include <Wire.h>
#include <SFE_LSM9DS0.h>
#include "inttypes.h"
#include "ANT_interface.h"
#include "GPS_UBLOX.h"
#include "slip.h"
#include "simulation.h"
#include <Kalman.h>
Kalman kalman;

int accSample = 500;
#include "RunningAverage.h"
RunningAverage accAvg(accSample);


#define G_BM 9.79778
#define PERIOD 1000 // Main loop period
#define SERIAL_PRINT true
#define CALIBRATE false

bool BRAKE_MODE = false;
// Old boolean for tracking "simulation mode"
bool simulation_mode = false;
// Revised boolean for enabling simulation during a real run
bool SIMULATION = false;


#define COURSE_LENGTH   8045 // Metres
#define POWER_START      300 // Watts
#define POWER_PRE_SPRINT 337 // Watts
#define CALIBRATION_TIME 15000 // Milliseconds (Should be a few seconds less than the beginningPanels time, so the pedal calibration value can be seen)

// IMU setup
// SDO_XM and SDO_G are both grounded, so our addresses are:
#define LSM9DS0_XM  0x1D // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G   0x6B // Would be 0x6A if SDO_G is LOW
// Create an instance of the LSM9DS0 library called `dof` the
// parameters for this constructor are:
// [SPI or I2C Mode declaration],[gyro I2C address],[xm I2C add.]
LSM9DS0 dof(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);
//const byte INT1XM = 33;
float abias[3] = {0, 0, 0}, gbias[3] = {0, 0, 0};
float ax, ay, az, gx, gy, gz, mx, my, mz;

const uint8_t TOGGLE_PIN = A1;

char slipBuffer[N_SLIP]; //SLIP.h
static uint8_t antBuffer[64]; // ANT+
char sdBuffer[128]; //SD
char gpsFilename[32];
char antFilename[32];
static uint8_t Hrt;
uint32_t TIME;
uint32_t TEMPTIME;

int8_t lastToggle;
int8_t Toggle;
int8_t profileNum = 0;
char profileFilename[32] = {0};
char logFilename[32] = "LogTest.txt"; // "LGdflt.txt"
char profileName[16];
double coeff[7] = {0};

// Default coordinates as bike room (256 McCaul Street)
int32_t LattitudeStart = 436585166, LongitudeStart = -793934912, AltitudeStart = 117689;
int32_t LattitudeFinish = 436585166, LongitudeFinish = -793934912, AltitudeFinish = 117689;

int32_t LattitudePrev, LongitudePrev, AltitudePrev;
int8_t startSet = 0;
uint32_t GPS_totalDistance = 0;


void setup() {

  // Serial2 is GPS
  Serial.begin(115200);	// COM port
  Serial3.begin(115200); // OSD
  Serial1.begin(57600);	// ANT+
  if (SERIAL_PRINT) { Serial.println("Program start!"); }


  // Accelerometer setup
  uint16_t status = dof.begin();
  dof.setAccelScale(dof.A_SCALE_2G); // The smaller the range, the higher the resolution
  dof.setAccelODR(dof.A_ODR_400); // Set output data rate (ODR) at 400 Hz
  dof.setAccelABW(dof.A_ABW_50); // Anti-aliasing filter (like a low pass) at 50 Hz
  // Use the FIFO mode to average accelerometer and gyro readings to calculate the biases, which can then be removed from
  // all subsequent measurements.
  dof.calLSM9DS0(gbias, abias);
  accAvg.fillValue(0, accSample);

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
  if (CALIBRATE) { calibrate(); }
  
  // PUT OSD INTO BRAKE MODE (for testing)
  sendBrakeMode(1);
  BRAKE_MODE = true;
  
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
static int32_t accel = 0;
float logDisplacement = 0;
static int32_t lat, lon, alt;
static bool GPSLost = false;
int counter = 1; // not used?

// For acceleration calc
int32_t prevSpeed = 0;
int32_t currSpeed = 0;
int32_t prevGPSTime = 500;
int32_t currGPSTime = 1000;

 


/********************************************************
 *               MAIN PROGRAM  
 ********************************************************/
void loop() {
  

  uint8_t temp;
  int8_t i, m;
  
  /*
  dof.readAccel();       
  ax = dof.calcAccel(dof.ax) - abias[0];   // Convert to g's, remove accelerometer biases
  ay = dof.calcAccel(dof.ay) - abias[1];
  az = dof.calcAccel(dof.az) - abias[2];
  Serial.print("x acc: ");
  Serial.println(ax*G_BM);
  Serial.print("y acc: ");
  Serial.println(ay*G_BM);
  Serial.print("z acc: ");
  Serial.println(az*G_BM);
 */
 
 
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
        }
      }
      
      if (index == numTerms)
        index = 0;

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
                  
      if (simpleDisplacement < 50000) {
        sendBrakeMode(1);  
        BRAKE_MODE = true;      
      }
      if (simpleDisplacement > 1000000) {
        sendBrakeMode(0); 
        BRAKE_MODE = false;
      }

      if (SIMULATION) { 
        // Target power based on simulated distance travelled
        targetPower = calcPower(distance, POWER_START, POWER_PRE_SPRINT);
        sendTargetPower(targetPower);
        
        targetSpeed = calcSpeed(COURSE_LENGTH - distance, coeff);
        sendSimSpeed();
      }
      else {
        targetSpeed = calcSpeed(COURSE_LENGTH - (simpleDisplacement / 1000.0), coeff);
      }
        
      sendTargetSpeed();
      sendSpeed();
      
      sendDistance(); // Hijacked to send simulated distance
      sendSimpleDisplacement();

      //sendBattery();
      //sendTemperature();

    } else if (GPSLost && millis() - lastGPSUpdate > 1000) {
      lastGPSUpdate = millis();
      Serial2.end();
      GPS.Init();
            
      if (SIMULATION) { 
        targetSpeed = calcSpeed(COURSE_LENGTH - distance, coeff);
        sendSimSpeed();
      }
      else {
        targetSpeed = calcSpeed(COURSE_LENGTH - (simpleDisplacement / 1000.0), coeff);
      }
        
      sendTargetSpeed();
      sendSpeed();
      
      sendDistance(); // Hijacked to send simulated distance
      sendSimpleDisplacement();
      
      //sendBattery();
      //sendTemperature();    
      
    } else if (millis() - lastGPSUpdate > 10000) {
      GPSLost = true;
      //sendGPSCOMM(0);

      sd_Open(logFilename);
      sd_Print(" GPS lost");
      sd_Close();
      if (SERIAL_PRINT) { Serial.println("GPS lost"); }
    }
    
    sendAccel();
    delay(2);
  }
  
  TIME += PERIOD;
  
  logData();  
}



/*********************************************************************************
 *                               FUNCTIONS
 *********************************************************************************/

// Logging data to the SD card
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


// Calibrating the pedals
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
