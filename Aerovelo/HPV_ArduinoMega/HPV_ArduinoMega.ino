/* Arduino Mega Main Program
 * By Oleksiy Ryndin and Sherry Shi
 */
 
 /*
     TODO:
     
      - BUG: When simulation mode, will never enter BRAKE MODE????
     
      - Integer overflow on the simulated distance (32,000? even though it's always treated as a float)
      - And overflow on something else in the log    
      - check 45 M as the simpleDisp shown after simulation
      
      - store default calibration number (507 Hz, found in simulation.cpp also) in one place

      - GPS frequency investigation (moving average, update freq, and so on)


      Done- percent difference = (gps - simulated) / gps * 100%
      Done- check wheel circumference (based on past excel data)
  
      Done- keep only one sd card logging function (the one that takes in a filename) 
        (or do something even smarter than the current method)
      
      Done- GPS time enable  

  */


/**********************************************************************************************************************
 *                                                      LIBRARIES                                                     *
 **********************************************************************************************************************/
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <SFE_LSM9DS0.h>
#include "inttypes.h"
#include "ANT_interface.h"
#include "GPS_UBLOX.h"
#include "slip.h"
#include "simulation.h"

/**********************************************************************************************************************
 *                                                      PARAMETERS                                                    *
 **********************************************************************************************************************/
// ADJUSTABLE PARAMETERS
#define PERIOD 1000 // Main loop period
#define BRAKE_MODE_ENTER 50000 // When distance*1000 [m] from start coordinates is less than this, enter brake mode
#define BRAKE_MODE_EXIT 1200000 // When distance*1000 [m] from start coordinates is larger than this, exit brake mode
#define G_BM 9.79778
#define TIRE_CIRC 1.93 // Metres
#define COURSE_LENGTH   8045 // Metres  ****Change this in simulation.cpp also!!!
#define POWER_START      300 // Watts
#define POWER_PRE_SPRINT 337 // Watts
#define CALIBRATION_TIME 15000 // Milliseconds (Should be a few seconds less than the beginningPanels time, so the pedal calibration value can be seen)

//#define DRIVE_ETA 0.95 // Drivetrain efficiency
//#define M 113.85 // Mass of bike in kg
//#define M_WHEELS 2.23 // Wheel mass in kg (effective factor applied)
//#define M_TOTAL M + M_WHEELS

// BOOLEAN FLAGS
#define SERIAL_PRINT false
#define CALIBRATE true
int BRAKE_MODE = 0;
bool SIMULATION = false; // enabling simulation during a real run

/**********************************************************************************************************************
 *                                                    PERIPHERAL SETUP                                                *
 **********************************************************************************************************************/
// OSD
char slipBuffer[N_SLIP]; //SLIP.h

// ANT+
static uint8_t antBuffer[64]; // ANT+
char antFilename[32];
static uint8_t Hrt;

// SD 
char sdBuffer[128]; //SD
char logFilename[32] = "LogTest.txt"; // "LGdflt.txt"

// GPS
char gpsFilename[32];
// Default coordinates as bike room (256 McCaul Street)
int32_t LattitudeStart = 436585166, LongitudeStart = -793934912, AltitudeStart = 117689;

// Finish based on run on Monday Sept 14 (Morning)
int32_t LattitudeFinish = 404667129, LongitudeFinish = -1170625839, AltitudeFinish = 1408776;

// Finish based on coordinates from old SD card
//int32_t LattitudeFinish = 404675025, LongitudeFinish = -1170627136, AltitudeFinish = 1406018;

int32_t LattitudePrev, LongitudePrev, AltitudePrev;
int8_t startSet = 0;
uint32_t GPS_totalDistance = 0;

// Toggle Button
const uint8_t TOGGLE_PIN = A1;
int8_t lastToggle;
int8_t Toggle;
int8_t profileNum = 0;
char profileFilename[32] = {0};
char profileName[16];

// IMU setup
#define LSM9DS0_XM  0x1D 
#define LSM9DS0_G   0x6B
LSM9DS0 dof(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);
//const byte INT1XM = 33; // For interrupts on accelerometer
float abias[3] = {0, 0, 0}, gbias[3] = {0, 0, 0};
float ax, ay, az, gx, gy, gz, mx, my, mz;
int accSample = 500;
#include "RunningAverage.h"
RunningAverage accAvg(accSample);

/* Hall Effect 
+5V - Yellow - RED
GND - Green - GREEN
Analog - Orange - BLUE */


/**********************************************************************************************************************
 *                                                        VARIABLES                                                   *
 **********************************************************************************************************************/
uint32_t TIME;
uint32_t TEMPTIME;
double coeff[7] = {0};

int8_t START = 0;

static float power = 0, cadence, velocity, distance = 0;
float averagePower, power10s, targetPower;
static bool coast = false;
static uint16_t time_int = 0;
static uint16_t t2 = 0;
static uint32_t lastGPSUpdate = millis();
static uint32_t targetSpeed = 0;
static int32_t displacement;
static int32_t simpleDisplacement = 0;
static int32_t accel = 0;
int32_t rando = 0;
float logDisplacement = 0;
static int32_t lat, lon, alt;
static bool GPSLost = false;

uint8_t temp;
int8_t i, m;

// Hall Effect
float vel, velAvg;
int32_t velAvgInt = 0;
int sensorValue = 0;
int count = 0;
static int32_t currentTimeFunc = 1;
static int32_t previousTimeFunc = 0;
boolean trigger = false;

static int32_t currSpeed = 0;
static int32_t prevSpeed = 0;
static int32_t prevTime = 0;
static int32_t currTime = 100;
static int32_t hallStartTime = 0;
int threshold = 288;

// Runtime
uint16_t timePlaceholder = 0;

int32_t qqq = 0;

/**********************************************************************************************************************
 *                                                         SETUP                                                      *
 **********************************************************************************************************************/
void setup() {

  timePlaceholder = millis();
  
  // Serial Setup
  Serial.begin(115200);	// COM port
  Serial3.begin(115200); // OSD
  Serial1.begin(57600);	// ANT+
  // Serial2 is GPS
  if (SERIAL_PRINT) { Serial.println("Program start!"); }
  
  // Toggle
  pinMode(TOGGLE_PIN, INPUT);
  digitalWrite(TOGGLE_PIN, HIGH); // Enables the internal pullup resistor

  // Check status of button, if HIGH simulation mode, else SIMULATION stays equal to false
  if (digitalRead(TOGGLE_PIN)) {
    SIMULATION = true; 
  }
   
  // Reset ANT+
  pinMode(3, OUTPUT);
  digitalWrite(3, LOW);
  delayMicroseconds(2000);
  digitalWrite(3, HIGH);
  delayMicroseconds(2000);

  // Initialize GPS and SD
  GPS.Init();
  //sendGPSCOMM(true);
  sd_Init();

  // Figure out SD card log iteration.
  for (int i = 1; i < 100000; ++i){
    sprintf(logFilename, "Log%d.csv", i);
    if (!SD.exists(logFilename)){
      //sprintf(SRMlogFilename, "SLg%d.csv", i);
      break;
    }
  }
  // Write the log file headings
  sd_Log("Time (hh:mm:ss), Time (ms), Displacement (m), Sim Distance (m), GPS Speed (km/h), HallEff Speed (km/h), Target Speed (km/h), Sim Speed (km/h), Power (W), Cadence (rpm), Accel Force (N), Latitude (deg), Longitude (deg), Altitude (m), Heart Rate (bpm), Battery (V), Temperature (C),");
  
  // ANT_SetupChannel(antBuffer, 0, 0, 0, 8182, 0); UNCOMMENT THIS LINE TO SEARCH FOR ANY DEVICE (ONLY SAFE FOR DEBUGGING)
  ANT_SetupChannel(antBuffer, 0, 11, 0, 8182, 12131); // Crank Torque Frequency
  ANT_SetupChannel(antBuffer, 1, 120, 0, 16140, 45063);  // Heart rate channel
  if (SERIAL_PRINT) { Serial.println("ANT+ setup complete."); }

  // Wait for at least 1000 ms from startup so the OSD receives the mode and battery level
  while (millis() - timePlaceholder < 3000) {
    delay(10);
  }

  // Placed here so there's time for the OSD to boot up
  sendMode(); // Send the mode to the OSD
  sendBattery(); 
  
  // Calibrate pedals
  if (CALIBRATE) { calibrate(); }

  // PUT OSD INTO BRAKE MODE (for testing)
//  BRAKE_MODE = 1;
//  sendBrakeMode(BRAKE_MODE);

  TIME = millis() + PERIOD; 
 
//  Serial.print("Setup ");
//  Serial.println(millis()-timePlaceholder);



// EXCESS CODE (MAY BE REQUIRED AT SOME POINT)

  // Hall effect setup
//  thresholdHallEffect(5);
//  Serial.print("threshold: ");
//  Serial.println(threshold);
  
   /* // Accelerometer setup
  uint16_t status = dof.begin();
  dof.setAccelScale(dof.A_SCALE_2G); // The smaller the range, the higher the resolution
  dof.setAccelODR(dof.A_ODR_400); // Set output data rate (ODR) at 400 Hz
  dof.setAccelABW(dof.A_ABW_50); // Anti-aliasing filter (like a low pass) at 50 Hz
  // Use the FIFO mode to average accelerometer and gyro readings to calculate the biases, which can then be removed from
  // all subsequent measurements.
  dof.calLSM9DS0(gbias, abias);
  accAvg.fillValue(0, accSample); */
  
  // For checking the battery voltage
  //analogReference(INTERNAL2V56);

  //loadFinishCoordinates();
}


/**********************************************************************************************************************
 *                                                      MAIN PROGRAM                                                  *
 **********************************************************************************************************************/
void loop() {
 
/*********************** BEGIN: PERIOD WHILE LOOP ***************************/  
 
  while (TIME > millis()) {
    
    //receiveOSD() // Should print messages received from OSD?
    
    if (BRAKE_MODE) {
      
      //timePlaceholder = millis();
      
      currSpeed = (int32_t)(hallSpeed(850, 1)*100000000); // m/s scaled up by 1E8 // 500ms gives clean data above 60km/h
      // time is in micro seconds (scaled up by 1E6)
      // NOTE: using currentTimeFunc which is updated at the end of the hallSpeed call
      accel = (currSpeed - prevSpeed) / (currentTimeFunc - prevTime); // Result is m/s^2 scaled up by 1E2

      prevTime = currentTimeFunc;
      prevSpeed = currSpeed;  

//      Serial.print("Current Time [micro s]: ");
//      Serial.println(currentTimeFunc);
//      Serial.print("Previous Time [micro s]: ");
//      Serial.println(prevTime);
//      Serial.print("Speed [km/h]: ");
//      Serial.print(float(currSpeed*3.6)/100000000);
//      Serial.print("    Accel Force [N]: ");
//      Serial.println(accel*105/100);
      
      sendAccel();
      //sendSpeed();

     
    //Serial.print("BRAKE MODE     ");
    //Serial.println(millis()-timePlaceholder); 
    }
    
/************************* BEGIN: NOT BRAKE MODE ****************************/
    else {     
      //timePlaceholder = millis();
/*********************** BEGIN: READ ANT+ DATA ***************************/
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
/************************* END: READ ANT+ DATA **************************/


/*********************** BEGIN: READ GPS DATA ***************************/
      doGPS();
/************************* END: READ GPS DATA ****************************/


    }
/************************* END: NOT BRAKE MODE ****************************/
 


  }
/*********************** END: PERIOD WHILE LOOP ***************************/ 

  TIME += PERIOD;
  
  //timePlaceholder = millis();
  logData();  
  //Serial.print("LogData        ");
  //Serial.println(millis()-timePlaceholder);
}



/*********************************************************************************
 *                               FUNCTIONS
 *********************************************************************************/

// Logging data to the SD card
void logData() {
  //   sd_Log("Time (hh:mm:ss), Time (ms), Displacement (m), Sim Distance (m), GPS Speed (km/h), HallEff Speed (km/h), Target Speed (km/h), Sim Speed (km/h), Power (W), Cadence (rpm), Accel Force (N), Latitude (deg), Longitude (deg), Altitude (m), Heart Rate (bpm), Battery (V), Temperature (C),");
  
  sd_Open(logFilename);
  
  // Times
  sd_Print("\r\n");
  sprintf((char*)sdBuffer, "%u:%u:%u, ", GPS.UTC.hour, GPS.UTC.minute, GPS.UTC.second);
  sd_Print(sdBuffer);
  sd_Print(dtoa(sdBuffer, (double)millis())); sd_Print(", ");

  // Distances
  sprintf((char*)sdBuffer, "%d, ", simpleDisplacement/ 1000);
  sd_Print(sdBuffer);  
  sd_Print(ftoa(sdBuffer, distance, 3)); sd_Print(", ");

  // Speeds
  sd_Print(dtoa(sdBuffer, GPS.Ground_Speed*0.036)); sd_Print(", ");
  sd_Print(dtoa(sdBuffer, velAvg*39/18.0*3.6)); sd_Print(", ");
  sd_Print(dtoa(sdBuffer, targetSpeed*0.036)); sd_Print(", ");  
  sd_Print(dtoa(sdBuffer, velocity * 3.6)); sd_Print(", ");
 
  // Power and Cadence
  sd_Print(dtoa(sdBuffer, power)); sd_Print(", ");
  sd_Print(dtoa(sdBuffer, cadence)); sd_Print(", ");
  
  // Accel Force
  sd_Print(dtoa(sdBuffer, float(accel)*105/100)); sd_Print(", ");
//  Serial.print("debug accel: ");
//  Serial.println(float(accel)*105/100);
 
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

// Gets the threshold for the hall effect sensor
void thresholdHallEffect(int N) {
  
  int sensorValue;
  int sensorMin = threshold;
  int sensorMax = threshold;
  boolean trigger= false;
  int i = 0;
  
  while (i < N) {
  sensorValue = analogRead(A3);
  //Serial.println(sensorValue);
  if ((sensorValue < threshold) && (trigger == 0)) {
    trigger = 1;
    i++;
    Serial.print(i);
    Serial.print('\t');
    Serial.println(2);
  }
  else if ((sensorValue > threshold) && (trigger == 1)) {
    trigger = 0;
  }
  if (sensorValue < sensorMin)
    sensorMin = sensorValue;
  else if (sensorValue > sensorMax)
      sensorMax = sensorValue;
  }
  
  threshold = (sensorMax + sensorMin)/2;
  
}

float hallSpeed(int sampleTime, int samples) {
   
    count = 0; // How many velocity points accumulated
    velAvg = 0;

    hallStartTime = millis();
   
    //Serial.println("entered hallSpeed");
//    while (millis() - hallStartTime < sampleTime) {     
    while (count < samples+1) {
          
      if (millis() - hallStartTime > sampleTime) {
        break;
      }
      
      sensorValue = analogRead(A3);
      //Serial.println(sensorValue);
        
      if ((sensorValue < threshold) && (trigger == 0)) {
        currentTimeFunc = micros();
          
          // Ignore the first trigger to get timing correctly
          if (count != 0) {
            vel = TIRE_CIRC/(currentTimeFunc-previousTimeFunc)*1000000;  
            velAvg = velAvg + vel;
            
//            Serial.print(currentTimeFunc);
//            Serial.print('\t');
//            Serial.print(vel);
//            Serial.print('\t');
//            Serial.print(count);
//            Serial.print('\t');
//            Serial.println(3);
          }
        trigger = 1;
        previousTimeFunc = currentTimeFunc;
        count++;
      }
      
      else if ((sensorValue > threshold) && (trigger == 1)) {
        trigger = 0;
      }
      
      
//      currentTimeFunc = micros();
//    timePlaceholder = millis();
      doGPS();
//    Serial.print("doGPS took: ");
//    Serial.println(millis() - timePlaceholder);
      
      
      

    }
    
//    Serial.println(3);
//    Serial.print("Collected ");
//    Serial.print(count);
//    Serial.print(" samples, velAvg = ");
//    Serial.print(velAvg/count);


    
    if (count == 0) {
      //Serial.println("Zero - bad avg...");
      return 0; 
  }
    else { 
      velAvgInt = (int32_t)(velAvg*100*39/18)/(count - 1); // m/s becomes 100 m/s and 39-18 gear reduction
      // Check gear ratio
      // Check wheel circumference
      // Check cadence and gear and speed correlation.

      return velAvg*39/18/(count - 1); 
    }    

}


void doGPS() {
  
  GPS.Read(); //Read GPS Data (Updates buffer?)
      //Serial.println(GPS.NewData);
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
        //logDisplacement = GPS_getDistance(LattitudeStart, LongitudeStart, AltitudeStart, lat, lon, alt);
        //Serial.print(GPS_getDistance(LattitudeStart, LongitudeStart, AltitudeStart, lat, lon, alt));


        //simpleDisplacement = GPS_getDistance(LattitudeStart, LongitudeStart, AltitudeStart, lat, lon, alt);
        simpleDisplacement = GPS_getDistance(LattitudeFinish, LongitudeFinish, AltitudeFinish, lat, lon, alt);
  
  
        delay(1); //required for proper update freq
        //Serial.println("New speed should have been sent....");
                      
        if (SIMULATION) { 
          // Simple displacement hijacked to send simulated distance
          simpleDisplacement = (COURSE_LENGTH - distance) * 1000;    
        }
        
        // Checking and adjusting BRAKE_MODE appropriately
        if (simpleDisplacement < BRAKE_MODE_ENTER) {  
          BRAKE_MODE = 1; 
        }
        if (simpleDisplacement > BRAKE_MODE_EXIT) {
          BRAKE_MODE = 0;
        }
        sendBrakeMode(BRAKE_MODE);
  
        targetSpeed = speedLookup(COURSE_LENGTH - (simpleDisplacement / 1000.0));
        targetPower = powerLookup(COURSE_LENGTH - (simpleDisplacement / 1000.0));
        
//        Serial.print("targetSpeed: ");
//        Serial.print(targetSpeed*0.036);
//        Serial.print("    distance: ");
//        Serial.println(COURSE_LENGTH - ((simpleDisplacement-qqq) / 1000.0));
//        Serial.print("simpleDisplacement: ");
//        Serial.println(simpleDisplacement / 1000.0);
  
        sendSpeed();
        sendTargetSpeed();
        sendSimSpeed();
  
        sendTargetPower(targetPower);
        
        sendSimpleDisplacement();
  
        //sendBattery();
        //sendTemperature();
  
      } else if (GPSLost && millis() - lastGPSUpdate > 1000) {
        lastGPSUpdate = millis();
        Serial2.end();
        GPS.Init();
        
                
        if (SIMULATION) { 
          // Simple displacement hijacked to send simulated distance
          simpleDisplacement = (COURSE_LENGTH - distance) * 1000;    
        }
        
        // Checking and adjusting BRAKE_MODE appropriately
        if (simpleDisplacement < BRAKE_MODE_ENTER) {  
          BRAKE_MODE = 1; 
        }
        if (simpleDisplacement > BRAKE_MODE_EXIT) {
          BRAKE_MODE = 0;
        }
        sendBrakeMode(BRAKE_MODE);
        
  
        targetSpeed = speedLookup(COURSE_LENGTH - (simpleDisplacement / 1000.0));
        targetPower = powerLookup(COURSE_LENGTH - (simpleDisplacement / 1000.0));
  
        sendSpeed();
        sendTargetSpeed();
        sendSimSpeed();
  
        sendTargetPower(targetPower);
        
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
      
}
  
  

