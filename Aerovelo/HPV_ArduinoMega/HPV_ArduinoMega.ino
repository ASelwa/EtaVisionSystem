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

#define COURSE_LENGTH   8045 // 8045 metres
#define POWER_START      300
#define POWER_PRE_SPRINT 337

const uint8_t TOGGLE_PIN = A1;

char slipBuffer[N_SLIP]; //SLIP.h
static uint8_t antBuffer[64]; // ANT+
char sdBuffer[128]; //SD
char gpsFilename[32];
char antFilename[32];
static uint8_t Hrt;
uint32_t TIME;

int32_t LattitudeStart = 436585166, LongitudeStart = -793934912, AltitudeStart = 117689;
int32_t LattitudeFinish, LongitudeFinish, AltitudeFinish;
int32_t LattitudePrev, LongitudePrev, AltitudePrev;
int8_t startSet = 0;
uint32_t GPS_totalDistance = 0;

int8_t lastToggle;
int8_t Toggle;
int8_t profileNum = 0;
char profileFilename[32] = {0};
char logFilename[32] = "LogTest.txt"; // "LGdflt.txt"
char SRMlogFilename[32] = "SRMLogDf.txt";
char profileName[16];
double coeff[7] = {0};

bool simulation_mode = false;

void setup() {

  // Select serial port.
  // Serial2 is GPS
  Serial.begin(115200);	// COM port
  Serial3.begin(115200); // OSD
  Serial1.begin(57600);	// ANT+
  Serial.println("Program start!");

  // For checking the battery voltage
  analogReference(INTERNAL2V56);

  // Reset ANT+
  pinMode(3, OUTPUT);
  digitalWrite(3, LOW);
  delayMicroseconds(1000);
  digitalWrite(3, HIGH);
  delayMicroseconds(1000);

  // Toggle
  pinMode(TOGGLE_PIN, INPUT);
  digitalWrite(TOGGLE_PIN, HIGH);

  // Initialize GPS and SD
  GPS.Init();
  sd_Init();

  // Figure out SD card log iteration.
  for (int i = 1; i < 100000; ++i){
    sprintf(logFilename, "Log%d.csv", i);
    if (!SD.exists(logFilename)){
	  sprintf(SRMlogFilename, "SLg%d.csv", i);
      break;
    }
  }
  
  loadFinishCoordinates();
  
  sd_Log("Time (hh:mm:ss), Latitude (deg), Longitude (deg), Altitude (m), Distance (m), Displacement (m), Ground Speed (km/h), Target Speed (km/h), Power (W), Cadence (rpm), Velocity (km/h), Simulated Distance (m), Heart Rate (bpm), Battery (V)\r\n");
  sd_Write("Time (ms), GPS Distance (m), GPS Displacement (m), GPS Speed (km/h), Target Speed (km/h), Power (W), Cadence (rpm), Velocity (km/h), Simulated Distance (m), Heart Rate (bpm)", SRMlogFilename);
  
  // Crank Torque Frequency
  //ANT_SetupChannel(antBuffer, 0, 0, 0, 8182, 0); UNCOMMENT THIS LINE TO SEARCH FOR ANY DEVICE (ONLY SAFE FOR DEBUGGING)
  ANT_SetupChannel(antBuffer, 0, 11, 0, 8182, 12131);
  // Heart rate channel.
  ANT_SetupChannel(antBuffer, 1, 120, 0, 16140, 45063);

  Serial.println("ANT+ setup complete.");

  calibrate();
  
  TIME = millis() + PERIOD;
}

int8_t START = 0;

/*
 * Calibrate the bike pedals.
 */
void calibrate() {
  bool calibrated = false;
  int8_t m;
  uint8_t i = 0; // Index for cal_values
  int16_t cal_values[6];
  int16_t calibrationValue;

  Serial.println("Begin calibration... Waiting for calibration messages.");
  sd_Log("Begin calibration... Waiting for calibration messages. ");
  calibrateMessageOSD(1, 0);

  while (i < 6) {
    if (Serial1.available() && (m = receiveANT(antBuffer)) == 9 && antBuffer[1] == 0x4E && antBuffer[4] == 0x10) {
      cal_values[i++] = antBuffer[9] * 256 + antBuffer[10];
      calibrateMessageOSD(2, 0);
      Serial.print("Receiving... ");
      sprintf(sdBuffer, "Received message %d. ", i);
      sd_Log(sdBuffer);
    }
  }

  calibrationValue = average(cal_values, 6);
  setOffset(calibrationValue);

  Serial.print("Calibration complete! Offset is: ");
  Serial.print(calibrationValue);
  Serial.println(" Hz");
  sprintf(sdBuffer, "Calibration complete! Offset is: %d Hz. ", calibrationValue);
  sd_Log(sdBuffer);

  calibrateMessageOSD(3, calibrationValue);
  delay(2000);
  calibrateMessageOSD(0, calibrationValue);
}

/*
 * Sends the specified message and value to the OSD
 */
void calibrateMessageOSD(uint8_t messageNum, int16_t calibrationValue) {
  *((uint8_t*)slipBuffer + 0) = ID_CALIBRATION;
  *((uint8_t*)(slipBuffer + 1 + 0)) = messageNum;
  *((int16_t*)(slipBuffer + 1 + 1)) = calibrationValue;
  *((uint8_t*)slipBuffer + 1 + 3) = 0;
  SlipPacketSend(4, (char*)slipBuffer, &Serial3);
}

static float power, cadence, velocity, distance = 0;
float averagePower, power10s, targetPower;
static bool coast = false;
static uint16_t time_int = 0;
static uint16_t t2 = 0;
static uint32_t lastGPSUpdate = millis();
static uint32_t targetSpeed;
static int32_t displacement;
static int32_t lat, lon, alt;
static bool GPSLost = false;

/*
 * The main program
 */
void loop() {
  int8_t slipLen;

  uint8_t temp;
  int8_t i, m;

  // Anything outside this loop is executed every PERIOD milliseconds
  while (TIME > millis()) {
//    UNCOMMENT THE FOLLOWING TO PRINT MESSAGES RECEIVED
//    slipLen = SlipReceive(slipBuffer, &Serial);
//    if (slipLen > 0) {
//      slipBuffer[slipLen] = 0;
//      Serial.println(slipBuffer);
//      if (slipBuffer[0] == '!') START = 1;
//      else if (slipBuffer[0] == '*') START = 0;
//    }

    // Read ANT+ Data
    if (Serial1.available()) {
      if ((m = receiveANT(antBuffer)) > 0) {
        Serial.print("ANT+ Packet Received: ");
        for (i = 0; i < m + 3; ++i) {
          Serial.print(antBuffer[i], HEX);
          Serial.print(' ');
        } Serial.print('\n');

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
              
              if (simulation_mode || GPSLost) {
                targetPower = calcPower(distance, POWER_START, POWER_PRE_SPRINT);
                *((uint8_t*)slipBuffer + 0) = ID_TPOWER;
                *((uint16_t*)(slipBuffer + 1 + 0)) = targetPower;
                *((uint8_t*)slipBuffer + 1 + 2) = 0;
                SlipPacketSend(3, (char*)slipBuffer, &Serial3);
              }

              *((uint8_t*)slipBuffer + 0) = ID_POWER;
              *((uint16_t*)(slipBuffer + 1 + 0)) = power;
              *((uint8_t*)slipBuffer + 1 + 2) = cadence;
              *((uint8_t*)slipBuffer + 1 + 3) = 0;
              SlipPacketSend(4, (char*)slipBuffer, &Serial3);
              
              *((uint8_t*)slipBuffer + 0) = ID_AVG_POWER;
              *((uint16_t*)(slipBuffer + 1 + 0)) = averagePower;
              *((uint8_t*)slipBuffer + 1 + 2) = 0;
              SlipPacketSend(3, (char*)slipBuffer, &Serial3);
              
              *((uint8_t*)slipBuffer + 0) = ID_10S_POWER;
              *((uint16_t*)(slipBuffer + 1 + 0)) = power10s;
              *((uint8_t*)slipBuffer + 1 + 2) = 0;
              SlipPacketSend(3, (char*)slipBuffer, &Serial3);
			  
			  // Write the updated values to the log file
			  logSRM();
              
              break;

            case 1: // Heart rate.
              Hrt = antBuffer[3 + 7];
              *((uint8_t*)slipBuffer + 0) = ID_HEART;
              *((uint8_t*)(slipBuffer + 1 + 0)) = Hrt;
              *((uint8_t*)slipBuffer + 1 + 1) = 0;
              SlipPacketSend(2, (char*)slipBuffer, &Serial3);
              break;
          }
        }

      }
    }

    //Read GPS Data
    GPS.Read(); // Updates buffer

    if (GPS.NewData) {
      GPS.NewData = 0;
      lastGPSUpdate = millis();
      GPSLost = false;
      
      Serial.println("GPS received!");
      
      *((uint8_t*)slipBuffer + 0) = ID_GPSCOMM;
      *((uint8_t*)(slipBuffer + 1 + 0)) = 1; // Received from GPS
      *((uint8_t*)slipBuffer + 1 + 1) = 0;
      SlipPacketSend(2, (char*)slipBuffer, &Serial3);

      // Track the moving average
      const int numTerms = 3; // Moving average of this many values
      static int32_t latitude[numTerms];
      static int32_t longitude[numTerms];
      static int32_t altitude[numTerms];
      static int index = 0;

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

      if (index == numTerms)
        index = 0;

      latitude[index] = GPS.Lattitude;
      longitude[index] = GPS.Longitude;
      altitude[index++] = GPS.Altitude;

      lat = average(latitude, numTerms);
      lon = average(longitude, numTerms);
      alt = average(altitude, numTerms);
      
      displacement = GPS_getDistance(LattitudeFinish, LongitudeFinish, AltitudeFinish, lat, lon, alt);
      uint32_t currDistance = GPS_getDistance(LattitudePrev, LongitudePrev, AltitudePrev, lat, lon, alt);

      if (currDistance >= 0) {
        GPS_totalDistance += currDistance;

        LattitudePrev = lat;
        LongitudePrev = lon;
        AltitudePrev = alt;
      }
      
      if (!simulation_mode) {
        targetPower = calcDisplacePower(displacement, POWER_START, POWER_PRE_SPRINT);
        *((uint8_t*)slipBuffer + 0) = ID_TPOWER;
        *((uint16_t*)(slipBuffer + 1 + 0)) = targetPower;
        *((uint8_t*)slipBuffer + 1 + 2) = 0;
        SlipPacketSend(3, (char*)slipBuffer, &Serial3);
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
      
      // Send real time or simulation mode through SLIP
      *((uint8_t*)slipBuffer + 0) = ID_MODE;
      *((uint8_t*)(slipBuffer + 1 + 0)) = simulation_mode;
      *((uint8_t*)slipBuffer + 1 + 2) = 0;
      SlipPacketSend(2, (char*)slipBuffer, &Serial3);

      if (simulation_mode) {
        targetSpeed = calcSpeed(COURSE_LENGTH - distance, coeff);
      } else {
        targetSpeed = calcSpeed(displacement / 1000.0, coeff);
      }
      //Send target speed through SLIP
      *((uint8_t*)slipBuffer + 0) = ID_TSPEED;
      *((int32_t*)(slipBuffer + 1 + 0)) = targetSpeed;
      *((uint8_t*)slipBuffer + 1 + 4) = 0;
      SlipPacketSend(6, (char*)slipBuffer, &Serial3);

      // Send Speed through SLIP
      *((uint8_t*)slipBuffer + 0) = ID_SPEED;
      *((int32_t*)(slipBuffer + 1 + 0)) = GPS.Ground_Speed;
      *((uint8_t*)slipBuffer + 1 + 4) = 0;
      SlipPacketSend(6, (char*)slipBuffer, &Serial3);
      
      // Send Simulated Speed through SLIP
      *((uint8_t*)slipBuffer + 0) = ID_SIM_SPEED;
      *((int32_t*)(slipBuffer + 1 + 0)) = velocity * 100; // Will be converted to km / h in OSD_SLIP
      *((uint8_t*)slipBuffer + 1 + 4) = 0;
      SlipPacketSend(6, (char*)slipBuffer, &Serial3);

      //Send Distance through SLIP
      *((uint8_t*)slipBuffer + 0) = ID_DISTANCE;
      if (simulation_mode) {
        *((uint32_t*)(slipBuffer + 1 + 0)) = distance * 1000;
      } else {
        *((uint32_t*)(slipBuffer + 1 + 0)) = GPS_totalDistance;
      }
      *((uint8_t*)slipBuffer + 1 + 4) = 0;
      SlipPacketSend(6, (char*)slipBuffer, &Serial3);

      //Send Displacement through SLIP
      *((uint8_t*)slipBuffer + 0) = ID_DISPLACEMENT;
      if (simulation_mode) {
        *((int32_t*)(slipBuffer + 1 + 0)) = (COURSE_LENGTH - distance) * 1000; // Assume same as distance
      } else {
        *((int32_t*)(slipBuffer + 1 + 0)) = displacement;
      }
      *((uint8_t*)slipBuffer + 1 + 4) = 0;
      SlipPacketSend(6, (char*)slipBuffer, &Serial3);
      
        // Send battery information
      float batteryLevel = getBatteryLevel();
      *((uint8_t*)slipBuffer + 0) = ID_BATTERY;
      *((uint16_t*)(slipBuffer + 1 + 0)) = batteryLevel * 100;
      *((uint8_t*)slipBuffer + 1 + 2) = lowBattery(batteryLevel);
      *((uint8_t*)slipBuffer + 1 + 3) = 0;
      SlipPacketSend(4, (char*)slipBuffer, &Serial3);
      
      uint16_t temperature = readTemp();
      *((uint8_t*)slipBuffer + 0) = ID_TEMPERATURE;
      *((int16_t*)(slipBuffer + 1 + 0)) = temperature;
      *((uint8_t*)slipBuffer + 1 + 2) = highTemp(temperature);
      *((uint8_t*)slipBuffer + 1 + 3) = 0;
      SlipPacketSend(4, (char*)slipBuffer, &Serial3);

    } else if (GPSLost && millis() - lastGPSUpdate > 1000) {
      lastGPSUpdate = millis();
      Serial2.end();
      if (!simulation_mode)
        GPS.Init();
      
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
      
      targetSpeed = calcSpeed(COURSE_LENGTH - distance, coeff);
      //Send target speed through SLIP
      *((uint8_t*)slipBuffer + 0) = ID_TSPEED;
      *((int32_t*)(slipBuffer + 1 + 0)) = targetSpeed;
      *((uint8_t*)slipBuffer + 1 + 4) = 0;
      SlipPacketSend(6, (char*)slipBuffer, &Serial3);

      // Send Speed through SLIP
      *((uint8_t*)slipBuffer + 0) = ID_SPEED;
      *((int32_t*)(slipBuffer + 1 + 0)) = velocity * 100;
      *((uint8_t*)slipBuffer + 1 + 4) = 0;
      SlipPacketSend(6, (char*)slipBuffer, &Serial3);
      
      // Send Simulated Speed through SLIP
      *((uint8_t*)slipBuffer + 0) = ID_SIM_SPEED;
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
      *((int32_t*)(slipBuffer + 1 + 0)) = (COURSE_LENGTH - distance) * 1000; // Assume same as distance
      *((uint8_t*)slipBuffer + 1 + 4) = 0;
      SlipPacketSend(6, (char*)slipBuffer, &Serial3);
      
      // Send battery information
      float batteryLevel = getBatteryLevel();
      *((uint8_t*)slipBuffer + 0) = ID_BATTERY;
      *((uint16_t*)(slipBuffer + 1 + 0)) = batteryLevel * 100;
      *((uint8_t*)slipBuffer + 1 + 2) = lowBattery(batteryLevel);
      *((uint8_t*)slipBuffer + 1 + 3) = 0;
      SlipPacketSend(4, (char*)slipBuffer, &Serial3);
      
      uint16_t temperature = readTemp();
      *((uint8_t*)slipBuffer + 0) = ID_TEMPERATURE;
      *((int16_t*)(slipBuffer + 1 + 0)) = temperature;
      *((uint8_t*)slipBuffer + 1 + 2) = highTemp(temperature);
      *((uint8_t*)slipBuffer + 1 + 3) = 0;
      SlipPacketSend(4, (char*)slipBuffer, &Serial3);
      
    } else if (millis() - lastGPSUpdate > 7000) {
      GPSLost = true;
      // *((uint8_t*)slipBuffer + 0) = ID_GPSCOMM;
      // *((uint8_t*)(slipBuffer + 1 + 0)) = 0; // No receive from GPS
      // *((uint8_t*)slipBuffer + 1 + 1) = 0;
      // SlipPacketSend(2, (char*)slipBuffer, &Serial3);
      
      sd_Open(logFilename);
      sd_Print("GPS lost connection. ");
      sd_Close();
      Serial.println("No message for 5 seconds.");
    }
  }
  TIME += PERIOD;
  
  //Store in SD
  sd_Open(logFilename);
  
  sd_Print("\r\n");
  sprintf((char*)sdBuffer, "%u:%u:%u, ", GPS.UTC.hour, GPS.UTC.minute, GPS.UTC.second);
  sd_Print(sdBuffer);
  sd_Print(dtoa(sdBuffer, lat*1.0 / 10000000)); sd_Print(", ");
  sd_Print(dtoa(sdBuffer, lon*1.0 / 10000000)); sd_Print(", ");
  sd_Print(dtoa(sdBuffer, alt*1.0 / 1000)); sd_Print(", ");
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
  sd_Print(dtoa(sdBuffer, getBatteryLevel())); sd_Print(", ");
  
  sd_Close();
}

void logSRM() {
  // Time (ms), GPS Distance (m), GPS Displacement (m), GPS Speed (km/h), Target Speed (km/h), Power (W), Cadence (rpm), Velocity (km/h), Simulated Distance (m), Heart Rate (bpm)
  sd_Open(SRMlogFilename);
  
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
}

/*
 * Calculates the average of int32_t elements in an array of given length
 */
int32_t average(int32_t *beg, const int len) {
  float total = 0;

  for (int i = 0; i < len; ++i) {
    total += *(beg + i) / len;
  }
  
  return (int32_t) (total + 0.5f);
}

/*
 * Calculates the average of int16_t elements in an array of given length
 */
int16_t average(int16_t *beg, const int len) {
  float total = 0;

  for (int i = 0; i < len; ++i) {
    total += *(beg + i);
  }

  total /= len;

  return (int16_t) (total + 0.5f);
}

/**
 * Double to string
 */
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
