
/* **********************************************/
/* ***************** INCLUDES *******************/

//#define membug 
//#define FORCEINIT  // You should never use this unless you know what you are doing 

// AVR Includes
#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Math.h>
#include <math.h>
#include <inttypes.h>
#include <avr/pgmspace.h>

// Configurations
#include "OSD_Config.h"
#include "ArduCam_Max7456.h"
#include "OSD_Vars.h"
#include "OSD_Func.h"

// Get the common arduino functions
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "wiring.h"
#endif
#include <EEPROM.h>
#include <SimpleTimer.h>

#include "OSD_SLIP.h"
#ifdef membug
#include <MemoryFree.h>
#endif


/* ************************************************************ */
/* **************** MAIN PROGRAM - MODULES ******************** */
/* ************************************************************ */

#undef PROGMEM 
#define PROGMEM __attribute__(( section(".progmem.data") )) 

#undef PSTR 
#define PSTR(s) (__extension__({static prog_char __c[] PROGMEM = (s); &__c[0];})) 

/* *************************************************/
/* ***************** DEFINITIONS *******************/

//OSD Hardware 
//#define ArduCAM328
#define MinimOSD

#define TELEMETRY_SPEED  115200  // How fast our MAVLink telemetry is coming to Serial port
#define BOOTTIME         2000   // Time in milliseconds that we show boot loading bar and wait user input

// Objects and Serial definitions
FastSerialPort0(Serial);
OSD osd; //OSD object 
int8_t profile_Num;

SimpleTimer  Timer;

char slipBuffer[N_SLIP];

int slipLen;

char display_string[128];
float cadence;
uint16_t power, power_10s, avgPower, targetPower;
uint32_t GPS_Time, GPS_Distance;
int32_t GPS_Displacement;
uint8_t GPS_NumSats;
int32_t GPS_Altitude, GPS_Heading;
double GPS_Speed, simulatedSpeed, targetSpeed;
uint8_t START, heartRate;
UTC_t UTC;
char profileName[16];
uint8_t calibrationState, mode;
int16_t offset;
float batteryLevel;
uint8_t GPSComm, SDComm = 1; // SDComm should default to not display

/* **********************************************/
/* ***************** SETUP() *******************/

void setup() 
{
#ifdef ArduCAM328
  pinMode(10, OUTPUT); // USB ArduCam Only
#endif
  pinMode(MAX7456_SELECT,  OUTPUT); // OSD CS

  Serial.begin(TELEMETRY_SPEED);

#ifdef membug
  Serial.println(freeMem());
#endif

  // Prepare OSD for displaying 
  unplugSlaves();
  osd.init();

  // Start 
  startPanels();
  delay(500);

  // OSD debug for development (Shown at start)
#ifdef membug
  osd.setPanel(1,1);
  osd.openPanel();
  osd.printf("%i",freeMem()); 
  osd.closePanel();
#endif
  /*
    // Check EEPROM to for a new version that needs EEPROM reset
   if(readEEPROM(CHK_VERSION) != VER) {
   osd.setPanel(3,9);
   osd.openPanel();
   osd.printf_P(PSTR("EEPROM mapping outdated!|Update with the OSD Tool.")); 
   osd.closePanel();
   // run for ever until EEPROM version is OK 
   for(;;) {}
   }
   */
  // Get correct panel settings from EEPROM
  readSettings();
  for(panel = 0; panel < npanels; panel++) readPanelSettings();
  panel = 0; //set panel to 0 to start in the first navigation screen
  // Show bootloader bar
  loadBar();

  // Startup timers  
  Timer.Set(&OnTimer, 120);

  // House cleaning, clear display and enable timers
  osd.clear();
  Timer.Enable();

} // END of setup();



/* ***********************************************/
/* ***************** MAIN LOOP *******************/

// Mother of all happenings, The loop()
// As simple as possible.
void loop() 
{
  /*
  if(0){
   	    osd.clear();
   osd.setPanel(3,10);
   osd.openPanel();
   osd.printf_P(PSTR("Requesting DataStreams...")); 
   osd.closePanel();
   //for(int n = 0; n < 3; n++){
   //    request_mavlink_rates();//Three times to certify it will be readed
   delay(50);
   		}
   }
   */
  //TEST
  //writePanels();

  slipLen = SlipReceive(slipBuffer, &Serial);
  if (slipLen>0){
    Serial.println(slipLen);
    OSD_SlipParse(slipBuffer);
    slipBuffer[slipLen] = 0; // Is this just the null character? Isn't this unsafe?
    //memcpy((void*)completedPacket, (void*)slipBuffer, slipLen+1);
    Serial.println(slipBuffer);
    //Serial.println(completedPacket);
  } 
  Timer.Run(); // writes panels every 120 ms

}

/* *********************************************** */
/* ******** functions used in main loop() ******** */
void OnTimer()
{
  setHeadingPatern();  // generate the heading patern

    //osd_battery_pic_A = setBatteryPic(osd_battery_remaining_A);     // battery A remmaning picture
  //osd_battery_pic_B = setBatteryPic(osd_battery_remaining_B);     // battery B remmaning picture

  //setHomeVars(osd);   // calculate and set Distance from home and Direction to home

  writePanels();       // writing enabled panels (check OSD_Panels Tab)
}


void unplugSlaves(){
  //Unplug list of SPI
#ifdef ArduCAM328
  digitalWrite(10,  HIGH); // unplug USB HOST: ArduCam Only
#endif
  digitalWrite(MAX7456_SELECT,  HIGH); // unplug OSD
}

