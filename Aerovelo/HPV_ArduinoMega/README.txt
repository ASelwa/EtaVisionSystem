++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
			HPV Vision and Data Acquisition System - Arduino Mega
				
By Oleksiy Ryndin and Sherry Shi
Last update: May 6, 2014
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

ANt_interface.cpp and ANT_intface.h
	Setup and usage functions for interacting with the ANT+ transceiver.
	
antmessage.h
	Defines various macros for the ANT+ transceiver (mostly unused).
	
GPS_Func.ino
	Setup and usage functions for the GPS.
	
GPS_UBLOX.cpp and GPS_UBLOX.h
	UBlox GPS functions.
	
HPV_ArduinoMega.h
	Not used.
	
HPV_ArduinoMega.ino
	Main program. Collects data from the various peripherals, encodes them and
	sends them to the OSD board using SLIP.
	
keywords.txt
	Not used.
	
SD_Func.ino
	SD card setup and usage functions.
	
slip.ccp and slip.h
	Serial Line Internet Protocol (SLIP) functions.
	
TargetSpeed.ino
	Calculates the target speed of a given rider from a 6-degree polynomial 
	with respect to distance (for Battle Mountain). Records the target speed 
	and actual speed on the SD card.
	

	