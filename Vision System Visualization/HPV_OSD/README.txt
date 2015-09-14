++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
			HPV Vision and Data Acquisition System - MinimOSD
				
By Oleksiy Ryndin and Sherry Shi
Last update: May 6, 2014
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

For hardware specifications of the MinimOSD board, visit https://code.google.com/p/arducam-osd/wiki/minimosd

The code for the MinimOSD board is adapted from: https://code.google.com/p/arducam-osd/

ArduCam_Max7456.h and ArduCam_Max7456.cpp
	OSD class, with various setup and usage methods.
	The structure of the class is based on "panels", which are blocks of 
	text/symbols on	the screen. To write to the screen:
		1. Call osd.setPanel() with a specified position
		2. Call osd.openPanel().
		3. Call osd.printf_P(PSTR("INSERT TEXT HERE"))
		4. Call osd.closePanel().
	
ArduNOTES.cpp
	Included with the ArduCAM code. Not used in the main code.
	
BOOT_Func.ino
	OSD boot screen.
	
Font.ino
	Updates the character set (included with original ArduCAM code, not used 
	in main code).
	
HPV_OSD.ino
	Main code. Periodically updates the screen on a timer.
	
OSD_Config.h and OSD_Vars.h
	Macros and variables. Part of the original ArduCAM code. Mostly not	used.
	
OSD_Config_Func.ino
	Configuration functions.
	
OSD_Func.h
	Miscellaneous OSD functions. Part of the original ArduCAM code. Mostly not 
	used.

OSD_Panels_HPV.ino
	Sets up all the panels to be displayed. 
	writePanels() is called by the main loop periodically to display all the 
	panels onto the screen.
	
OSD_SLIP.h and OSD_SLIP.cpp
	Parses received SLIP data.

slip.h and slip.cpp
	Serial Line Internet Protocol (SLIP) functions.
	
SPIcopy.h and SPIcopy.cpp
	Not used.