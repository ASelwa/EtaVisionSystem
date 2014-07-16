#include "antmessage.h"

#define SYNC 0xA4
#define PERIOD 2000

#define NET_KEY_ANTP {0xB9, 0xA5, 0x21, 0xFB, 0xBD, 0x72, 0xC3, 0x45, 0x00}
#define NET_KEY_PUBLIC {0xE8, 0xE4, 0x21, 0x3B, 0x55, 0x7A, 0x67, 0xC1, 0x00}

uint8_t buffer[32] = { 
  0};

uint8_t rxBuffer[32] = {
  0};


uint32_t TIME;

void ANT_SendMessage(uint8_t *buf, uint8_t n){
  uint8_t i;

  buf[n] = 0;
  for (i = 0; i < n; ++i){
    buffer[n] ^= buffer[i];
  }
  buffer[n+1] = 0;
  buffer[n+2] = 0;
  Serial1.write(buffer, n+3);
}

void ANT_SetupChannel(uint8_t channel){

  const char key[] = NET_KEY_ANTP;

  uint8_t chType = 0x00;  // Bidirectional slave.
  //uint8_t chType = 0x40;  // Receive only slave.


  // Set Network Key
  buffer[0] = MESG_TX_SYNC;
  buffer[1] = MESG_NETWORK_KEY_SIZE;
  buffer[2] = MESG_NETWORK_KEY_ID;
  buffer[3] = 0;
  strcpy((char*)(buffer+4), key);

  ANT_SendMessage(buffer, 12);
  delayMicroseconds(1000);
  
  // Assign channel.
  buffer[0] = MESG_TX_SYNC;
  buffer[1] = MESG_ASSIGN_CHANNEL_SIZE;
  buffer[2] = MESG_ASSIGN_CHANNEL_ID;
  buffer[3] = channel;
  buffer[4] = chType;
  buffer[5] = 0;      // Network number, 0 = public.

  ANT_SendMessage(buffer, 6);
  delayMicroseconds(1000);

  // Assign channel ID. 121 device ID for speed sensor.
  buffer[0] = MESG_TX_SYNC;
  buffer[1] = MESG_CHANNEL_ID_SIZE;
  buffer[2] = MESG_CHANNEL_ID_ID;
  buffer[3] = channel;
  //*((uint16_t*)(buffer+5)) = 20484;      // Device number, 0 is wildcard.
  *((uint16_t*)(buffer+5)) = 0;		// set for searching mode *NEED TO SAVE DEVICE NUMBER IN FUTURE*
  //buffer[6] = 0b00000000 + 121;      // Pairing bit + device type. 0 is wildcard
  buffer[6] = 0b00000000 + 11;		// 11 or 0x0B for bike power sensor
  buffer[7] = 0x00;      // Transmission type *FOR PAIRING MODE, NEED TO SAVE TRANSMISSION TYPE IN FUTURE*

  ANT_SendMessage(buffer, 8);
  delayMicroseconds(1000);
  
  // Set period.
  buffer[0] = MESG_TX_SYNC;
  buffer[1] = MESG_CHANNEL_MESG_PERIOD_SIZE;
  buffer[2] = MESG_CHANNEL_MESG_PERIOD_ID;
  buffer[3] = channel;
  //*((uint16_t*)(buffer+4)) = 8090;
  *((uint16_t*)(buffer+4)) = 8182;

  ANT_SendMessage(buffer, 6);
  delayMicroseconds(1000);
  
  // Set frequency.
  buffer[0] = MESG_TX_SYNC;
  buffer[1] = MESG_CHANNEL_RADIO_FREQ_SIZE;
  buffer[2] = MESG_CHANNEL_RADIO_FREQ_ID;
  buffer[3] = channel;
  buffer[4] = ANTPLUS_RF_FREQ;

  ANT_SendMessage(buffer, 5);
  delayMicroseconds(1000);
  
  // Open channel.
  buffer[0] = MESG_TX_SYNC;
  buffer[1] = MESG_OPEN_CHANNEL_SIZE;
  buffer[2] = MESG_OPEN_CHANNEL_ID;
  buffer[3] = channel;

  ANT_SendMessage(buffer, 4);
  delayMicroseconds(1000);

  // Open RX Scan Mode.
  buffer[0] = MESG_TX_SYNC;
  buffer[1] = 1;
  buffer[2] = 0x5B;
  buffer[3] = 0;

  ANT_SendMessage(buffer, 4);
  delayMicroseconds(1000);


  // Request data.
  buffer[0] = MESG_TX_SYNC;
  buffer[1] = MESG_REQUEST_SIZE;
  buffer[2] = MESG_REQUEST_ID;
  buffer[3] = channel;
  buffer[4] = MESG_CHANNEL_ID_ID;

  ANT_SendMessage(buffer, 5);
  delayMicroseconds(1000);

  Serial.print("Set up Channel. \n");


}



void receiveInterrupt(){
  Serial.println("Receive successful");
}


uint8_t receiveANT(uint8_t *rxBuf){
  uint8_t temp = Serial1.read();
  static uint8_t rxState = 0;
  static uint8_t msgLen;
  static uint8_t msgType;
  uint8_t chksum;
  static uint8_t index;

  int8_t result = 0;

  switch (rxState){
  case 0:  // Searching for 0xA4 or 0xA5
    if (temp == 0xA4) rxState = 1;
	// result (ie. m) is still 0 so loop back to here
    break;
  case 1:  // 0xA4 received (sync byte), reading message length
    index = 0;
    msgLen = temp;
    rxBuf[index++] = temp;
    rxState = 2;
	// result (ie. m) is still 0 so loop back to here
    break;
  case 2:	// reading message ID
    rxBuf[index++] = temp;
    rxState = 3;
	// result (ie. m) is still 0 so loop back to here
    break;
  case 3:	// reading message, don't change rxState until whole message is read
    rxBuf[index++] = temp;
    if (index > (msgLen+2)) rxState = 4;
	// result (ie. m) is still 0 so loop back to here
    break;
  case 4:	// reading checksum
    chksum = temp;
    rxBuf[index++] = temp;
    rxBuf[index] = 0;
    result = msgLen;
    rxState = 0;
    break;

  default:; 

  }

  return result;


}

void setup() {  

  Serial.begin(115200);
  // Set up CTS interrupt.
  
  //pinMode(2, INPUT);
  //attachInterrupt(0, receiveInterrupt, RISING);
  //Serial.print("Set up CTS interrupt. \n");

  // Set up ANT+ reset pin.
  pinMode(3, OUTPUT);
  digitalWrite(3, LOW);
  delayMicroseconds(1000);  
  digitalWrite(3, HIGH);
  Serial.print("Set up ANT+ reset pin \n");
  
  // Testing pins
  pinMode(31, OUTPUT);
  pinMode(35, OUTPUT);
  pinMode(39, OUTPUT);
  pinMode(43, OUTPUT);
  digitalWrite(31, LOW);
  digitalWrite(35, LOW);
  digitalWrite(39, LOW);
  digitalWrite(43, LOW);

  // Select serial port.
  Serial1.begin(9600);
  //Serial2.begin(9600);
  Serial.begin(115200);
  Serial.print("Set up serial ports. \n");

  TIME = millis();

  Serial.println("Program start!");

  ANT_SetupChannel(0);
}

uint16_t cB(const uint8_t x, const uint8_t y) {
	return (x*256+y);
}


void loop() {

  
	uint8_t temp;
	int8_t i, m;
	static uint8_t prev_data[12] = {0};
	static uint16_t offset = 507;
	uint16_t temp_time = 0;
	float cadence = 0;
	float torque_freq = 0;
	float torque = 0;
	float power = 0;
	


	// put your main code here, to run repeatedly: 
	

	
	if (Serial1.available()){
		if ((m = receiveANT(rxBuffer)) > 0){
			if (m == 9) {
				// check for duplicate messages (equal time stamps)
				if ((rxBuffer[7] != prev_data[7]) && (rxBuffer[8] != prev_data[8]) && (rxBuffer[1] == 0x4E)) {	
					if (rxBuffer[3] == 0x20) {	// broadcast power data page
							
							// 9 4E 0 20 1 0 CC B B 2 60 6C 
							// [0] msglen, [1] msg type, [2] ch #, [3] data page #, [4] cadence event counter 
							// [5:6] slope, [7:8] time stamp, [9:10] torque ticks, [11] chksum					
						
						// if prev time stamp != 0, and prev_data is 09 4E __ 20 ... ie not corrupted
						if (cB(prev_data[7], prev_data[8]) != 0 && prev_data[0] == 0x09 && prev_data[1] == 0x4E && prev_data[3] == 0x20) {
					
							if ( cB(rxBuffer[7], rxBuffer[8]) < cB(prev_data[7], prev_data[8])) {
								temp_time = (65536-cB(prev_data[7], prev_data[8])) + cB(rxBuffer[7],rxBuffer[8]);
								//Serial.print("\ntemp_time Rollover!\n");
							} else {
								temp_time = (cB(rxBuffer[7], rxBuffer[8]) - cB(prev_data[7], prev_data[8]));
							}
							uint8_t temp_cadence = 0;
							if (rxBuffer[4] < prev_data[4]) {
								cadence = (60 / 0.0005f) * ((256 - prev_data[4])+rxBuffer[4]) / (temp_time);
								//Serial.print("\ncadence Rollover!\n\n");
							} else {
								cadence = (60 / 0.0005f) * (rxBuffer[4] - prev_data[4]) / (temp_time);
							} 
							uint16_t torque_tick = 0;
							if ( cB(rxBuffer[9], rxBuffer[10]) < cB(prev_data[9], prev_data[10])) {
								torque_tick = (65536-cB(prev_data[9], prev_data[10])) + cB(rxBuffer[9],rxBuffer[10]);
								//Serial.print("\ntorque_tick Rollover!\n");
							} else {
								torque_tick = (cB(rxBuffer[9], rxBuffer[10]) - cB(prev_data[9], prev_data[10]));
							}
					
							torque_freq = ((1.0f * torque_tick) / (temp_time * .0005)) - 506; // offset = 506
							torque = (torque_freq * 10) / cB(rxBuffer[5], rxBuffer[6]);
							power = torque * cadence * 3.14159f / 30;
						
							Serial.print("Time stamp: ");
							Serial.print(cB(rxBuffer[7], rxBuffer[8]));
							Serial.print("\tTime diff: ");
							Serial.print(temp_time);
							Serial.print("\tCadence Event #: ");
							Serial.print(rxBuffer[4]);
							Serial.print("\tTorque tick diff: ");
							Serial.print(torque_tick);
							Serial.print("\tTorque Tick Count: ");
							Serial.print(cB(rxBuffer[9], rxBuffer[10]));
							Serial.print("\tCadence: ");
							Serial.print(cadence, 2);
							Serial.print("RPM\tTorque: ");
							Serial.print(torque, 2);
							Serial.print("Nm \tPower: ");
							Serial.print(power, 2);
							Serial.println(" W");
							Serial.flush();
							Serial.print("Cadence Event #: ");
							Serial.print(rxBuffer[4]);
				
						} // else first data point, so don't calculate anything						
						for (i = 0; i < 12; i++) {
							prev_data[i] = rxBuffer[i];
						}
					} else if (rxBuffer[3] == 0x1) { //broadcast calibration data page
												
						/*offset = cB(rxBuffer[9], rxBuffer[10]);
						Serial.print("Calibration offset: ");
						Serial.print(offset);
						Serial.println("Hz.");*/

					}
				} // else do nothing
			} 
        }
	}
	// testing
	else digitalWrite(35, LOW);
	
	// Simulation Code
	uint8_t rider_mass = 80;
	uint8_t bike_mass = 20;
	float drag_coefficient = 0.016; // CdA
	float Crr_one = 0.00273;
	float Crr_two = 0.000097;
	static float velocity = 0;
	static float distance = 0;
	float prev_velocity = 0;
	float prev_distance = 0;
	float elevation = 0;
	float power_in = 0;
	float energy_in = 0;
	float energy_left = 0;
	float efficiency = 0.92; 
	float power_interval = temp_time * 0.0005;
	float friction = 0;
	float change_elev = 0;
	uint16_t total_mass = rider_mass + bike_mass;
	
	prev_velocity = velocity;
	prev_distance = distance;
	
	if (power > 0) {
	
		power_in = power * efficiency;	// watts (J/s)
		energy_in = power_in * power_interval;	// energy (joules) input since previous measurement
	
		energy_left = energy_in 	/*rolling friction*/ - (Crr_one + Crr_two*prev_velocity)*(total_mass*9.8)*(prev_velocity*power_interval)
									/*air drag*/ - 0.5*1*(prev_velocity*prev_velocity)*drag_coefficient*(prev_velocity*power_interval)
									/*elevation*/ - total_mass*9.8*change_elev;
									/*rolling mass*/
	
		if (energy_left < 0)	velocity = prev_velocity - sqrt(2*-1*energy_left/(total_mass));
		else velocity = prev_velocity + sqrt(2*energy_left/(total_mass));
		
		distance = prev_distance + 0.5*(prev_velocity + velocity)* power_interval;
		
		Serial.print("Power: ");
		Serial.print(power);
		Serial.print(" W.\tPower interval: ");
		Serial.print(power_interval);
		Serial.print(" s.\tEnergy in: ");
		Serial.print(energy_in);
		Serial.print(" J.\tVelocity: ");
		Serial.print(velocity*3.6); // convert to km/h
		Serial.print(" km/h.\tDistance: ");
		Serial.print(distance);
		Serial.print(" m.\tNet energy: ");
		Serial.print(energy_left);
		Serial.println(" J.");
		Serial.print("\tDistance: ");
		Serial.print(distance);
		Serial.print("m\n");
		Serial.print("Bytes Available: ");
		Serial.print(Serial1.available());
		Serial.print(" bytes.\n");
		Serial.flush();
	}

}






