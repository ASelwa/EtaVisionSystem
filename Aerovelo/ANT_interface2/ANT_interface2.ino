#include "antmessage.h"
#include <math.h>

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
  
  // Set up ANT+ reset pin.
  pinMode(3, OUTPUT);
  digitalWrite(3, LOW);
  delayMicroseconds(1000);  
  digitalWrite(3, HIGH);
  Serial.print("Set up ANT+ reset pin \n");

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

void readPowerMeter(uint8_t *pwrRx, uint8_t print, uint16_t *time_interval, float* power, float* cadence_out, bool* coast) {
	
	uint8_t i;
	static uint8_t prev_data[12] = {0};
	static uint16_t offset = 507;
	static int last_msg_time = 0;
	uint16_t temp_time = 0;
	float cadence = 0;
	float torque_freq = 0;
	float torque = 0;
	float pwr = 0;
	uint16_t tc = 0;
	static bool first_data = true;
	
	// exit function if not a power meter broadcast message on channel 0
	if (pwrRx[0] != 0x9 || pwrRx[1] != 0x4E || pwrRx[2] != 0x0) {
		*power = 0;
		*time_interval = 0;
		return;
	}
	
	if (print == 2) {
		Serial.print("Time: ");
		Serial.println(millis());
		for (i=0;i<12;i++) {
			Serial.print(pwrRx[i], HEX);
			Serial.print(" ");
		} Serial.print("\n");
		
	}
	switch(pwrRx[3]) {
	
		case 0x20: // broadcast pwr data page

			// check for duplicate messages (equal time stamps)
			if ((pwrRx[7] != prev_data[7]) && (pwrRx[8] != prev_data[8]) && (pwrRx[1] == 0x4E)) {
							
				// 9 4E 0 20 1 0 CC B B 2 60 6C 
				// [0] msglen, [1] msg type, [2] ch #, [3] data page #, [4] cadence event counter 
				// [5:6] slope, [7:8] time stamp, [9:10] torque ticks, [11] chksum					
						
				// if not first data, and prev_data is 9 4E ch# 20 ... ie not corrupted
				if (!first_data && prev_data[0] == 0x09 && prev_data[1] == 0x4E && prev_data[3] == 0x20) {
					if (*coast) {
						temp_time = (millis() - last_msg_time)*2;
					} else if ( cB(pwrRx[7], pwrRx[8]) < cB(prev_data[7], prev_data[8])) {
						temp_time = (65536-cB(prev_data[7], prev_data[8])) + cB(pwrRx[7],pwrRx[8]);
						//Serial.print("\ntemp_time Rollover!\n");
					} else {
						temp_time = (cB(pwrRx[7], pwrRx[8]) - cB(prev_data[7], prev_data[8]));
					}
					if (pwrRx[4] < prev_data[4]) {
						cadence = (60 / 0.0005f) * ((256 - prev_data[4])+pwrRx[4]) / (temp_time);
						//Serial.print("\ncadence Rollover!\n\n");
					} else {
						cadence = (60 / 0.0005f) * (pwrRx[4] - prev_data[4]) / (temp_time);
					} 
					uint16_t torque_tick = 0;
					if ( cB(pwrRx[9], pwrRx[10]) < cB(prev_data[9], prev_data[10])) {
						torque_tick = (65536-cB(prev_data[9], prev_data[10])) + cB(pwrRx[9],pwrRx[10]);
						//Serial.print("\ntorque_tick Rollover!\n");
					} else {
						torque_tick = (cB(pwrRx[9], pwrRx[10]) - cB(prev_data[9], prev_data[10]));
					}
					
					torque_freq = ((1.0f * torque_tick) / (temp_time * .0005)) - offset;
					torque = (torque_freq * 10) / cB(pwrRx[5], pwrRx[6]);
					pwr = torque * cadence * 3.14159f / 30;
				
					if (print == 1) {
						Serial.print("Time stamp: ");
						Serial.print(cB(pwrRx[7], pwrRx[8]));
						Serial.print("\tTime diff: ");
						Serial.print(temp_time);
						Serial.print("\tCadence Event #: ");
						Serial.print(pwrRx[4]);
						Serial.print("\tTorque tick diff: ");
						Serial.print(torque_tick);
						Serial.print("\tTorque Tick Count: ");
						Serial.print(cB(pwrRx[9], pwrRx[10]));
						Serial.print("\tCadence: ");
						Serial.print(cadence, 2);
						Serial.print("RPM\tTorque: ");
						Serial.print(torque, 2);
						Serial.print("Nm \tPower: ");
						Serial.print(pwr, 2);
						Serial.println(" W");
						Serial.flush();
					}
				} else { // first data point, so don't calculate anything
					first_data = false; 
				}
				// copy received data in prev_data array
				for (i = 0; i < 12; i++) {
					prev_data[i] = pwrRx[i];
				}
				// received new power data message, so must not be coasting anymore
				*coast = false;
			}
			break;
		
		case 0x1: //broadcast calibration data page						
			
			//offset = cB(pwrRx[9], pwrRx[10]);
			if (print) {
				Serial.print("Calibration offset: ");
				Serial.print(cB(pwrRx[9], pwrRx[10]));
				Serial.println("Hz.");
			}

			last_msg_time = millis();
			*coast = true;
			break;
		
		default:

			last_msg_time = millis();
			*coast = true;
			break;
			
		// end of case statement
	} // else do nothing
	*power = pwr;
	*time_interval = temp_time;
	*cadence_out = cadence;
}


float elevations[19][2] = {
	{0, 1470.75144}, 
	{182.88, 1468.8312},
	{563.88, 1463.25336},
	{792.48, 1461.42456},
	{960.12, 1460.72352},
	{1310.64, 1458.28512},
	{1630.68, 1456.60872},
	{1950.72, 1454.68848},
	{2240.28, 1453.56072},
	{2697.48, 1450.45176},
	{2910.84, 1449.35448},
	{3215.64, 1447.22088},
	{4114.8, 1439.84472},
	{4465.32, 1436.5224},
	{5760.72, 1427.25648},
	{6141.72, 1424.20848},
	{7025.64, 1418.6916},
	{7467.6, 1416.80184},
	{8046.72, 1412.93088}
};

// Finds a linear approximation of the elevation based on key points
float getElevation(float distance) {
	int distIndex = 0;
	const uint8_t elevationsLength = 19;
	const float endSlope = -0.00715;
	float elev;
	
	// Simple extrapolation
	if (distance > elevations[elevationsLength-1][0]) {
		return elevations[elevationsLength-1][1] + (distance - elevations[elevationsLength-1][1]) * endSlope;
	}
	
	// Find correct index
	while (distance > elevations[distIndex][0]) {
		distIndex++;
	}
	
	if (distIndex == 0)
		elev = elevations[0][1];
	else
		elev = elevations[distIndex-1][1] + (distance - elevations[distIndex-1][0]) * (elevations[distIndex][1] - elevations[distIndex-1][1]) / (elevations[distIndex][0] - elevations[distIndex-1][0]);
	
	return elev;
}

void simulate(float power, uint16_t time_interval, uint8_t print) {

	// Simulation Code
	uint8_t rider_mass = 80;
	uint8_t bike_mass = 20;
	float drag_coefficient = 0.016; // CdA
	float Crr_one = 0.00273;
	float Crr_two = 0.000097;
	static float velocity = 0;
	static float distance = 0;
	float power_in = 0;
	float energy_in = 0;
	float energy_left = 0;
	float efficiency = 0.92; 
	float power_interval = time_interval * 0.0005;
	float change_elev = 0;
	uint16_t total_mass = rider_mass + bike_mass;
	
	// check for bad input (power can't be negative)
	if (power > 0) {
	
		power_in = power * efficiency;	// watts (J/s)
		energy_in = power_in * power_interval;	// energy (joules) input since previous measurement
	
		energy_left = energy_in 	/*rolling friction*/ - (Crr_one + Crr_two*velocity)*(total_mass*9.8)*(velocity*power_interval)
									/*air drag*/ - 0.5*1*(velocity*velocity)*drag_coefficient*(velocity*power_interval)
									/*elevation*/ - total_mass*9.8*change_elev;
									/*rolling mass*/
	
		float prev_velo = velocity;
	
		if (energy_left < 0)	velocity = velocity - sqrt(2*-1*energy_left/(total_mass));
		else velocity = velocity + sqrt(2*energy_left/(total_mass));
		
		distance = distance + 0.5*(prev_velo + velocity)* power_interval;

		if (print) {
			Serial.print("Power: ");
			Serial.print(power);
			Serial.print(" W.\tPower interval: ");
			Serial.print(power_interval);
			Serial.print(" s.\tEnergy in: ");
			Serial.print(energy_in);
			Serial.print(" J.\tVelocity: ");
			Serial.print(velocity*3.6); // convert to km/h
			Serial.print(" km/h.\tDistance: ");
			Serial.print(distance*1.0);
			Serial.print(" m.\tNet energy: ");
			Serial.print(energy_left);
			Serial.println(" J.");
			Serial.flush();
		}
	}
}

void simulate2(float power, uint16_t time_interval, uint8_t print, float* velo, float* dist) {
	uint16_t t1, t2 = 0;
	t1 = millis();
	if (power == 0 && time_interval == 0) return;

	static float velocity = 0;
	static float distance = 0;	
	
	if (isnan(velocity)) {
		Serial.print("static variable velocity in function simulate2 is NAN. velocity passed in as a parameter is ");
		Serial.println(*velo);
		velocity = *velo;
	}

	// constants
	float g = 9.81;	// m/s
	float mu = 0.0000185;	// Pa*s
	uint8_t rho = 1;	// kg/m^3 for battle mountain
	
	// rolling parameters
	uint16_t M = /*pilot*/ 80 + /*hpv*/ 20 + /*wheels*/ 2; // total mass, kg
	float Crr_one = 0.0015;	
	float Crr_two = 2/3*0.000041*3.6; // s/m
	
	// aerodynamic parameters
	float L = 2.88; // m
	float Lnose = 1.00; // m 
	float h = 0.75; // m 
	float w = 0.45; // m 
	float xt = 1.90; // transition point, m 
	float Af = 3.14159*(h/2)*(w/2); // frontal area, m^2
	float Aside = 0.5*3.14159*Lnose*(h/2) + (L-Lnose)*h;	// side view area
	float Awet = (0.037*Af*Af + 0.02*Af + 1)*Aside*2;	// wetted area
	
	// drivetrain efficiency
	float etaD = 0.96334; // %
	
	// rolling resistance
	float Proll;
	if (velocity == 0) Proll = 0;
	else Proll = velocity*M*g*(Crr_one + Crr_two*velocity);
	
	/**** aerodynamic drag ****/
	float q = 0.5*rho*pow(velocity,2); // dynamic pressure
	// flat plate resistance
	float Dflam = 1.328*h*q*sqrt(mu/velocity/rho)*sqrt(xt);	// laminar drag
	float deltalamxt = 5*sqrt(mu/velocity/rho)*sqrt(xt);	// lam BL thickness at xt
	float deltaturbxt = 0.13/0.097*deltalamxt;	// turb BL thickness at xt
	float xdel = deltaturbxt/0.375*pow(pow(velocity*rho/mu,2),1/0.8);
	float xzero = xt - xdel;	// imaginary turb start
	float Dfturb = 0.0576/0.8*h*q*pow(mu/velocity/rho,0.2)*(pow(L-xzero,0.8) - pow(xdel,0.8));
	float Cfflat = (Dflam + Dfturb)/(q*h*L);

	// body drag
	float Cdwet = Cfflat*(1 + 1.8*pow(Af,0.75)/pow(L,1.5) + 39*pow(Af,3)/pow(L,6));
	float CdAbody = Cdwet*Awet;
	
	// total aerodynamic drag
	float CdA = CdAbody + /*CdAfwheel*/ 0.002 + /*CdArwheel*/ 0.003 + /*CdAunclean*/ 0.001;
	float Paero;
	if (velocity == 0) Paero = 0;
	else Paero = q*CdA*velocity;
	
	if (isnan(velocity)) {
		Serial.print("static variable velocity in function simulate2 is NAN. (check #2) velocity passed in as a parameter is ");
		Serial.println(*velo);
		velocity = *velo;
	}
	
	float prev_velo = velocity;	// for average speed calculation used in distance formula
	float prev_dist = distance; // for Pelev
	float power_left = 0;
	float power_interval = time_interval * 0.0005; // in seconds

	// change in elevation
	float change_elev, Pelev, prev_Pelev = 0; 

	float delta_v, delta_d;
	/***************** subtract the powers added in the previous iteration first *******************/
	// check for bad input (power can't be negative)
	if (power >= 0) {
	
		float power_in = power * etaD;	// watts (J/s)
		//energy_in = power_in * power_interval;	// energy (joules) input since previous measurement
		
		bool calc_dist = true;
		uint8_t count = 0;
		float elev_calc[10] = {0};
		while (calc_dist) {
		
			power_left = power_in 	/*rolling friction*/ - Proll
									/*air drag*/ - Paero
									/*elevation change*/ + Pelev;
		
			if (power_left < 0)	delta_v = (-1)*sqrt(2*(-1)*power_left*power_interval/(M + /*Mwheels*/ 2));
			else if (power_left == 0) delta_v = 0;
			else delta_v = sqrt(2*power_left*power_interval/(M + /*Mwheels*/ 2));
			
			delta_d = 0.5*(prev_velo*2 + delta_v)*power_interval;	
		
			velocity = prev_velo + delta_v;
			distance = prev_dist + delta_d;
		
			change_elev = getElevation(distance)-getElevation(prev_dist); // guess elevation change using previous velocity and time travelled
			prev_Pelev = Pelev;
			Pelev = M*(-g)*change_elev/power_interval; //kgm2/s3  kg*m/s2 * m / s 
			
			elev_calc[count] = Pelev;
			count++;
			
			if (abs(Pelev - prev_Pelev) <= 1) calc_dist = false;
			else if (count > 10) calc_dist = false;
			
		}
		for (i=0;i<10;i++) {
			Serial.print(elev_calc[i]);
			Serial.print("W.\t");
		}	Serial.print("\n");
		
		if (velocity < 0) velocity = 0;
		
		t2 = millis();
		
		if (print == 1) {
			Serial.print("Power: ");
			Serial.print(power);
			Serial.print(" W.\tPower interval: ");
			Serial.print(power_interval);
			Serial.print(" s.\tEnergy in: ");
			Serial.print(power_in*power_interval);
			Serial.print(" J.\tVelocity: ");
			Serial.print(velocity*3.6); // convert to km/h
			Serial.print(" km/h.\tDistance: ");
			Serial.print(distance*1.0);
			Serial.print(" m.\tNet energy: ");
			Serial.print(power_left*power_interval);
			Serial.println(" J.");
			/*Serial.print("Compute time = ");
			Serial.println(t2-t1);*/
		} else if (print == 2) {
			Serial.print("Power in: ");
			Serial.print(power_in);
			Serial.print(" W.\tPower interval: ");
			Serial.print(power_interval);
			Serial.print(" s.\tRolling Power: ");
			Serial.print(Proll);
			Serial.print(" W.\tAir Power: ");
			Serial.print(Paero);
			Serial.print(" W.\tElev Power: ");
			Serial.print(Pelev);
			Serial.print(" W.\tPower left: ");
			Serial.print(power_left);
			Serial.print(" W.\tVelocity: ");
			Serial.print(velocity*3.6);
			Serial.print(" km/h.\tDistance: ");
			Serial.print(distance*1.0);
			Serial.print(" m.\n");
		} else if (print == 3) {
			Serial.print("Velocity = ");
			Serial.print(velocity*3.6);
			Serial.print(" km/h\n");
		}
		if (print) Serial.flush();
	}
	
	*dist = distance;
	*velo = velocity;
}

void loop() {
	
	float power, cadence, velocity, distance = 0;
	uint16_t time_int = 0;
	uint16_t t2 = 0; 

	/*
	while(1){
		if (Serial1.available()) {
			if ((m = receiveANT(rxBuffer)) > 0){
				if (m == 9) {
					readPowerMeter(rxBuffer, 0, &time_int, &power);
					simulate2(power, time_int, 1);
				}
			}
		}
	}*/
	
	uint8_t len = 0;
	bool coast = false;
	while(1){
		if (Serial1.available()) {
			if (len = receiveANT(rxBuffer) > 0) {
				readPowerMeter(rxBuffer, 0, &time_int, &power, &cadence, &coast);
				if (!coast && power != 0 && time_int != 0) {
					simulate2(power, time_int, 2, &velocity, &distance); 
					t2 = millis();	// last power meter data message received
				} else if (coast) {
					simulate2(0, 2*(millis()-t2) , 2, &velocity, &distance);
					t2 = millis();
				}
			}
		}
	}
}






