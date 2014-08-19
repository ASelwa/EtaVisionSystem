#include "simulation.h"

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
  if (distance > elevations[elevationsLength - 1][0]) {
    return elevations[elevationsLength - 1][1] + (distance - elevations[elevationsLength - 1][1]) * endSlope;
  }

  // Find correct index
  while (distance > elevations[distIndex][0]) {
    distIndex++;
  }

  if (distIndex == 0)
    elev = elevations[0][1];
  else
    elev = elevations[distIndex - 1][1] + (distance - elevations[distIndex - 1][0]) * (elevations[distIndex][1] - elevations[distIndex - 1][1]) / (elevations[distIndex][0] - elevations[distIndex - 1][0]);

  return elev;
}

uint16_t cB(const uint8_t x, const uint8_t y) {
  return (x * 256 + y);
}

static uint16_t offset = 507;
void setOffset(int off) {
  offset = off;
}

uint16_t getOffset() {
  return offset;
}

void readPowerMeter(uint8_t *pwrRx, uint8_t print, uint16_t *time_interval, float* power, float* cadence_out, bool* coast) {

  uint8_t i;
  static uint8_t prev_data[12] = {0};
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
    Serial.print("RESET POWER");
    return;
  }

  switch (pwrRx[3]) {

    case 0x20: // broadcast pwr data page

      // check for duplicate messages (equal time stamps)
      if ((pwrRx[7] != prev_data[7]) && (pwrRx[8] != prev_data[8]) && (pwrRx[1] == 0x4E)) {

        // 9 4E 0 20 1 0 CC B B 2 60 6C
        // [0] msglen, [1] msg type, [2] ch #, [3] data page #, [4] cadence event counter
        // [5:6] slope, [7:8] time stamp, [9:10] torque ticks, [11] chksum

        // if not first data, and prev_data is 9 4E ch# 20 ... ie not corrupted
        if (!first_data && prev_data[0] == 0x09 && prev_data[1] == 0x4E && prev_data[3] == 0x20) {
          if (*coast) {
            temp_time = (millis() - last_msg_time) * 2;
          } else if ( cB(pwrRx[7], pwrRx[8]) < cB(prev_data[7], prev_data[8])) {
            temp_time = (65536 - cB(prev_data[7], prev_data[8])) + cB(pwrRx[7], pwrRx[8]);
            //Serial.print("\ntemp_time Rollover!\n");
          } else {
            temp_time = (cB(pwrRx[7], pwrRx[8]) - cB(prev_data[7], prev_data[8]));
          }

          if (pwrRx[4] < prev_data[4]) {
            cadence = (60 / 0.0005f) * ((256 - prev_data[4]) + pwrRx[4]) / (temp_time);
            //Serial.print("\ncadence Rollover!\n\n");
          } else {
            cadence = (60 / 0.0005f) * (pwrRx[4] - prev_data[4]) / (temp_time);
          }

          uint16_t torque_tick = 0;
          if ( cB(pwrRx[9], pwrRx[10]) < cB(prev_data[9], prev_data[10])) {
            torque_tick = (65536 - cB(prev_data[9], prev_data[10])) + cB(pwrRx[9], pwrRx[10]);
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

        *power = pwr;
        *cadence_out = cadence;
      } else if (first_data && *coast) {
        // copy received data in prev_data array
        for (i = 0; i < 12; i++) {
          prev_data[i] = pwrRx[i];
        } 
      }
      break;

    case 0x1: //broadcast calibration data page

      //offset = cB(pwrRx[9], pwrRx[10]);
      if (print == 2) {
        Serial.print("Calibration offset: ");
        Serial.print(cB(pwrRx[9], pwrRx[10]));
        Serial.println("Hz.");
      }

      last_msg_time = millis();
      first_data = true; // Skip the first calculation coming out of a coast
      *coast = true;
      *power = pwr;
      *cadence_out = cadence;

      break;

    default:

      last_msg_time = millis();
      *coast = true;
      *power = pwr;
      *cadence_out = cadence;
      break;

      // end of case statement
  } // else do nothing
  *time_interval = temp_time;
}

void simulate(float power, uint16_t time_interval, uint8_t print, float* velo, float* dist) {
  uint16_t t1, t2 = 0;
  t1 = millis();
  if (power == 0 && time_interval == 0) return;

  float velocity = *velo;
  float distance = *dist;

  if (isnan(velocity)) {
    Serial.print("static variable velocity in function simulate is NAN. velocity passed in as a parameter is ");
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
  float Crr_two = 2.0 / 3 * 0.000041 * 3.6; // s/m

  // aerodynamic parameters
  float L = 2.88; // m
  float Lnose = 1.00; // m
  float h = 0.75; // m
  float w = 0.45; // m
  float xt = 1.90; // transition point, m
  float Af = 3.14159 * (h / 2) * (w / 2); // frontal area, m^2
  float Aside = 0.5 * 3.14159 * Lnose * (h / 2) + (L - Lnose) * h;	// side view area
  float Awet = (0.037 * Af * Af + 0.02 * Af + 1) * Aside * 2;	// wetted area

  // drivetrain efficiency
  float etaD = 0.96334; // %

  // rolling resistance
  float Proll;
  if (velocity == 0) Proll = 0;
  else Proll = velocity * M * g * (Crr_one + Crr_two * velocity);

  /**** aerodynamic drag ****/
  float Paero;
  float q = 0.5 * rho * pow(velocity, 2); // dynamic pressure
  
  // flat plate resistance
  float Dflam = 1.328 * h * q * sqrt(mu / velocity / rho) * sqrt(xt);	// laminar drag
  float deltalamxt = 5 * sqrt(mu / velocity / rho) * sqrt(xt);	// lam BL thickness at xt
  float deltaturbxt = 0.13 / 0.097 * deltalamxt;	// turb BL thickness at xt
  float xdel = deltaturbxt / 0.375 * pow(pow(velocity * rho / mu, 0.2), 1 / 0.8);
  float xzero = xt - xdel;	// imaginary turb start
  float Dfturb = 0.0576 / 0.8 * h * q * pow(mu / velocity / rho, 0.2) * (pow(L - xzero, 0.8) - pow(xdel, 0.8));
  float Cfflat = (Dflam + Dfturb) / (q * h * L);

  // body drag
  float Cdwet = Cfflat * (1 + 1.8 * pow(Af, 0.75) / pow(L, 1.5) + 39 * pow(Af, 3) / pow(L, 6));
  float CdAbody = Cdwet * Awet;

  // total aerodynamic drag
  float CdA = CdAbody + /*CdAfwheel*/ 0.002 + /*CdArwheel*/ 0.003 + /*CdAunclean*/ 0.001;
  if (velocity == 0) Paero = 0;
  else Paero = q * CdA * velocity;
  
  Serial.print("CdA: "); Serial.println(CdA);
  
  if (isnan(CdA)) {
    Paero = 0;
    Serial.println("CdA is NaN.");
    Serial.print("q: "); Serial.println(q); Serial.print("CdA: "); Serial.println(CdA); Serial.print("velocity: "); Serial.println(velocity);
  }


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

      // Method 2
      float net_energy = power_left * power_interval + 0.5 * (M+2) * pow(prev_velo, 2); // Don't use _velocity_ variable: this double counts the energy increase from each iteration
      if (net_energy < 0) velocity = 0;
      else velocity = sqrt(2 * net_energy / (M+2));

      distance = prev_dist + 0.5 * (prev_velo + velocity) * power_interval;
      // end Method 2

      change_elev = getElevation(distance) - getElevation(prev_dist); // guess elevation change using previous velocity and time travelled
      prev_Pelev = Pelev;
      Pelev = M * (-g) * change_elev / power_interval; //kgm2/s3  kg*m/s2 * m / s

      elev_calc[count] = Pelev;
      count++;
      
//      if (isinf(Pelev)) {
//        Serial.println("Pelev is infinity");
//        Serial.print("Change_elev: "); Serial.println(change_elev);
//        Serial.print("Power interval: "); Serial.println(power_interval);
//        
//        char sdBuffer[32];
//        sd_Log("Pelev is infinity. ");
//        sd_Log("Change_elev: "); sd_Log(dtoa(sdBuffer, change_elev));
//        sd_Log(" Power interval: "); sd_Log(dtoa(sdBuffer, power_interval));
//        
//        // Stop iterating
//        Pelev = prev_Pelev;
//      }

      if (abs(Pelev - prev_Pelev) <= 1) calc_dist = false;
      else if (count > 9) calc_dist = false;

    }

	/*for (uint8_t i=0;i<10;i++) {
    	Serial.print(elev_calc[i]);
    	Serial.print("W.\t");
    }	Serial.println;*/

    if (velocity < 0) velocity = 0;

    //t2 = millis();

    if (print == 1) {
      Serial.print("Power: ");
      Serial.print(power);
      Serial.print(" W.\tPower interval: ");
      Serial.print(power_interval);
      Serial.print(" s.\tEnergy in: ");
      Serial.print(power_in * power_interval);
      Serial.print(" J.\tVelocity: ");
      Serial.print(velocity * 3.6); // convert to km/h
      Serial.print(" km/h.\tDistance: ");
      Serial.print(distance * 1.0);
      Serial.print(" m.\tNet energy: ");
      Serial.print(power_left * power_interval);
      Serial.print (" J.\n");
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
      Serial.print(velocity * 3.6);
      Serial.print(" km/h.\tDistance: ");
      Serial.print(distance * 1.0);
      Serial.print(" m.\n");
    } else if (print == 3) {
      Serial.print("Velocity = ");
      Serial.print(velocity * 3.6);
      Serial.print(" km/h\n");
    }
  }

  *dist = distance;
  *velo = velocity;
}
