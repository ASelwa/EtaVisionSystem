#include "simulation.h"

#define M 113.85
#define M_WHEELS 2.23
#define DRIVE_ETA 0.95
#define G 9.79778

float course_length = 8045; // metres
float sim_dt = 0;
float sim_vel = 0;
float sim_vel_new = 0;
float sim_dist = 0;
float sim_Pdrag = 0;
float sim_Pslope = 0;
float sim_Pnet = 0;

float PslopeArray[51] = {
  0.0051,
  0.004,
  0.0042,
  0.0066,
  0.0073,
  0.0066,
  0.0062,
  0.0049,
  0.0038,
  0.0046,
  0.006,
  0.0064,
  0.0062,
  0.0067,
  0.006,
  0.0065,
  0.0086,
  0.0082,
  0.007,
  0.0067,
  0.0069,
  0.0077,
  0.0079,
  0.0069,
  0.0071,
  0.0071,
  0.0094,
  0.0095,
  0.0074,
  0.0091,
  0.0091,
  0.0069,
  0.0086,
  0.008,
  0.0069,
  0.0055,
  0.0053,
  0.0067,
  0.0067,
  0.0063,
  0.0033,
  0.0047,
  0.0067,
  0.0054,
  0.0053,
  0.0062,
  0.0075,
  0.0049,
  0.0054,
  0.0111,
  0.0146 
};

float distArray[51] = {
  0,
  163.1,
  326.2,
  489.3,
  652.4,
  815.5,
  978.6,
  1141.7,
  1304.8,
  1467.9,
  1631,
  1794.1,
  1957.2,
  2120.3,
  2283.4,
  2446.5,
  2609.6,
  2772.7,
  2935.8,
  3098.9,
  3262,
  3425.1,
  3588.2,
  3751.3,
  3914.4,
  4077.5,
  4240.6,
  4403.7,
  4566.8,
  4729.9,
  4893,
  5056.1,
  5219.2,
  5382.3,
  5545.4,
  5708.5,
  5871.6,
  6034.7,
  6197.8,
  6360.9,
  6524,
  6687.1,
  6850.2,
  7013.3,
  7176.4,
  7339.5,
  7502.6,
  7665.7,
  7828.8,
  7991.9,
  8155
};

float PdragArray[50] = {
  0,
  1.9584,
  4.0034,
  6.1753,
  8.5101,
  11.0414,
  13.8016,
  16.8218,
  20.1324,
  23.7631,
  27.7429,
  32.1006,
  36.8642,
  42.0616,
  47.7203,
  53.8673,
  60.5296,
  67.7338,
  75.5062,
  83.8732,
  92.8606,
  102.4942,
  112.7996,
  123.8023,
  135.5275,
  148.0005,
  161.2461,
  175.2894,
  190.1549,
  205.8674,
  222.4513,
  239.9311,
  258.331,
  277.6752,
  297.9879,
  319.293,
  341.6144,
  364.976,
  389.4015,
  414.9145,
  441.5388,
  469.2977,
  498.2146,
  528.3131,
  559.6163,
  592.1475,
  625.9298,
  660.9864,
  697.3403,
  735.0145
};


float velArray[50] = {
  0,
  0.907,
  1.8141,
  2.7211,
  3.6281,
  4.5351,
  5.4422,
  6.3492,
  7.2562,
  8.1633,
  9.0703,
  9.9773,
  10.8844,
  11.7914,
  12.6984,
  13.6054,
  14.5125,
  15.4195,
  16.3265,
  17.2336,
  18.1406,
  19.0476,
  19.9546,
  20.8617,
  21.7687,
  22.6757,
  23.5828,
  24.4898,
  25.3968,
  26.3039,
  27.2109,
  28.1179,
  29.0249,
  29.932,
  30.839,
  31.746,
  32.6531,
  33.5601,
  34.4671,
  35.3741,
  36.2812,
  37.1882,
  38.0952,
  39.0023,
  39.9093,
  40.8163,
  41.7234,
  42.6304,
  43.5374,
  44.4444
};


// POWER LOOKUP BASED ON DISTANCE
float distPArray[7] = {
  0, 300, 2000, 6400, 7200, 7600, 8045
};

float PdistArray[7] = {
  200, 300, 310, 400, 700, 700, 400
};

// VELOCITY LOOKUP BASED ON DISTANCE
float distVArray[45] = {
  0,
  40.9,
  98.2,
  169.2,
  252.4,
  346,
  448.4,
  558.8,
  676.7,
  802.3,
  934.8,
  1073.4,
  1217.6,
  1367.5,
  1522.6,
  1682.2,
  1845.9,
  2014.4,
  2187.7,
  2365.3,
  2546.9,
  2732.6,
  2923.0,
  3118.0,
  3317.3,
  3521.4,
  3729.7,
  3942.5,
  4159.4,
  4379.8,
  4603.4,
  4830.3,
  5060.4,
  5293.3,
  5529.3,
  5768.5,
  6010.2,
  6254.2,
  6500.4,
  6749.1,
  7000.2,
  7254.8,
  7514.0,
  7777.9,
  8044.9
};
float VdistArray[45] = {
  17.0,
  26.3183,
  34.1837,
  41.0268,
  47.0099,
  52.0903,
  56.4689,
  60.503,
  64.6191,
  68.4874,
  71.9079,
  74.9954,
  77.9479,
  80.9446,
  83.5075,
  85.6252,
  88.0441,
  90.6193,
  93.0973,
  95.2418,
  97.3258,
  99.6573,
  102.1795,
  104.5432,
  106.9289,
  109.3926,
  111.5672,
  113.9932,
  115.9624,
  117.688,
  119.437,
  121.2102,
  122.7476,
  124.2508,
  125.9955,
  127.5388,
  128.7407,
  129.9686,
  131.1598,
  132.4554,
  133.9281,
  136.1652,
  138.724,
  140.9808,
  141.9474
};


float FmultiMap(float val, float * _in, float * _out, uint8_t size)
{
  // take care the value is within range
  // val = constrain(val, _in[0], _in[size-1]);
  if (val <= _in[0]) return _out[0];
  if (val >= _in[size-1]) return _out[size-1];

  // search right interval
  uint8_t pos = 1;  // _in[0] allready tested
  while(val > _in[pos]) pos++;

  // this will handle all exact "points" in the _in array
  if (val == _in[pos]) return _out[pos];

  // interpolate in the right segment for the rest
  return (val - _in[pos-1]) * (_out[pos] - _out[pos-1]) / (_in[pos] - _in[pos-1]) + _out[pos-1];
}
 
    
float powerLookup(float distLocal) {
  return FmultiMap(distLocal, distPArray, PdistArray, 7);
}

float speedLookup(float distLocal) {
  return FmultiMap(distLocal, distVArray, VdistArray, 45)*100/3.6;
}
 
    
void simulate(float power, uint16_t time_interval, uint8_t print, float* velo, float* dist) {
  
  // Run time measure
  uint16_t t1, t2 = 0;
  t1 = millis();
  
  // Nothing to calculate
  if (power == 0 && time_interval == 0) return;

  // save variables for manipulation
  sim_vel = *velo;
  sim_dist = *dist;
  sim_dt = time_interval * 0.0005; // in seconds
  
  // check for bad input (power can't be negative)
  if (power >= 0) {

    // Lookup Pdrag based on velocity
    sim_Pdrag = FmultiMap(sim_vel, velArray, PdragArray, 50);
    // Lookup Pslope based on distance
    sim_Pslope = FmultiMap(course_length - sim_dist, distArray, PslopeArray, 51) * G * M * sim_vel;
    // Pnet(i) = etaD*Ppedal(i) - Pdrag(i) + Pslope_t(i);
    sim_Pnet = DRIVE_ETA * power - sim_Pdrag + sim_Pslope;
    //v2(i) = v2(i-1) + 2/(M+Mwheels)*Pnet(i)*dt;
    //v(i) = sqrt(v2(i));
    sim_vel_new = sqrt(sim_vel * sim_vel + 2 / (M + M_WHEELS) * sim_Pnet * sim_dt);
    //d(i) = d(i-1) + 0.5*(v(i-1) + v(i))*dt;
    sim_dist = sim_dist + 0.5 * (sim_vel + sim_vel_new) * sim_dt;
    
//    Serial.print("sim_vel: ");
//    Serial.println(sim_vel);
//    Serial.print("sim_Pdrag: ");    
//    Serial.println(sim_Pdrag);
//    Serial.print("sim_dist: ");
//    Serial.println(sim_dist);
//    Serial.print("sim_Pslope: ");
//    Serial.println(sim_Pslope);
//    Serial.print("sim_Pnet: ");
//    Serial.println(sim_Pnet);
//    Serial.print("sim_vel_new: ");
//    Serial.println(sim_vel_new);
    
  }
   
  // Check for a problem before updating the variables
  if (isnan(sim_dist) || isinf(sim_dist) || isnan(sim_vel_new) || isinf(sim_vel_new)) {
    Serial.println("Warning: distance or velocity not a number. Breaking from simulate.");
    sd_Log("Warning: distance or velocity not a number. Breaking from simulate.");
    return;
  }

  // Update the variables with the new simulated distance and speed
  *dist = sim_dist;
  *velo = sim_vel_new;
}


// Takes ANT bytes and calculates value
uint16_t cB(const uint8_t x, const uint8_t y) {
  return (x * 256 + y);
}

// Sets offset
static uint16_t offset = 507;
void setOffset(int off) {
  offset = off;
}

// Returns offset
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

  // Exit function if not a power meter broadcast message on channel 0
  if (pwrRx[0] != 0x9 || pwrRx[1] != 0x4E || pwrRx[2] != 0x0) {
    *power = 0;
    *time_interval = 0;
    Serial.print("RESET POWER");
    return;
  }

  switch (pwrRx[3]) {
    case 0x10:
      if (pwrRx[6] != 0xFF)
        cadence = pwrRx[6];
      // else cadence doesn't change   
      *power = cB(pwrRx[10], pwrRx[9]);
      *time_interval = 0;
      break;
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
          } else if ( cB(pwrRx[7], pwrRx[8]) < cB(prev_data[7], prev_data[8])) { // Time stamp rollover
            temp_time = (65536 - cB(prev_data[7], prev_data[8])) + cB(pwrRx[7], pwrRx[8]);
          } else {
            temp_time = (cB(pwrRx[7], pwrRx[8]) - cB(prev_data[7], prev_data[8]));
          }

          if (pwrRx[4] < prev_data[4]) { // Cadence rollover
            cadence = (60 / 0.0005f) * ((256 - prev_data[4]) + pwrRx[4]) / (temp_time);
          } else {
            cadence = (60 / 0.0005f) * (pwrRx[4] - prev_data[4]) / (temp_time);
          }

          uint16_t torque_tick = 0;
          if ( cB(pwrRx[9], pwrRx[10]) < cB(prev_data[9], prev_data[10])) { // Torque tick rollover
            torque_tick = (65536 - cB(prev_data[9], prev_data[10])) + cB(pwrRx[9], pwrRx[10]);
          } else {
            torque_tick = (cB(pwrRx[9], pwrRx[10]) - cB(prev_data[9], prev_data[10]));
          }

          // Calculations based on those specified in the "ANT Bicycle Profile" documentation
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
        // received new power data message, so we must not be coasting anymore
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

    case 0x1: // Broadcast calibration data page
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

  } // else do nothing
  *time_interval = temp_time;
}


// Calculates cumulative average power
// Call function every time a power message is received (0.25s)
float pwrAvg(float pwrIn) {
  static unsigned int numData = 0;
  static float cumulativePower = 0;
  
  // Calculate average power over whole run
  numData++;
  cumulativePower += pwrIn;
  
  if (numData == 0)
    return 0;
  
  float avgTotalPower = cumulativePower/numData;
  return avgTotalPower;
}


// Calculates power averaged over the last 10 seconds
float tenSecPower(float pwrIn) {
  float avgPwr = 0;
  static float pwrData[15] = {0};
  static uint8_t index = 0;

  if (index >= 15) index = 0; // loop index around if larger than array size
  pwrData[index] = pwrIn;
  
  for (uint8_t i = 0; i < 15; i++) {
    avgPwr += pwrData[i];
  } avgPwr /= 15;
  
  index++;
  
  return avgPwr;
}

































































//// Run a step of the simulation
//void simulate(float power, uint16_t time_interval, uint8_t print, float* velo, float* dist) {
//  uint16_t t1, t2 = 0;
//  t1 = millis();
//  if (power == 0 && time_interval == 0) return;
//
//  float velocity = *velo;
//  float distance = *dist;
//
//  if (isnan(velocity)) {
//    Serial.print("static variable velocity in function simulate is NAN. velocity passed in as a parameter is ");
//    Serial.println(*velo);
//    velocity = *velo;
//  }
//
//  // constants
//  float g = 9.81;	// m/s
//  float mu = 0.0000185;	// Pa*s
//  uint8_t rho = 1;	// kg/m^3 for battle mountain
//
//  // rolling parameters
//  uint16_t M = /*pilot*/ 80 + /*hpv*/ 20 + /*wheels*/ 2; // total mass, kg
//  float Crr_one = 0.0015;
//  float Crr_two = 2.0 / 3 * 0.000041 * 3.6; // s/m
//
//  // aerodynamic parameters
//  float L = 2.88; // m
//  float Lnose = 1.00; // m
//  float h = 0.75; // m
//  float w = 0.45; // m
//  float xt = 1.90; // transition point, m
//  float Af = 3.14159 * (h / 2) * (w / 2); // frontal area, m^2
//  float Aside = 0.5 * 3.14159 * Lnose * (h / 2) + (L - Lnose) * h;	// side view area
//  float Awet = (0.037 * Af * Af + 0.02 * Af + 1) * Aside * 2;	// wetted area
//
//  // drivetrain efficiency
//  float etaD = 0.96334; // %
//  
//  // rolling resistance
//  float Proll;
//  if (velocity == 0) Proll = 0;
//  else Proll = velocity * M * g * (Crr_one + Crr_two * velocity);
//
//  /**** aerodynamic drag ****/
//  float Paero;
//  float q = 0.5 * rho * pow(velocity, 2); // dynamic pressure
//  
//  // flat plate resistance
//  float Dflam = 1.328 * h * q * sqrt(mu / velocity / rho) * sqrt(xt);	// laminar drag
//  float deltalamxt = 5 * sqrt(mu / velocity / rho) * sqrt(xt);	// lam BL thickness at xt
//  float deltaturbxt = 0.13 / 0.097 * deltalamxt;	// turb BL thickness at xt
//  float xdel = deltaturbxt / 0.375 * pow(pow(velocity * rho / mu, 0.2), 1 / 0.8);
//  float xzero = xt - xdel;	// imaginary turb start
//  float Dfturb = 0.0576 / 0.8 * h * q * pow(mu / velocity / rho, 0.2) * (pow(L - xzero, 0.8) - pow(xdel, 0.8));
//  float Cfflat = (Dflam + Dfturb) / (q * h * L);
//
//  // body drag
//  float Cdwet = Cfflat * (1 + 1.8 * pow(Af, 0.75) / pow(L, 1.5) + 39 * pow(Af, 3) / pow(L, 6));
//  float CdAbody = Cdwet * Awet;
//
//  // total aerodynamic drag
//  float CdA = CdAbody + /*CdAfwheel*/ 0.002 + /*CdArwheel*/ 0.003 + /*CdAunclean*/ 0.001;
//  if (velocity == 0) Paero = 0;
//  else Paero = q * CdA * velocity;
//  
//  if (isnan(CdA)) {
//    Paero = 0;
//    Serial.println("CdA is NaN.");
//    Serial.print("q: "); Serial.println(q); Serial.print("CdA: "); Serial.println(CdA); Serial.print("velocity: "); Serial.println(velocity);
//  }
//
//
//  if (isnan(velocity)) {
//    Serial.print("static variable velocity in function simulate2 is NAN. (check #2) velocity passed in as a parameter is ");
//    Serial.println(*velo);
//    velocity = *velo;
//  }
//
//  float prev_velo = velocity;	// for average speed calculation used in distance formula
//  float prev_dist = distance; // for Pelev
//  float power_left = 0;
//  float power_interval = time_interval * 0.0005; // in seconds
//
//  // change in elevation
//  float change_elev, Pelev, prev_Pelev = 0;
//
//  float delta_v, delta_d;
//  
//  // check for bad input (power can't be negative)
//  if (power >= 0) {
//
//    float power_in = power * etaD;	// watts (J/s)
//
//    bool calc_dist = true;
//    uint8_t count = 0;
//    float elev_calc[10] = {0};
//
//    while (calc_dist) {
//      power_left = power_in 	/*rolling friction*/ - Proll
//                   /*air drag*/ - Paero
//                   /*elevation change*/ + Pelev;
//
//      float net_energy = power_left * power_interval + 0.5 * (M+2) * pow(prev_velo, 2);
//      if (net_energy < 0) velocity = 0;
//      else velocity = sqrt(2 * net_energy / (M+2));
//
//      distance = prev_dist + 0.5 * (prev_velo + velocity) * power_interval;
//
//      // Guess elevation change using previous velocity and time travelled
//      change_elev = getElevation(distance) - getElevation(prev_dist);
//      if (change_elev > 100) {
//        Serial.println("Elevation greater than 100");
//        Serial.println(change_elev);
//      } else {
//        prev_Pelev = Pelev;
//        Pelev = M * (-g) * change_elev / power_interval; //kgm2/s3  kg*m/s2 * m / s
//  
//        elev_calc[count] = Pelev;
//      }
//      
//      count++;
//      
//      // IF ELEVATION IS GOING TO INFINITY (HAVEN'T SEEN IT HAPPEN IN A LONG TIME), PUT THIS CODE BACK IN.
////      if (isinf(Pelev)) {
////        Serial.println("Pelev is infinity");
////        Serial.print("Change_elev: "); Serial.println(change_elev);
////        Serial.print("Power interval: "); Serial.println(power_interval);
////        
////        char sdBuffer[32];
////        sd_Log("Pelev is infinity. ");
////        sd_Log("Change_elev: "); sd_Log(dtoa(sdBuffer, change_elev));
////        sd_Log(" Power interval: "); sd_Log(dtoa(sdBuffer, power_interval));
////        
////        // Stop iterating
////        Pelev = prev_Pelev;
////      }
//
//      if (abs(Pelev - prev_Pelev) <= 1) calc_dist = false;
//      else if (count > 9) calc_dist = false;
//
//    }
//
//    if (velocity < 0) velocity = 0;
//
//    if (print == 1) {
//      Serial.print("Power: ");
//      Serial.print(power);
//      Serial.print(" W.\tPower interval: ");
//      Serial.print(power_interval);
//      Serial.print(" s.\tEnergy in: ");
//      Serial.print(power_in * power_interval);
//      Serial.print(" J.\tVelocity: ");
//      Serial.print(velocity * 3.6); // convert to km/h
//      Serial.print(" km/h.\tDistance: ");
//      Serial.print(distance * 1.0);
//      Serial.print(" m.\tNet energy: ");
//      Serial.print(power_left * power_interval);
//      Serial.print (" J.\n");
//      /*Serial.print("Compute time = ");
//      Serial.println(t2-t1);*/
//    } else if (print == 2) {
//      Serial.print("Power in: ");
//      Serial.print(power_in);
//      Serial.print(" W.\tPower interval: ");
//      Serial.print(power_interval);
//      Serial.print(" s.\tRolling Power: ");
//      Serial.print(Proll);
//      Serial.print(" W.\tAir Power: ");
//      Serial.print(Paero);
//      Serial.print(" W.\tElev Power: ");
//      Serial.print(Pelev);
//      Serial.print(" W.\tPower left: ");
//      Serial.print(power_left);
//      Serial.print(" W.\tVelocity: ");
//      Serial.print(velocity * 3.6);
//      Serial.print(" km/h.\tDistance: ");
//      Serial.print(distance * 1.0);
//      Serial.print(" m.\n");
//    } else if (print == 3) {
//      Serial.print("Velocity = ");
//      Serial.print(velocity * 3.6);
//      Serial.print(" km/h\n");
//    }
//  }
//  
//  if (isnan(distance) || isinf(distance) || isnan(velocity) || isinf(velocity)) {
//    Serial.println("Warning: distance or velocity not a number. Breaking from simulate.");
//    sd_Log("Warning: distance or velocity not a number. Breaking from simulate.");
//    return; // If something goes really, really wrong and none of the other checks can save it, return without updating distance or velocity
//  }
//
//  *dist = distance;
//  *velo = velocity;
//}
//
//static const uint8_t ELEVATIONS_LENGTH = 20;
//
//float elevations[ELEVATIONS_LENGTH][2] = {
//  {0, 1470.75144},
//  {182.88, 1468.8312},
//  {563.88, 1463.25336},
//  {792.48, 1461.42456},
//  {960.12, 1460.72352},
//  {1310.64, 1458.28512},
//  {1630.68, 1456.60872},
//  {1950.72, 1454.68848},
//  {2240.28, 1453.56072},
//  {2697.48, 1450.45176},
//  {2910.84, 1449.35448},
//  {3215.64, 1447.22088},
//  {4114.8, 1439.84472},
//  {4465.32, 1436.5224},
//  {5760.72, 1427.25648},
//  {6141.72, 1424.20848},
//  {7025.64, 1418.6916},
//  {7467.6, 1416.80184},
//  {8046.72, 1412.93088},
//  {200000, 1412.93088}
//};
//
//// Finds a linear approximation of the elevation based on key points
//float getElevation(float distance) {
//  int distIndex = 0;
//  const float endSlope = -0.00715;
//  float elev;
//
//  // Simple extrapolation
//  if (distance > elevations[ELEVATIONS_LENGTH - 1][0]) {
//    return elevations[ELEVATIONS_LENGTH - 1][1] + (distance - elevations[ELEVATIONS_LENGTH - 1][1]) * endSlope;
//  }
//
//  // Find correct index
//  while (distance > elevations[distIndex][0]) {
//    distIndex++;
//  }
//
//  if (distIndex == 0)
//    elev = elevations[0][1];
//  else
//    elev = elevations[distIndex - 1][1] + (distance - elevations[distIndex - 1][0]) * (elevations[distIndex][1] - elevations[distIndex - 1][1]) / (elevations[distIndex][0] - elevations[distIndex - 1][0]);
//  
//  return elev;
//}






// OLD LOOKUP TABLES

//float PdragArray[50] = {
//  0,
//  1.9582,
//  4.0022,
//  6.1711,
//  8.5002,
//  11.022,
//  13.7681,
//  16.7687,
//  20.0531,
//  23.6502,
//  27.588,
//  31.8944,
//  36.5965,
//  41.7213,
//  47.2952,
//  53.3444,
//  59.895,
//  66.9727,
//  74.6028,
//  82.8106,
//  91.6212,
//  101.0595,
//  111.15,
//  121.9174,
//  133.386,
//  145.5799,
//  158.5233,
//  172.2401,
//  186.7542,
//  202.0891,
//  218.2686,
//  235.3159,
//  253.2547,
//  272.108,
//  291.899,
//  312.6509,
//  334.3866,
//  357.1289,
//  380.9008,
//  405.725,
//  431.624,
//  458.6206,
//  486.7371,
//  515.996,
//  546.4198,
//  578.0306,
//  610.8508,
//  644.9024,
//  680.2077,
//  716.7886
//};


//float distPArray[6] = {
//  0.0,
//  420.0,
//  5800.0,
//  7250.0,
//  8050.0,
//  8150.0
//};
//
//float PdistArray[6] = {
//  200.0,
//  300.0,
//  350.0,
//  500.0,
//  500.0,
//  0.0
//};

//float VdistArray[6] = {
//  20.0,
//  55.21,
//  126.2,
//  132.8,
//  136.9,
//  137.1
//};









//
//// VELOCITY LOOKUP BASED ON DISTANCE
//float distVArray[309] = {
//  0.0, 4.7, 9.8, 15.3, 21.2, 27.4, 34, 40.9, 
//  48.2, 55.7, 63.6, 71.8, 80.3, 89.1, 98.2, 107.5, 
//  117.1, 127, 137.2, 147.6, 158.3, 169.2, 180.4, 191.8, 
//  203.5, 215.4, 227.5, 239.8, 252.4, 265.1, 278.1, 291.3, 
//  304.7, 318.3, 332, 346, 360.1, 374.4, 388.9, 403.5, 418.3, 
//  433.3, 448.4, 463.7, 479.2, 494.8, 510.5, 526.5, 542.5, 558.8, 
//  575.1, 591.7, 608.4, 625.2, 642.2, 659.4, 676.7, 694.2, 711.9, 
//  729.7, 747.6, 765.7, 784, 802.3, 820.9, 839.5, 858.3, 877.2, 896.3, 
//  915.5, 934.8, 954.2, 973.8, 993.5, 1013.3, 1033.2, 1053.2, 1073.4, 1093.6, 
//  1114, 1134.5, 1155.1, 1175.8, 1196.7, 1217.6, 1238.7, 1259.8, 1281.1, 1302.5, 
//  1324.1, 1345.7, 1367.5, 1389.3, 1411.3, 1433.4, 1455.5, 1477.8, 1500.2, 1522.6, 
//  1545.2, 1567.8, 1590.5, 1613.3, 1636.2, 1659.1, 1682.2, 1705.3, 1728.5, 1751.8, 
//  1775.2, 1798.6, 1822.2, 1845.9, 1869.7, 1893.5, 1917.5, 1941.6, 1965.7, 1990, 2014.4, 
//  2038.8, 2063.4, 2088.1, 2112.8, 2137.7, 2162.6, 2187.7, 2212.8, 2238, 2263.3, 2288.7, 
//  2314.2, 2339.7, 2365.3, 2391, 2416.8, 2442.7, 2468.6, 2494.6, 2520.7, 2546.9, 2573.2, 
//  2599.5, 2626, 2652.5, 2679.1, 2705.8, 2732.6, 2759.5, 2786.5, 2813.6, 2840.8, 2868.1, 
//  2895.5, 2923, 2950.5, 2978.2, 3006, 3033.9, 3061.8, 3089.9, 3118, 3146.2, 3174.5, 3202.9, 
//  3231.3, 3259.9, 3288.6, 3317.3, 3346.2, 3375.2, 3404.2, 3433.4, 3462.6, 3492, 3521.4, 3550.9, 
//  3580.5, 3610.2, 3639.9, 3669.8, 3699.7, 3729.7, 3759.8, 3790, 3820.3, 3850.7, 3881.2, 3911.8, 
//  3942.5, 3973.2, 4004.1, 4035, 4066, 4097.1, 4128.2, 4159.4, 4190.7, 4222, 4253.5, 4284.9, 4316.5, 
//  4348.1, 4379.8, 4411.5, 4443.3, 4475.2, 4507.1, 4539.1, 4571.2, 4603.4, 4635.6, 4667.9, 4700.2, 4732.7, 
//  4765.1, 4797.7, 4830.3, 4863, 4895.8, 4928.6, 4961.5, 4994.4, 5027.4, 5060.4, 5093.5, 5126.7, 5159.9, 5193.2, 
//  5226.5, 5259.9, 5293.3, 5326.9, 5360.4, 5394.1, 5427.8, 5461.5, 5495.4, 5529.3, 5563.3, 5597.3, 5631.5, 5665.6, 
//  5699.9, 5734.2, 5768.5, 5802.9, 5837.3, 5871.8, 5906.3, 5940.9, 5975.5, 6010.2, 6044.9, 6079.7, 6114.5, 6149.3, 
//  6184.2, 6219.2, 6254.2, 6289.2, 6324.3, 6359.5, 6394.6, 6429.9, 6465.1, 6500.4, 6535.8, 6571.2, 6606.7, 6642.2, 
//  6677.8, 6713.4, 6749.1, 6784.8, 6820.5, 6856.4, 6892.2, 6928.1, 6964.1, 7000.2, 7036.3, 7072.5, 7108.8, 7145.1, 
//  7181.6, 7218.1, 7254.8, 7291.5, 7328.3, 7365.3, 7402.3, 7439.4, 7476.7, 7514, 7551.4, 7589, 7626.6, 7664.3, 7702.1, 
//  7740, 7777.9, 7816, 7854, 7892.1, 7930.3, 7968.5, 8006.7, 8044.9
//};
//float VdistArray[309] = {
//  17.0, 18.113, 19.6336, 21.079, 22.4617, 23.7913, 25.075, 26.3183, 27.526, 28.7013, 29.8474, 
//  30.9668, 32.0615, 33.1333, 34.1837, 35.214, 36.2253, 37.2186, 38.1946, 39.1542, 40.0982, 41.0268, 
//  41.9378, 42.8297, 43.7025, 44.5567, 45.3924, 46.2101, 47.0099, 47.7919, 48.5563, 49.3034, 50.0332, 
//  50.7427, 51.4261, 52.0903, 52.7444, 53.3885, 54.0229, 54.6479, 55.2637, 55.8707, 56.4689, 57.0585, 
//  57.6398, 58.2129, 58.7817, 59.3529, 59.9266, 60.503, 61.0822, 61.6644, 62.2498, 62.8384, 63.4304, 64.0258, 
//  64.6191, 65.202, 65.7747, 66.3372, 66.8896, 67.432, 67.9645, 68.4874, 69.0006, 69.5047, 70.0009, 70.4892, 70.9697, 
//  71.4426, 71.9079, 72.3658, 72.8161, 73.259, 73.6977, 74.1333, 74.5659, 74.9954, 75.4221, 75.8458, 76.2667, 76.6846, 
//  77.1032, 77.5243, 77.9479, 78.3741, 78.803, 79.2344, 79.6684, 80.105, 80.5309, 80.9446, 81.3461, 81.7355, 82.1128, 
//  82.4781, 82.8314, 83.174, 83.5075, 83.8321, 84.1478, 84.4546, 84.7526, 85.0419, 85.3273, 85.6252, 85.9358, 86.2589, 
//  86.5945, 86.9427, 87.3033, 87.6739, 88.0441, 88.4138, 88.7831, 89.1518, 89.5202, 89.8881, 90.2549, 90.6193, 90.9816, 
//  91.3419, 91.7004, 92.0569, 92.4115, 92.759, 93.0973, 93.4263, 93.7463, 94.0572, 94.359, 94.6536, 94.9479, 95.2418, 
//  95.5353, 95.8285, 96.1213, 96.4137, 96.7115, 97.0155, 97.3258, 97.6423, 97.965, 98.2941, 98.6282, 98.9667, 99.3098, 
//  99.6573, 100.0093, 100.3657, 100.7252, 101.0863, 101.4491, 101.8135, 102.1795, 102.547, 102.909, 103.2592, 103.5977, 
//  103.9244, 104.2396, 104.5432, 104.851, 105.1704, 105.5015, 105.8441, 106.1983, 106.564, 106.9289, 107.292, 107.6533, 
//  108.0128, 108.3705, 108.7235, 109.0641, 109.3926, 109.7088, 110.0129, 110.305, 110.6028, 110.9124, 111.2339, 111.5672, 
//  111.9122, 112.2649, 112.6152, 112.9632, 113.3088, 113.6522, 113.9932, 114.3189, 114.6281, 114.9209, 115.1972, 115.4571, 
//  115.7103, 115.9624, 116.2135, 116.4635, 116.7124, 116.9598, 117.2049, 117.4476, 117.688, 117.926, 118.1637, 118.4072, 
//  118.6564, 118.9113, 119.1718, 119.437, 119.6996, 119.9597, 120.2174, 120.4726, 120.725, 120.9709, 121.2102, 121.443, 
//  121.6693, 121.8894, 122.1073, 122.323, 122.5364, 122.7476, 122.9571, 123.1679, 123.38, 123.5934, 123.8082, 124.0256, 
//  124.2508, 124.4835, 124.7238, 124.9716, 125.2249, 125.48, 125.7368, 125.9955, 126.2558, 126.5085, 126.7449, 126.9652, 
//  127.1695, 127.3578, 127.5388, 127.7156, 127.8881, 128.0565, 128.2206, 128.3894, 128.5628, 128.7407, 128.9232, 129.1076, 
//  129.2877, 129.4635, 129.6351, 129.8023, 129.9686, 130.1358, 130.304, 130.4732, 130.6433, 130.8101, 130.9818, 131.1598, 
//  131.3442, 131.5322, 131.7189, 131.9042, 132.0882, 132.2708, 132.4554, 132.6431, 132.8338, 133.0275, 133.2268, 133.4433, 
//  133.6771, 133.9281, 134.1959, 134.4816, 134.7853, 135.1072, 135.4469, 135.8033, 136.1652, 136.5273, 136.8896, 137.252, 
//  137.6163, 137.983, 138.3523, 138.724, 139.0949, 139.4572, 139.811, 140.1466, 140.4599, 140.7381, 140.9808, 141.1883, 
//  141.3609, 141.5106, 141.6435, 141.7599, 141.8598, 141.9474
//};



