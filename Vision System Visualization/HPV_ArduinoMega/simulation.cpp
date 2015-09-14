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
  1.9582,
  4.0022,
  6.1711,
  8.5002,
  11.022,
  13.7681,
  16.7687,
  20.0531,
  23.6502,
  27.588,
  31.8944,
  36.5965,
  41.7213,
  47.2952,
  53.3444,
  59.895,
  66.9727,
  74.6028,
  82.8106,
  91.6212,
  101.0595,
  111.15,
  121.9174,
  133.386,
  145.5799,
  158.5233,
  172.2401,
  186.7542,
  202.0891,
  218.2686,
  235.3159,
  253.2547,
  272.108,
  291.899,
  312.6509,
  334.3866,
  357.1289,
  380.9008,
  405.725,
  431.624,
  458.6206,
  486.7371,
  515.996,
  546.4198,
  578.0306,
  610.8508,
  644.9024,
  680.2077,
  716.7886
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

float PdistArray[6] = {
  0,
  420,
  5800,
  7250,
  8050,
  8150
};

float PArray[6] = {
  200,
  300,
  350,
  500,
  500,
  0
};

/* Distance to power target interpolation
  0 m - 200 W
  420 m - 300 W
  5800 m - 350 W
  7250 m - 470 W
  8050 m - 470 W
  8150 m - 0 W */
  
/* if speed is less than:  , then gearR =
  66.9 km/h - 93/38 * 39/18
  84.0 km/h - 93/30 * 39/18
  104.0 km/h - 93/24 * 39/18
  123.7 km/h - 93/20 * 39/18
  136.1 km/h - 93/18 * 39/18
  137.1 km/h -  93/16 * 39/18 */



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
  return FmultiMap(distLocal, PdistArray, PArray, 6);
}

float gearLookup(float v) {
  if (v < 66.9/3.6) { return 93.0/38.0 * 39.0/18.0; }
  else if (v < 84.0/3.6) { return 93.0/30.0 * 39.0/18.0; }
  else if (v < 104.0/3.6) { return 93.0/24.0 * 39.0/18.0; }
  else if (v < 123.7/3.6) { return 93.0/20.0 * 39.0/18.0; }
  else if (v < 136.1/3.6) { return 93.0/18.0 * 39.0/18.0; }
  else { return 93.0/16.0 * 39.0/18.0; }
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






