/* GPS Data Processing Functions */

#define R	6371000.0
#define pi	3.14159

#include <math.h>

/*
 * Set starting position
 */
void GPS_setStart(){

 // LattitudeStart = GPS.Lattitude;
 // LongitudeStart = GPS.Longitude;
 // AltitudeStart = GPS.Altitude;

// FROM GOOGLE MAPS
 // LattitudeStart = 40.295852*10000000;
 // LongitudeStart = -83.530750*10000000;
 // AltitudeStart = 327*1000;


// FROM A RUN
  LattitudeStart = 40.2982673*10000000;
  LongitudeStart = -83.5329132*10000000;
  AltitudeStart = 327.7030334*1000;


  LattitudePrev = GPS.Lattitude;
  LongitudePrev = GPS.Longitude;
  AltitudePrev = GPS.Altitude;

  startSet = 1; // Flag to not run this function again

  return;
}
/*
 * Calculates the 3D distance in mm between two points using 
 * Pythagoras Theorem on an equirectangular projection
 */
uint32_t GPS_getDistance(const int32_t lat1, const int32_t lon1, const int32_t alt1, const int32_t lat2, const int32_t lon2, const int32_t alt2){
  
  /*  THIS MATH DOESN'T WORK FOR LARGE DISTANCES?
  double x, y, z, dist;
  x = ((deg2rad(lon2)-deg2rad(lon1))/10000.0 * cos((deg2rad(lat1)+deg2rad(lat2))/2/10000.0));
  y = ((deg2rad(lat2)-deg2rad(lat1)))/10000.0; 
  dist = sqrt(x*x + y*y) * R;
  */
  
  // This math works with all GPS coords, not just two extremely close points
  float dist;
  float latRad, lonRad;
  float flat, flon;
  float tlat, tlon;
  float tlatRad, tlonRad;
  float midLat, midLon;
    
  //convert to decimal degree
  flat = lat1 /10000.0;
  flon = lon1 /10000.0;
  tlat = lat2 / 10000.0;
  tlon = lon2 / 10000.0;

  //convert decimal degree into radian
  latRad = flat * 0.017453293;
  lonRad = flon * 0.017453293;
  tlatRad = tlat * 0.017453293;
  tlonRad = tlon * 0.017453293;

  midLat = tlatRad - latRad;
  midLon = tlonRad - lonRad;

  //Calculate the distance in KM
  float latSin = sin((latRad - tlatRad)/2);
  float lonSin = sin((lonRad - tlonRad)/2);
  dist = 2 * R * asin(sqrt((latSin*latSin) + cos(latRad) * cos(tlatRad) * (lonSin * lonSin)));  
  
  //uint32_t final = 0;
  //final += dist;  
  return (dist);
}

double deg2rad(int32_t deg) {
  double rad = ((deg * pi) / 180.0);
  return rad;
}
double rad2deg(int32_t rad) {
  return ((rad * 180.0) / pi);
}


