/* GPS Data Processing Functions */

#define R	6371000.0
#define pi	3.14159

#include <math.h>

/*
 * Set starting position
 */
void GPS_setStart(){

  LattitudeStart = GPS.Lattitude;
  LongitudeStart = GPS.Longitude;
  AltitudeStart = GPS.Altitude;

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

  double x, y, z, dist;
  x = ((deg2rad(lon2)-deg2rad(lon1))/10000.0 * cos((deg2rad(lat1)+deg2rad(lat2))/2/10000.0));
  y = ((deg2rad(lat2)-deg2rad(lat1)))/10000.0;
  
  dist = sqrt(x*x + y*y) * R;

  uint32_t final = 0;
  final += dist;  
  return (dist);
}

double deg2rad(int32_t deg) {
  double rad = ((deg * pi) / 180.0);
  return rad;
}
double rad2deg(int32_t rad) {
  return ((rad * 180.0) / pi);
}


