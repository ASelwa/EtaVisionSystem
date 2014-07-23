#ifndef GPS_UBLOX_h
#define GPS_UBLOX_h

#include <inttypes.h>

#define UBX_MAXPAYLOAD 60

#define GPS_SERIAL2
#define GPS_BAUD 115200


class GPS_UBLOX_Class
{
  private:
    // Internal variables
	uint8_t ck_a;     // Packet checksum
	uint8_t ck_b;
	uint8_t UBX_step;
	uint8_t UBX_class;
	uint8_t UBX_id;
	uint8_t UBX_payload_length_hi;
	uint8_t UBX_payload_length_lo;
	uint8_t UBX_payload_counter;
	uint8_t UBX_buffer[UBX_MAXPAYLOAD];
	uint8_t UBX_ck_a;
	uint8_t UBX_ck_b;
	int32_t GPS_Timer;
	int32_t UBX_ecefVZ;
	void parse_ubx_gps();
	void ubx_checksum(unsigned char ubx_data);
	int32_t join_4_bytes(unsigned char Buffer[]);

  public:
    // Methods
	GPS_UBLOX_Class();
	void Init();
	void Read();
	void printBuffer();
	// Properties
	uint32_t GPSTime;          //GPS Millisecond Time of Week
        struct {
          uint16_t year;
          uint8_t month;
          uint8_t day;
          uint8_t hour;
          uint8_t minute;
          uint8_t second;
        } UTC;
        
        
	int32_t Lattitude;     // Geographic coordinates
	int32_t Longitude;
	int32_t Altitude;
	int32_t Ground_Speed;
	int32_t Speed_3d;      //Speed (3-D)
	int32_t Ground_Course;
	uint8_t NumSats;      // Number of visible satelites
	uint8_t Fix;        // 1:GPS FIX   0:No FIX (normal logic)
	uint8_t NewData;    // 1:New GPS Data
	uint8_t PrintErrors; // 1: To Print GPS Errors (for debug)
        uint32_t vacc;       // Horizontal Accuracy
        uint32_t hacc;      // Vertical Accuracy
      };

extern GPS_UBLOX_Class GPS;

#endif
