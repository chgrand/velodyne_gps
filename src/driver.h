// -*- c++-mode -*-

#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>

#define BUF_SIZE 1024

#pragma pack (push, 1)

typedef struct {
  //uint8_t   header[42];
  uint8_t   not_used_0[14];
  uint16_t  gyro_1;
  uint16_t  temp_1;
  uint16_t  accel_1_x;
  uint16_t  accel_1_y;
  uint16_t  gyro_2;
  uint16_t  temp_2;
  uint16_t  accel_2_x;
  uint16_t  accel_2_y;
  uint16_t  gyro_3;
  uint16_t  temp_3;
  uint16_t  accel_3_x;
  uint16_t  accel_3_y;
  uint8_t   not_used[160];
  uint32_t  GPS_timestamp;
  uint8_t   not_used_2[4];
  char	    NMEA_sentence[72];
  uint8_t   not_used_3[234];	

} GPS_packet_t;

#pragma pack (pop)


using namespace std;


class VelodyneGPS
{
public:
  VelodyneGPS(int port) : port_(port), connected_(false) {};
  ~VelodyneGPS();

  bool connect();
  bool isConnected() {return connected_;}
  void readFrame();
  double getTime();
  string getNmea();
  void getImu(double gyro[], double accel[]);

private:
  GPS_packet_t packet_;
  int port_;
  bool connected_;
}
