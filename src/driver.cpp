//    This file is part of Velodyne_Gps
//
//    This Velodyne_Gps is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//    Copyright (C) 2015 - ONERA - Christophe Grand <christophe.grand@onera.fr>

#include "driver.h"

//------------------------------------------------------------------------------
bool VelodyneGPS::connect()
{
  struct sockaddr_in self;  
  connected_ = false;

  // initialize socket 
  if ((socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    return false;

  printf("Packet_size = %li\n", sizeof(packet_));

  // bind to server port
  memset((char *) &self, 0, sizeof(struct sockaddr_in));
  self.sin_family = AF_INET;
  self.sin_port = htons(port_);
  self.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(socket_, (struct sockaddr *) &self, sizeof(self)) == -1)
    return false;
  
  connected_ = true;
  return true;
}

//------------------------------------------------------------------------------
bool VelodyneGPS::readFrame()
{
  struct sockaddr_in other;
  socklen_t len;

  int n = recvfrom(socket_, (void *)&packet_, sizeof(packet_), 0, 
		   (struct sockaddr *) &other, &len);

  if(n!=512) return false;

  printf("Received from %s:%d: ", 
	   inet_ntoa(other.sin_addr), ntohs(other.sin_port)); 
  printf("(size=%i) \n", n);
}

//------------------------------------------------------------------------------
VelodyneGPS::~VelodyneGPS()
{
  if(connected_)
    close(socket_);
}

//------------------------------------------------------------------------------
// return top hour GPS time in s 
double VelodyneGPS::getTime()
{
  return packet_.GPS_timestamp*1e-6;
}

//------------------------------------------------------------------------------
string VelodyneGPS::getNmea()
{
  string nmea_str(packet_.NMEA_sentence);
  nmea_str.erase(std::remove(nmea_str.begin(), nmea_str.end(), '\r'), 
		 nmea_str.end());
  nmea_str.erase(std::remove(nmea_str.begin(), nmea_str.end(), '\n'), 
		 nmea_str.end());
  return nmea_str;
}

//------------------------------------------------------------------------------
inline int VelodyneGPS::convert(uint16_t data)
{
  int value = (data & 0x0fff);
  if(value&0x0800) value|=0xFFFFF000;  
  return value;
}

//------------------------------------------------------------------------------
void VelodyneGPS::getImu(double gyro[], double accel[])
{
#define GYRO_SCALE (0.09766*2.14926/180.) // rad/s
#define ACCEL_SCALE (0.001221*9.81)  //m/s2


  gyro[0] = -(double)convert(packet_.gyro_1) * GYRO_SCALE; // X
  gyro[1] = +(double)convert(packet_.gyro_2) * GYRO_SCALE; // Y
  gyro[2] = +(double)convert(packet_.gyro_3) * GYRO_SCALE; // Z

  accel[0] = (double)(convert(packet_.accel_3_x)-convert(packet_.accel_1_y)) * ACCEL_SCALE/2; 
  accel[1] = (double)(convert(packet_.accel_2_y)+convert(packet_.accel_3_y)) * ACCEL_SCALE/2; 
  accel[2] = (double)(convert(packet_.accel_1_x)+convert(packet_.accel_2_x)) * ACCEL_SCALE/2; 

}
