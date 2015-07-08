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

  if(n!=10) return false;

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
double VelodyneGPS::getTime()
{
  return packet_.GPS_timestamp*1e-6;
}

//------------------------------------------------------------------------------
string VelodyneGPS::getNmea()
{
  return string(packet_.NMEA_sentence);
}
