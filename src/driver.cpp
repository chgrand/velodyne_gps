#include "driver.h"

VelodyneGPS::connect()
{
  struct sockaddr_in self, other;
  int len = sizeof(struct sockaddr_in);
  int n, s;
  
  connected_ = false;

  // initialize socket 
  if ((socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    return false;

  ROS_INFO("Packet_size = %li\n", sizeof(packet));

  // bind to server port
  memset((char *) &self, 0, sizeof(struct sockaddr_in));
  self.sin_family = AF_INET;
  self.sin_port = htons(port_);
  self.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(s, (struct sockaddr *) &self, sizeof(self)) == -1)
    return false;
  
  connected_ = true;
  return true;
}

bool VelodyneGPS::readFrame()
{
  int n = recvfrom(socket_, (void *)&packet_, sizeof(packet_), 0, 
		   (struct sockaddr *) &other, &len);

  if(n!=10) return false;

  ROS_INFO("Received from %s:%d: ", inet_ntoa(other.sin_addr), ntohs(other.sin_port)); 
  ROS_INFO("(size=%i) ", n);
  //ROS_INFO("%.3f ", (float)packet_.GPS_timestamp*1e-6);
  //ROS_INFO("%s\n", packet_.NMEA_sentence);
}

VelodynGPS::~VelodyneGPS()
{
  if(connected_)
    close(socket_);
}


double VelodyneGPS::getTime()
{
  return packet_.GPS_timestamp*1e-6;
}

string VelodyneGPS::getNmea()
{
  return String(packet_.NMEA_sentence);
}
