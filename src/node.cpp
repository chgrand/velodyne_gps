#include "driver.h"
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/TimeReference.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "VelodyneGPS_driver");
  ros::NodeHandle nh_;

  VelodyneGPS gps_driver(1010);

  if(!gps_driver.connect()) {
    ROS_ERROR("Could not connect to Velodyne at port XXXX");
    return 1;
  }

  while (ros::ok()) {
    if(gps_driver.readFrame()) {
      ROS_INFO("GPS time = %.3f ", gps_driver.getTIme());
      ROS_INFO("NMEA = %s\n", gps_driver.getNmea().c_str());
    }    
    ros::spinOnce();
  }
  
  return 0;
}
