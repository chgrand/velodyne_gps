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
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/TimeReference.h>


int main(int argc, char **argv)
{
  std_msgs::String string_msg;
  sensor_msgs::Imu imu_msg;
  sensor_msgs::TimeReference time_msg;

  double gyro[3];
  double accel[3];

  ros::init(argc, argv, "VelodyneGPS_driver");
  ros::NodeHandle n;

  VelodyneGPS gps_driver(8308);

  if(!gps_driver.connect()) {
    ROS_ERROR("Could not connect to Velodyne at port 8308");
    return 1;
  }


  ros::Publisher nmea_pub = n.advertise<std_msgs::String>("gps/nmea", 1);
  ros::Publisher time_pub = n.advertise<sensor_msgs::TimeReference>("gps/time", 1);
  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu/data", 1);


  while (ros::ok()) {
    if(gps_driver.readFrame()) {

      ros::Time now = ros::Time::now();
      int now_secs = now.toSec();
      int from_hour = now_secs - (now_secs%3600);
      double gps_timestamp = gps_driver.getTime()+(double)from_hour;
      gps_driver.getImu(gyro, accel);

      // LOG data
      ROS_INFO("Sys time = %.3f", now.toSec());
      ROS_INFO("GPS time = %.3f ", gps_timestamp);
      ROS_INFO("NMEA = %s\n", gps_driver.getNmea().c_str());
      ROS_INFO("Gyro = <%.3f, %.3f, %.3f>",  gyro[0], gyro[1], gyro[2]);
      ROS_INFO("Accel = <%.3f, %.3f, %.3f>", accel[0], accel[1], accel[2]);

      string_msg.data = gps_driver.getNmea();
      nmea_pub.publish(string_msg);      
 
      time_msg.header.stamp = now;
      time_msg.source = "velodyne_gps";
      time_msg.time_ref = ros::Time(gps_timestamp);
      time_pub.publish(time_msg);
      
      imu_msg.header.stamp = now;
      imu_msg.header.frame_id = "velodyne";
      imu_msg.angular_velocity.x = gyro[0];
      imu_msg.angular_velocity.y = gyro[1];
      imu_msg.angular_velocity.z = gyro[2];
      imu_msg.linear_acceleration.x = accel[0];
      imu_msg.linear_acceleration.y = accel[1];
      imu_msg.linear_acceleration.z = accel[2];
      imu_pub.publish(imu_msg);
    }    
    ros::spinOnce();
  }
  
  return 0;
}
