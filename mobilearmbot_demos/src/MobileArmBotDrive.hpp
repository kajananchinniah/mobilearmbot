#pragma once

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>

#include <string>

class MobileArmBotDrive
{
   public:
      MobileArmBotDrive(ros::NodeHandle nh);

   private:
      ros::NodeHandle nh;
      ros::Subscriber laser_sub;
      ros::Publisher cmd_vel_pub;
      
      geometry_msgs::Twist vel;
      std_msgs::Float32 closest_target_msg;
      int queue_size;
      std::string laser_topic_name;
      float heading_angle;
      int chosen_index;
      float closest_target;
      float kp_vel;
      float kp_ang;

      void laserScanCallback(const sensor_msgs::LaserScan &scanned_msg);
      void mobilearmbot_drive_controller(float dist, float angle);
};
