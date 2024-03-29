#pragma once

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <sensor_msgs/LaserScan.h>

#include <string>
#include <math.h>

class MobileArmBotArm
{
   public:
      MobileArmBotArm(ros::NodeHandle nh);

   private:
      ros::NodeHandle nh;
      ros::Subscriber laser_sub;
      int queue_size;
      std::string laser_topic_name;
      float heading_angle;
      float closest_target;
      float in_range;
      int chosen_index;
      bool start_grasp;

      tf2::Quaternion orientation;
      moveit::planning_interface::MoveGroupInterface arm_group;
      moveit::planning_interface::MoveGroupInterface claw_group;

      void laserScanCallback(const sensor_msgs::LaserScan& msg);
      void openEndEffector();
      void closeEndEffector();
};
