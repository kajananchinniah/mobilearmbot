#pragma once

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <sensor_msgs/LaserScan.h>

class MobileArmBotArm
{
   public:
      MobileArmBotArm(ros::NodeHandle nh);

   private:
      ros::NodeHandle nh;
      ros::Subscriber laser_sub;

      int n_objects_to_grasp; //currently only supports 1 (i.e. the closest object)
      int n_collision_objects; 

      float obj_radius; //currently will only support the beer can found from gazebo
      float obj_height;
      float obj_x;
      float obj_y;
      float obj_z;

      std::vector<moveit_msgs::Grasp> grasps;
      std::vector<moveit_msgs::CollisionObject> collision_objects;
      tf2::Quaternion orientation;


      void laserScanCallback(const sensor_msgs::LaserScan& msg);
      void openEndEffector(trajectory_msgs::JointTrajectory& posture);
      void closeEndEffector(trajectory_msgs::JointTrajectory& posture);
      void pick(moveit::planning_interface::MoveGroupInterface& move_group);
      void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface);
};
