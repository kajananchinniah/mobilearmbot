#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class MobileArmBotArm
{
   public:
      MobileArmBotArm(ros::NodeHandle nh);
   private:
      ros::NodeHandle nh;
      ros::Subscriber closest_object_dist_sub;

      int n_objects_to_grasp;
      int n_collision_objects;
      std::vector<moveit_msgs::Grasp> grasps;
      std::vector<moveit_msgs::CollisionObject> collision_objects;
      tf2::Quaternion orientation;

      void openEndEffector(trajectory_msgs::JointTrajectory& posture);
      void closeEndEffector(trajectory_msgs::JointTrajectory& posture);
      void pick(moveit::planning_interface::MoveGroupInterface& move_group);
      void place(moveit::planning_interface::MoveGroupInterface& group);
      void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface);
};
