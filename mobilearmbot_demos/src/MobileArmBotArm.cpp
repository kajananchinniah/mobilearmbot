/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
//Credits for basis of code:
/* Code source: https://github.com/ros-planning/moveit_tutorials/blob/melodic-devel/doc/pick_place/src/pick_place_tutorial.cpp */
/* Author: Ioan Sucan, Ridhwan Luthra*/
//NOTE: the code found here is modified. 

#include "MobileArmBotArm.hpp"

MobileArmBotArm::MobileArmBotArm(ros::NodeHandle nh)
{
   this->nh = nh;
   this->n_objects_to_grasp = 1;
   this->grasps.resize(this->n_objects_to_grasp);
   this->n_collision_objects = 1;
   this->collision_objects.resize(this->n_collision_objects);
   this->queue_size = 5;
   this->object_height = 0.23;
   this->object_radius = 0.055;

   this->closest_object_dist_sub = this->nh.subscribe("target_distance", this->queue_size, 
		   &MobileArmBotArm::closestObjectDistCallback, this);
   ros::WallDuration(1.0).sleep();
   moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
   moveit::planning_interface::MoveGroupInterface group("mobilearmbot_arm");   
   addCollisionObjects(planning_scene_interface);
   ros::WallDuration(1.0).sleep();
   pick(group);
   ros::WallDuration(1.0).sleep();
}

void MobileArmBotArm::closestObjectDistCallback(const std_msgs::Float32 closest_object_dist)
{
   this->closest_object_dist = closest_object_dist.data;
   return;
}

void MobileArmBotArm::openEndEffector(trajectory_msgs::JointTrajectory& posture)
{
   //Add end effector joints
   posture.joint_names.resize(2);
   posture.joint_names[0] = "left_end_effector_joint";
   posture.joint_names[1] = "right_end_effector_joint";

   //Open them
   posture.points.resize(1);
   posture.points[0].positions.resize(2);
   posture.points[0].positions[0] = 0.00; //TODO: change these values
   posture.points[0].positions[1] = 0.00;
   posture.points[0].time_from_start = ros::Duration(0.5);
}

void MobileArmBotArm::closeEndEffector(trajectory_msgs::JointTrajectory& posture)
{
  //Add end effector joints
  posture.joint_names.resize(2);
  posture.joint_names[0] = "left_end_effector_joint";
  posture.joint_names[1] = "right_end_effector_joint";

  //Close them
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 1.89; //TODO: changes these values
  posture.points[0].positions[1] = -1.89;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void MobileArmBotArm::pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
   this->grasps[0].grasp_pose.header.frame_id = "base_scan";
   this->orientation.setRPY(-M_PI/2, -M_PI/4, -M_PI/2);
   this->grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
   this->grasps[0].grasp_pose.pose.position.x = 0.415;
   this->grasps[0].grasp_pose.pose.position.y = 0;
   this->grasps[0].grasp_pose.pose.position.z = 0.5;

   this->grasps[0].pre_grasp_approach.direction.header.frame_id = "base_scan";
   this->grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
   this->grasps[0].pre_grasp_approach.min_distance = 0.095;
   this->grasps[0].pre_grasp_approach.desired_distance = 0.115;

   this->grasps[0].post_grasp_retreat.direction.header.frame_id = "base_scan";
   this->grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
   this->grasps[0].post_grasp_retreat.min_distance = 0.1;
   this->grasps[0].post_grasp_retreat.desired_distance = 0.25;

   this->openEndEffector(this->grasps[0].pre_grasp_posture);

   this->closeEndEffector(this->grasps[0].grasp_posture);
   move_group.pick("can", grasps);
}

void MobileArmBotArm::place(moveit::planning_interface::MoveGroupInterface& group)
{
   return;
}

void MobileArmBotArm::addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
   this->collision_objects[0].header.frame_id = "base_scan";
   this->collision_objects[0].id = "can";

   this->collision_objects[0].primitives.resize(1);
   this->collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].CYLINDER;
   this->collision_objects[0].primitives[0].dimensions.resize(2);
   this->collision_objects[0].primitives[0].dimensions[0] = this->object_height;
   this->collision_objects[0].primitives[0].dimensions[1] = this->object_radius;

   this->collision_objects[0].primitive_poses.resize(1);
   //TODO: change these values
   this->collision_objects[0].primitive_poses[0].position.x = 0.00;
   this->collision_objects[0].primitive_poses[0].position.y = this->closest_object_dist;
   this->collision_objects[0].primitive_poses[0].position.z = 0.00;
   collision_objects[0].operation = collision_objects[0].ADD;
   planning_scene_interface.applyCollisionObjects(collision_objects);
   return;
}
