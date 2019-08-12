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
}

void openEndEffector(trajectory_msgs::JointTrajectory& posture)
{
   //Add end effector joints
   posture.joint_names.resize(2);
   posture.joint_names[0] = "left_end_effector_joint";
   posture.joint_names[1] = "right_end_effector_joint";

   //Open them
   posture.points.resize(1);
   posture.points[0].positions.resize(2);
   posture.points[0].positions[0] = 0.04; //TODO: change these values
   posture.points[0].positions[1] = 0.04;
   posture.points[0].time_from_start = ros::Duration(0.5);
}

void closeEndEffector(trajectory_msgs::JointTrajectory& posture)
{
  //Add end effector joints
  posture.joint_names.resize(2);
  posture.joint_names[0] = "left_end_effector_joint";
  posture.joint_names[1] = "right_end_effector_joint";

  //Close them
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.00; //TODO: changes these values
  posture.points[0].positions[1] = 0.00;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
   return;
}

void place(moveit::planning_interface::MoveGroupInterface& group)
{
   return;
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
   return;
}
