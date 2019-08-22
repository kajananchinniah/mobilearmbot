#include "MobileArmBotArm.hpp"

MobileArmBotArm::MobileArmBotArm(ros::NodeHandle nh) 
	: group("mobilearmbot_arm")
{ 
   this->queue_size = 5;
   this->obj_radius = 0.055;
   this->obj_height = 0.23;
   this->laser_topic_name = "scan";
   this->nh = nh;
   this->in_range = 0.5;
   this->laser_sub = this->nh.subscribe(this->laser_topic_name,
		   this->queue_size, &MobileArmBotArm::laserScanCallback,
		   this);
}

//TODO: should probably just publish the heading angle & dist from other node
void MobileArmBotArm::laserScanCallback(const sensor_msgs::LaserScan &msg)
{
   this->chosen_index = 0;
   this->closest_target = 9999.99;
   for (int i = 0; i < msg.ranges.size(); i++)
   {
      if (this->closest_target > msg.ranges[i])
      {
         this->closest_target = msg.ranges[i];
	 this->chosen_index = i;
      }
   }

   if (this->closest_target >= 9999.99)
   {
      ROS_ERROR_STREAM("Cannot find target");
      return;
   }

   this->heading_angle = msg.angle_min + this->chosen_index * msg.angle_increment;
   if (this->closest_target <= this->in_range){
      this->group.setPlanningTime(45.0);
      this->addCollisionObjects(this->planning_scene_interface);
      ros::WallDuration(1.0).sleep();
      this->pick(this->group);
      return;}
}

void MobileArmBotArm::openEndEffector(trajectory_msgs::JointTrajectory& posture)
{
   posture.joint_names.resize(2);
   posture.joint_names[0] = "left_end_effector_joint";
   posture.joint_names[1] = "right_end_effector_joint";

   posture.points.resize(1);
   posture.points[0].positions.resize(1);
   posture.points[0].positions[0] = 0.00;
   posture.points[0].positions[1] = 0.00;
   posture.points[0].time_from_start = ros::Duration(0.5);
}

void MobileArmBotArm::closeEndEffector(trajectory_msgs::JointTrajectory& posture)
{
   posture.joint_names.resize(2);
   posture.joint_names[0] = "left_end_effector_joint";
   posture.joint_names[1] = "right_end_effector_joint";

   posture.points.resize(1);
   posture.points[0].positions[0] = 1.89;
   posture.points[0].positions[1] = -1.89;
   posture.points[0].time_from_start = ros::Duration(0.5);
}

void MobileArmBotArm::pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
   this->grasps.resize(1);
   this->grasps[0].grasp_pose.header.frame_id = "arm_link1";
   this->orientation.setRPY(0, M_PI, 0);
   grasps[0].grasp_pose.pose.orientation = tf2::toMsg(this->orientation);
   grasps[0].grasp_pose.pose.position.x = 0.4;
   grasps[0].grasp_pose.pose.position.y = 0.0;
   grasps[0].grasp_pose.pose.position.z = 0.3;

   grasps[0].pre_grasp_approach.direction.header.frame_id = "arm_link1";
   // Setting pre-grasp approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  grasps[0].pre_grasp_approach.direction.header.frame_id = "arm_link1";
  /* Direction is set as positive x axis */
  grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.095;
  grasps[0].pre_grasp_approach.desired_distance = 0.115;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  grasps[0].post_grasp_retreat.direction.header.frame_id = "arm_link1";
  /* Direction is set as positive z axis */
  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.1;
  grasps[0].post_grasp_retreat.desired_distance = 0.25;

  // Setting posture of eef before grasp
  // +++++++++++++++++++++++++++++++++++
  openEndEffector(grasps[0].pre_grasp_posture);
  // END_SUB_TUTORIAL

  // BEGIN_SUB_TUTORIAL pick2
  // Setting posture of eef during grasp
  // +++++++++++++++++++++++++++++++++++
  closeEndEffector(grasps[0].grasp_posture);
  // END_SUB_TUTORIAL

  // BEGIN_SUB_TUTORIAL pick3
  // Set support surface as table1.
  move_group.setSupportSurfaceName("ground");
  // Call pick to pick up the object using the grasps given
  move_group.pick("can", grasps);
  // END_SUB_TUTORIAL
}

void MobileArmBotArm::addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
   this->collision_objects.resize(2);
   this->collision_objects[0].id = "ground";
   this->collision_objects[0].header.frame_id = "base_scan";
   this->collision_objects[0].primitives[0].type = this->collision_objects[0].primitives[0].BOX;
   this->collision_objects[0].primitives[0].dimensions[0] = 1;
   this->collision_objects[0].primitives[0].dimensions[0] = 1;
   this->collision_objects[0].primitives[0].dimensions[0] = 1;

   this->collision_objects[0].primitive_poses.resize(1);
   this->collision_objects[0].primitive_poses[0].position.x = this->closest_target * cos(this->heading_angle);
   this->collision_objects[0].primitive_poses[0].position.y = this->closest_target * sin(this->heading_angle);
   this->collision_objects[0].primitive_poses[0].position.z = -1.125;

   this->collision_objects[0].operation = collision_objects[0].ADD;

   this->collision_objects[1].id = "can";
   this->collision_objects[1].header.frame_id = "base_scan";
   this->collision_objects[1].primitives.resize(1);
   this->collision_objects[1].primitives[0].type = collision_objects[0].primitives[0].CYLINDER;
   this->collision_objects[1].primitives[0].dimensions.resize(2);
   this->collision_objects[1].primitives[0].dimensions[0] = this->obj_height;
   this->collision_objects[1].primitives[0].dimensions[1] = this->obj_radius;
   this->collision_objects[1].primitive_poses.resize(1);
   this->collision_objects[1].primitive_poses[0].position.x = this->closest_target * cos(this->heading_angle);
   this->collision_objects[1].primitive_poses[0].position.y = this->closest_target * sin(this->heading_angle);
   this->collision_objects[1].primitive_poses[0].position.z = 0.00;
   this->collision_objects[0].operation = this->collision_objects[0].ADD;
   this->planning_scene_interface.applyCollisionObjects(this->collision_objects);
   return;
}
