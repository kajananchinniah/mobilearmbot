#include "MobileArmBotArm.hpp"

MobileArmBotArm::MobileArmBotArm(ros::NodeHandle nh) 
	: arm_group("mobilearmbot_arm"), claw_group("mobilearmbot_claw")
{ 
   this->queue_size = 5;
   this->obj_radius = 0.055;
   this->obj_height = 0.23;
   this->laser_topic_name = "scan";
   this->nh = nh;
   this->in_range = 0.75;
   this->laser_sub = this->nh.subscribe(this->laser_topic_name,
		   this->queue_size, &MobileArmBotArm::laserScanCallback,
		   this);
}

//TODO: should probably just publish the heading angle & dist from other node
void MobileArmBotArm::laserScanCallback(const sensor_msgs::LaserScan &msg)
{
   this->chosen_index = 0;
   this->closest_target = 9999.99;
   ROS_INFO_STREAM("Laser message received!");
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
   if (this->closest_target <= this->in_range) 
   {
      ROS_INFO_STREAM("Starting if block...");
      this->arm_group.setNamedTarget("arm_getobject"); 
      this->arm_group.move();
      ROS_INFO_STREAM("Moving to get object target...");
      ros::WallDuration(1.0).sleep();
      this->closeEndEffector();
      ros::WallDuration(1.0).sleep();
      this->arm_group.setNamedTarget("home");
      this->arm_group.move();
      ros::WallDuration(1.0).sleep();
      return;
   }
}

void MobileArmBotArm::openEndEffector()
{
   ROS_INFO_STREAM("Opening end effector...");
   this->claw_group.setNamedTarget("open");
   this->claw_group.move();
}

void MobileArmBotArm::closeEndEffector()
{
   ROS_INFO_STREAM("Closing end effector...");
   this->claw_group.setNamedTarget("close");
   this->claw_group.move();
}
