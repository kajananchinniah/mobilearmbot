#include "MobileArmBotArm.hpp"

MobileArmBotArm::MobileArmBotArm(ros::NodeHandle nh) 
	: arm_group("mobilearmbot_arm"), claw_group("mobilearmbot_claw")
{ 
   this->queue_size = 5;
   this->start_grasp = false;
   this->laser_topic_name = "scan";
   this->nh = nh;
   this->laser_sub = this->nh.subscribe(this->laser_topic_name,
		   this->queue_size, &MobileArmBotArm::laserScanCallback,
		   this);
}

//TODO: should probably just publish the heading angle & dist from other node
void MobileArmBotArm::laserScanCallback(const sensor_msgs::LaserScan &msg)
{
   this->chosen_index = 0;
   this->closest_target = 9999.99;
//   ROS_INFO_STREAM("Laser message received!");
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
      this->start_grasp = true;
      this->arm_group.setNamedTarget("arm_getobject");
      this->arm_group.move();
      this->closeEndEffector();
}

void MobileArmBotArm::openEndEffector()
{
   ROS_INFO_STREAM("Opening end effector...");
   this->claw_group.setNamedTarget("open");
   this->claw_group.move();
   ROS_INFO_STREAM("Done opening end effector");
}

void MobileArmBotArm::closeEndEffector()
{
   ROS_INFO_STREAM("Closing end effector...");
   this->claw_group.setNamedTarget("close");
   this->claw_group.move();
   ROS_INFO_STREAM("Done closing end effector...");
   return;
}
