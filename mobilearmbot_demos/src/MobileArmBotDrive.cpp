#include "MobileArmBotDrive.hpp"

MobileArmBotDrive::MobileArmBotDrive(ros::NodeHandle nh)
{
   ROS_INFO_STREAM("Executed!");
   this->queue_size = 5;
   this->laser_topic_name = "scan";
   this->kp_vel = 0.1;
   this->kp_ang = 0.2;
   this->nh = nh;

   this->laser_sub = this->nh.subscribe(this->laser_topic_name, 
		   this->queue_size, &MobileArmBotDrive::laserScanCallback, 
		   this);
   this->cmd_vel_pub = this->nh.advertise<geometry_msgs::Twist>("cmd_vel", 
		   this->queue_size);
   this->target_distance_pub = this->nh.advertise<std_msgs::Float32>("target_distance", this->queue_size);
}


void MobileArmBotDrive::laserScanCallback(const sensor_msgs::LaserScan &msg)
{
   this->chosen_index = 0;
   this->closest_target = 9999.99;
   ROS_INFO_STREAM("Laser scan message received!");
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
	   ROS_ERROR_STREAM("Cannot find target!");
	   this->vel.linear.x = 0.00;
	   this->vel.linear.z = 0.00;
	   this->cmd_vel_pub.publish(this->vel);
	   return;
   }

   this->heading_angle = msg.angle_min + this->chosen_index * msg.angle_increment;
   this->mobilearmbot_drive_controller(this->closest_target, this->heading_angle);
   ROS_INFO_STREAM("Heading Angle = " << this->heading_angle << " Vel = " << this->vel);
   this->cmd_vel_pub.publish(this->vel);
   this->closest_target_msg.data = this->closest_target;
   this->target_distance_pub.publish(this->closest_target_msg);
}

void MobileArmBotDrive::mobilearmbot_drive_controller(float dist, float angle)
{
   float angle_error = 0.0 - angle;
   float dist_error = 0.0 - dist;
   this->vel.linear.x = -this->kp_vel * dist_error;
   this->vel.angular.z = -this->kp_ang * angle_error;
}
