#include <ros/ros.h>

#include <geometry_msgs/Twist.h>

#include <iostream>
#include <ncurses.h>

//TODO: should clean this up later; currently just being used to test this out
//TODO: add teleop for arm
//TODO: add absolute max values for ang and lin vels
const double LIN_VEL_STEP_SIZE = 0.01;
const double ANG_VEL_STEP_SIZE = 0.1;

int main(int argc, char **argv)
{
   ros::init(argc, argv, "mobilearmbot_teleop");
   ros::NodeHandle nh;

   ros::Publisher drive_cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("mobilearmbot/cmd_vel", 1);

   geometry_msgs::Twist drive_vel_cmd;
   drive_vel_cmd.linear.x = 0.00;
   drive_vel_cmd.angular.z = 0.00;
   char drive_vel_input;
   initscr();
   cbreak();
   timeout(1000);
   while(ros::ok())
   {
      std::cout << "w/x to increase/decrease linear velocity\n\r";
      std::cout << "a/d to increase/decrease angular velocity\n\r";
      std::cout << "s to force stop and q to quit\n\r";
      drive_vel_input = getch();
      if (drive_vel_input == 'w')
      {
         drive_vel_cmd.linear.x = drive_vel_cmd.linear.x + LIN_VEL_STEP_SIZE;
      }

      if (drive_vel_input == 'x')
      {
         drive_vel_cmd.linear.x = drive_vel_cmd.linear.x - LIN_VEL_STEP_SIZE;
      }

      if (drive_vel_input == 'a')
      {
         drive_vel_cmd.angular.z = drive_vel_cmd.angular.z + ANG_VEL_STEP_SIZE;
      }

      if (drive_vel_input == 'd')
      {
         drive_vel_cmd.angular.z = drive_vel_cmd.angular.z - ANG_VEL_STEP_SIZE;
      }

      if (drive_vel_input == 's')
      {
         drive_vel_cmd.angular.z = 0.00;
	 drive_vel_cmd.linear.x = 0.00;
      }

      if (drive_vel_input == 'q')
      {
         drive_vel_cmd.angular.z = 0.00;
	 drive_vel_cmd.linear.x = 0.00;
	 drive_cmd_vel_pub.publish(drive_vel_cmd);
	 break;
      }
      
      drive_cmd_vel_pub.publish(drive_vel_cmd);
      std::cout << "Lin vel = " << drive_vel_cmd.linear.x << " Ang vel = " << drive_vel_cmd.angular.z << "\n\r";
   }
   nocbreak();
   endwin();
   return 0;
}
