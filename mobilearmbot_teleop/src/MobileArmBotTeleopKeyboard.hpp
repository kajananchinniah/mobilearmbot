#include <ros/ros.h>

#include <geometry_msgs/Twist.h>

#include <iostream>
#include <ncurses.h>

//constants relevant to increment/decrementing linear and angular velocities
const double DRIVE_LIN_VEL_STEP_SIZE = 0.01;
const double DRIVE_ANG_VEL_STEP_SIZE = 0.1;

class MobileArmBotTeleop
{
   public:
      MobileArmBotTeleop(ros::NodeHandle nh);

   private:
      ros::NodeHandle nh;
      int queue_size;

      //Drive variables and functions
      geometry_msgs::Twist drive_vel;
      ros::Publisher drive_vel_pub;
      char user_input;
      void teleopDrive(char user_input);

      //TODO: add joint variables and functions
      
      //Variables and functions used for both arm and drive
      void printMessage();
      void teleop();

};
