#include "MobileArmBotTeleopKeyboard.hpp"

MobileArmBotTeleop::MobileArmBotTeleop(ros::NodeHandle nh)
{
   this->queue_size = 1;
   this->nh = nh;
   //TODO: should consider some checks with user_input to ensure it has a valid value
   this->user_input = '0'; // Temporary value; will be overwritten

   this->drive_vel_pub = this->nh.advertise<geometry_msgs::Twist>("mobilearmbot/cmd_vel", this->queue_size);

   //Starting teleop
   this->teleop();
}

void MobileArmBotTeleop::teleop()
{
   //Stopping drive train
   this->drive_vel.linear.x = 0.00;
   this->drive_vel.angular.z = 0.00;
   this->drive_vel_pub.publish(this->drive_vel);

   //Setting up ncurses
   initscr();
   cbreak();
   timeout(1000);
   while(ros::ok())
   {
      this->printMessage();
      this->user_input = getch();
      this->teleopDrive(user_input);
      //TODO: add this->teleopArm(user_input) when made..
      if (this->user_input == 'q')
      {
         this->drive_vel.angular.z = 0.00;
	 this->drive_vel.linear.x = 0.00;
	 this->drive_vel_pub.publish(this->drive_vel);
	 break;
      }

   }
   nocbreak();
   endwin();
}

void MobileArmBotTeleop::printMessage()
{
   //Keyboard information
   std::cout << "w/x to increase/decrease linear velocity\n\r";
   std::cout << "a/d to increase/decrease angular velocity\n\r";
   std::cout << "s to force stop and q to quit\n\r";
   std::cout << "NOTE: s only stops the drivetrain\n\r"; 

   //Drive information
   std::cout << "Lin vel = " << this->drive_vel.linear.x << 
	   " Ang vel = " << this->drive_vel.angular.z << 
	   "\n\r";

   //Arm information
}


//Drive functions
void MobileArmBotTeleop::teleopDrive(const char user_input)
{
   //TODO: use a switch statement later 
   if (user_input == 'w')
   {
      this->drive_vel.linear.x = this->drive_vel.linear.x + DRIVE_LIN_VEL_STEP_SIZE;
   }

   if (user_input == 'x')
   {
      this->drive_vel.linear.x = this->drive_vel.linear.x - DRIVE_LIN_VEL_STEP_SIZE;
   }

   if (user_input == 'a')
   {
	   this->drive_vel.angular.z = this->drive_vel.angular.z + DRIVE_ANG_VEL_STEP_SIZE;
   }

   if (user_input == 'd')
   {
      this->drive_vel.angular.z = this->drive_vel.angular.z - DRIVE_ANG_VEL_STEP_SIZE;
   }

   if (user_input == 's')
   {
      this->drive_vel.angular.z = 0.00;
      this->drive_vel.linear.x = 0.00;
   }

   this->drive_vel_pub.publish(this->drive_vel);
}


//Arm functions
