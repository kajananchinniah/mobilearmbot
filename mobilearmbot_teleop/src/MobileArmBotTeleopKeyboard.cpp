#include "MobileArmBotTeleopKeyboard.hpp"

MobileArmBotTeleop::MobileArmBotTeleop(ros::NodeHandle nh)
{
   this->queue_size = 1;
   this->nh = nh;
   //TODO: should consider some checks with user_input to ensure it has a valid value
   this->user_input = '0'; // Temporary value; will be overwritten

   this->drive_vel_pub = this->nh.advertise<geometry_msgs::Twist>("mobilearmbot/cmd_vel", 
		   this->queue_size);

   //Setting up arm
   this->initializeArm();
   this->initializeEndEffector();
   
   //Starting teleop
   this->teleop();
}

void MobileArmBotTeleop::initializeArm()
{
   this->arm_link1_joint_pub = this->nh.advertise<std_msgs::Float64>("mobilearmbot/arm_link1_joint_position/command", 
		   this->queue_size);
   this->arm_link2_joint_pub = this->nh.advertise<std_msgs::Float64>("mobilearmbot/arm_link2_joint_position/command",
		   this->queue_size);
   this->arm_link3_joint_pub = this->nh.advertise<std_msgs::Float64>("mobilearmbot/arm_link3_joint_position/command",
		   this->queue_size);

   this->arm_joint_pub_vec.push_back(this->arm_link1_joint_pub);
   this->arm_joint_pub_vec.push_back(this->arm_link2_joint_pub);
   this->arm_joint_pub_vec.push_back(this->arm_link3_joint_pub);

   this->arm_link1_joint_pos.data = 0.00;
   this->arm_link2_joint_pos.data = 0.00;
   this->arm_link3_joint_pos.data = 0.00;

   this->arm_joint_pos_vec.push_back(this->arm_link1_joint_pos);
   this->arm_joint_pos_vec.push_back(this->arm_link2_joint_pos);
   this->arm_joint_pos_vec.push_back(this->arm_link3_joint_pos);

   this->curr_joint_idx = 0;
   this->arm_size = arm_joint_pub_vec.size();
  
   //Resetting arm position
   for (int i = 0; i < this->arm_size; i++)
   {
      this->arm_joint_pub_vec[i].publish(this->arm_joint_pos_vec[i]);
   }
}

void MobileArmBotTeleop::initializeEndEffector()
{
   this->left_eef_joint_pub = this->nh.advertise<std_msgs::Float64>(
		   "mobilearmbot/left_end_effector_joint_position/command", this->queue_size);
   this->right_eef_joint_pub = this->nh.advertise<std_msgs::Float64>(
		   "mobilearmbot/right_end_effector_joint_position/command", this->queue_size);
   
   this->left_eef_joint_pos.data = 0.00;
   this->right_eef_joint_pos.data = 0.00;
   
   this->left_eef_joint_pub.publish(this->left_eef_joint_pos);
   this->right_eef_joint_pub.publish(this->right_eef_joint_pos);
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
      this->teleopArm(user_input);
      this->teleopEndEffector(user_input);

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
   std::cout << "i/k to increment/decrement to the next joint\n\r";
   std::cout << "j/l to increase/decrease joint position\n\r";
   std::cout << "o/p to open/close end effector\n\r";
   std::cout << "s to force stop and q to quit\n\r";
   std::cout << "NOTE: s only stops the drivetrain\n\r";

   //Drive information
   std::cout << "Lin vel = " << this->drive_vel.linear.x << 
	   " Ang vel = " << this->drive_vel.angular.z << 
	   "\n\r";

   //Arm information
   for (int i = 0; i < this->arm_size; i++)
   {
      std::cout << "Position of arm_link" << (i+1)  << " = " << this->arm_joint_pos_vec[i].data << "\n\r";
   } 
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
//TODO: add a check to ensure we don't go outside of possible values
void MobileArmBotTeleop::teleopArm(char user_input)
{
   if (user_input == 'i')
   {
      this->curr_joint_idx++;
      this->curr_joint_idx = this->curr_joint_idx % this->arm_size; 
   }

   if (user_input == 'k')
   {
      this->curr_joint_idx--;
      if (this->curr_joint_idx < 0)
      {
         this->curr_joint_idx = this->arm_size - 1; // Hard code loop around
	 //TODO: find a better way to do this
      }
      this->curr_joint_idx = this->curr_joint_idx % this->arm_size;      
   }

   if (user_input == 'j')
   {
      this->arm_joint_pos_vec[this->curr_joint_idx].data -= ARM_JOINT_POS_STEP_SIZE;
   }

   if (user_input == 'l')
   {
      this->arm_joint_pos_vec[this->curr_joint_idx].data += ARM_JOINT_POS_STEP_SIZE;
   }
   this->arm_joint_pub_vec[this->curr_joint_idx].publish(this->arm_joint_pos_vec[this->curr_joint_idx]);
}

void MobileArmBotTeleop::teleopEndEffector(char user_input)
{
   if (user_input == 'o')
   {
      this->left_eef_joint_pos.data = END_EFFECTOR_OPEN_POS;
      this->right_eef_joint_pos.data = END_EFFECTOR_OPEN_POS;
   }

   if (user_input == 'p')
   {
      this->left_eef_joint_pos.data = END_EFFECTOR_CLOSE_POS;
      this->right_eef_joint_pos.data = -END_EFFECTOR_CLOSE_POS;
   }

   this->left_eef_joint_pub.publish(this->left_eef_joint_pos);
   this->right_eef_joint_pub.publish(this->right_eef_joint_pos);
}
