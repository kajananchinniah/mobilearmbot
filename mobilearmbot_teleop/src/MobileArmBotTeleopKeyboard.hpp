//ROS 
#include <ros/ros.h>

//Include specific things from ROS
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

#include <controller_manager_msgs/SwitchController.h> //To be used in initializing the node

//Other libraries
#include <ncurses.h>

//STL libraries
#include <iostream>
#include <vector>
#include <string>

//constants relevant to increment/decrementing stuff
const double DRIVE_LIN_VEL_STEP_SIZE = 0.01;
const double DRIVE_ANG_VEL_STEP_SIZE = 0.1;
const double ARM_JOINT_POS_STEP_SIZE = 0.1;
const double END_EFFECTOR_OPEN_POS = 0.00;
const double END_EFFECTOR_CLOSE_POS = 1.9;

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
      ros::Publisher arm_link1_joint_pub;
      ros::Publisher arm_link2_joint_pub;
      ros::Publisher arm_link3_joint_pub;
      std::vector<ros::Publisher> arm_joint_pub_vec;

      std_msgs::Float64 arm_link1_joint_pos;
      std_msgs::Float64 arm_link2_joint_pos;
      std_msgs::Float64 arm_link3_joint_pos;
      std::vector<std_msgs::Float64> arm_joint_pos_vec;
      int curr_joint_idx;
      int arm_size; // DO NOT CONSIDER END EFFECTOR HERE
      void initializeArm();
      void teleopArm(char user_input);

      //Left and Right end effector variables and functions
      ros::Publisher left_eef_joint_pub;
      ros::Publisher right_eef_joint_pub;
      std_msgs::Float64 left_eef_joint_pos;
      std_msgs::Float64 right_eef_joint_pos;
      
      void initializeEndEffector();
      void teleopEndEffector(char user_input);
      
      
      //Variables and functions used for both arm and drive
      void printMessage();
      void teleop();

};
