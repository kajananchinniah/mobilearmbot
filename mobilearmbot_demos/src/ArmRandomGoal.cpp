#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "move_to_random_pos");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  moveit::planning_interface::MoveGroupInterface arm_group("mobilearmbot_arm");
  arm_group.setRandomTarget();
  arm_group.move();
  ros::waitForShutdown();
}
