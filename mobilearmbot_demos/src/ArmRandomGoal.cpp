#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "move_to_random_pos");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  moveit::planning_interface::MoveGroupInterface group("mobilearmbot_arm");
  group.setRandomTarget();
  group.move();
  ros::waitForShutdown();
}
