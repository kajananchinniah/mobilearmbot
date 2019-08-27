#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "move_to_random_pos");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  moveit::planning_interface::MoveGroupInterface arm_group("mobilearmbot_arm");
  moveit::planning_interface::MoveGroupInterface claw_group("mobilearmbot_claw");

  arm_group.setNamedTarget("arm_getobject");
  arm_group.move();
  ros::WallDuration(1.0).sleep();
  claw_group.setNamedTarget("open");
  claw_group.move();
  ros::WallDuration(1.0).sleep();
  arm_group.setNamedTarget("home");
  arm_group.move();
  ros::WallDuration(1.0).sleep();
  claw_group.setNamedTarget("close");
  claw_group.move();
  ros::WallDuration(1.0).sleep();
  arm_group.setRandomTarget();
  arm_group.move();
  ros::waitForShutdown();
}
