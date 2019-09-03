#include "MobileArmBotTeleopKeyboard.hpp"

int main(int argc, char **argv)
{
   ros::init(argc, argv, "mobilearmbot_teleop");
   ros::NodeHandle nh;

   //Switching Controllers
   ros::ServiceClient switch_controller_client = nh.serviceClient<controller_manager_msgs::SwitchController>(
		   "mobilearmbot/controller_manager/switch_controller");

   controller_manager_msgs::SwitchController::Request req;
   controller_manager_msgs::SwitchController::Response resp;

   std::vector<std::string> start_controllers_names{"arm_link1_joint_position", "arm_link2_joint_position", 
	   "arm_link3_joint_position", "left_end_effector_joint_position", "right_end_effector_joint_position"};

   std::vector<std::string> stop_controllers_names{"mobilearmbot_arm_controller", "mobilearmbot_claw_controller"};
   int strictness = 1; // best effort 
   req.start_controllers = start_controllers_names;
   req.stop_controllers = stop_controllers_names;
   req.strictness = strictness;

   bool success = switch_controller_client.call(req, resp);
   
   if (success)
   {
      ROS_WARN_STREAM("NOTE: mobilearmbot_arm_controller and mobilearmbot_claw_controller have stopped running! So anything relying on them sHOULD NOT be running. Use q to exit and switch back upon teleop executation");
      MobileArmBotTeleop mobilearmbotTeleop(nh);
      ROS_WARN_STREAM("Switching back to mobilearmbot_arm_controller and mobilearmbot_claw_controller");
      req.start_controllers = stop_controllers_names;
      req.stop_controllers = start_controllers_names;
      req.strictness = strictness;
      bool success_back = switch_controller_client.call(req, resp);
      if (!success_back) { ROS_ERROR_STREAM("Cannot switch back!"); }
   }

   else
   {
      ROS_ERROR_STREAM("Unable to switch controllers!");
   }

   return 0;
}
