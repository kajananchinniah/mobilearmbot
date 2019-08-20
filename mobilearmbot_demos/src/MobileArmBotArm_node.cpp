#include "MobileArmBotArm.hpp"

int main(int argc, char **argv)
{
   ros::init(argc, argv, "arm_controller");
   ros::NodeHandle nh;

   MobileArmBotArm mobilearmbotArm(nh);
   ros::spin();
   return 0;
}
