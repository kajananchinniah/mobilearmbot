#include "MobileArmBotArm.hpp"

int main(int argc, char **argv)
{
   ros::init(argc, argv, "mobilearmbot_arm_getobject");
   ros::NodeHandle nh;
   ros::AsyncSpinner spinner(1);

   MobileArmBotArm mobilearmbotArm(nh);
   spinner.start();
   ros::waitForShutdown();
   return 0;
}
