#include "MobileArmBotArm.hpp"

int main(int argc, char **argv)
{
   ros::init(argc, argv, "mobilearmbot_arm_getobject");
   ros::NodeHandle nh;
   ros::AsyncSpinner spinner(1);
   spinner.start();

   MobileArmBotArm mobilearmbotArm(nh);
   ros::spin();
   return 0;
}
