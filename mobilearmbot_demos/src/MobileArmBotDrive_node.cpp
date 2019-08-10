#include "MobileArmBotDrive.hpp"

int main(int argc, char **argv)
{
   ros::init(argc, argv, "mobilearmbot_drive");
   ros::NodeHandle nh;

   MobileArmBotDrive mobilearmbotDrive(nh);

   ros::spin();
   return 0;
}
