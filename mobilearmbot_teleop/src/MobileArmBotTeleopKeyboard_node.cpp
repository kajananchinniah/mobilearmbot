#include "MobileArmBotTeleopKeyboard.hpp"

int main(int argc, char **argv)
{
   ros::init(argc, argv, "mobilearmbot_teleop");
   ros::NodeHandle nh;

   MobileArmBotTeleop mobilearmbotTeleop(nh);
   return 0;
}
