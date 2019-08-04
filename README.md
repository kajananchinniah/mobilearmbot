# mobilearmbot

This is a ROS package that I'm working on. My end goal is to create a four wheel mobile manipulator.

This robot will probably not be viable to create in real life. My intent is to familarize myself with ROS, rather than
create a real robot. The turtlebot3 was also a 2wd, which meant that with a manipulator, it was prone to tipping, which
is why I chose to do a 4wd w/ a manipulator.

Currently, I just want to have the drivetrain working, so most of my efforts will be directed towards that. I plan to start
working on the manipulator once the drivetrain is completely done. For sensors, I plan on having al laser scan, imu & camera.
The manipulator will be heavily based on the open manipulator x from robotis.

Currently, the robot subscribes to cmd_vel (using the gazebo plugins), and is able to use a laser scanner. 
