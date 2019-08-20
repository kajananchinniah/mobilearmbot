# mobilearmbot

This is a ROS package that I'm working on. My end goal is to create a four wheel mobile manipulator.

This robot will probably not be viable to create in real life. My intent is to familarize myself with ROS, rather than create a real robot.

NOTE: for now, I will not be relying on my moveit package (since I need to write action servers as well). For now, I'll be relying on position controllers, and writing to them directly through simulation. This is less realistic, but also simpler. I did this because I lack the knowledge for now to write the action server, and how the controller is supposed to work exactly.

For sensors, I currently only plan on having a laser scan & a camera (camera will be added later). 

I'm planning on using a simple manipulator. 

Current file structure:
- mobilearmbot_description contains the urdf and xacro files that describe the robot. It also contains a rviz file that will be updated later. This directory also temporarily contains launch files
- mobilearmbot_moveit was autogenerated by the moveit setup assistant
- mobilearmbot_demos (currently working on this) will contain a demo of the robot driving to a can and picking it up, and then possibly driving back to the origin. This will not rely on sensors much, aside from the laser scan.
- mobilearmbot_control currently (not complete) will have two controllers; a position controller & a follow trajectory controller. This will also contain launch files for launching these two controllers. The action server source code will be located here as well.

Future plans:
- Add a kinect or other type of sensor to allow for perception purposes (will be following moveit tutorials for this)
- Setup the gazebo file (basically just have some worlds & launch files)
- Add a teleop in mobilearmbot_teleop. Control will be done via keyboard: drive will be W,A,S,D. Currently need to come up with a plan for controlling the manipulator
- Setup ROS' navigation stack & try to add SLAM to the robot (I also have to learn how SLAM works)
