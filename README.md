# mobilearmbot

This is a ROS package that I'm working on. My end goal is to create a four wheel mobile manipulator.

This robot will probably not be viable to create in real life. My intent is to familarize myself with ROS, rather than create a real robot.

For sensors, I currently only plan on having a laser scan & a kinect (kinect will be added later). 

The thing I'm working on at the moment is having the robot able to pick up and object (without pose estimation or anything so it's mostly hardcoded).

I'm planning on using a simple manipulator. 
Current file structure:
- mobilearmbot_description contains the urdf and xacro files that describe the robot. It also contains a rviz file that will be updated later. This directory also temporarily contains launch files
- mobilearmbot_moveit was autogenerated by the moveit setup assistant
- mobilearmbot_demos (currently working on this) will contain a demo of the robot driving to a can and picking it up, and then possibly driving back to the origin. This will not rely on sensors much, aside from the laser scan. This demo will only work in the included world (the location of the object shouldn't matter too much as long as it's on the ground, but the type matters a lot)
- mobilearmbot_control currently uses position controllers (follow joint trajectory). It also contains a launch file for launching the control. 
- mobilearmbot_gazebo contains a launch file for launching gazebo, spawning mobilearmbot, and it's controller. It currently only contains one world, but more to come (I'll likely just take turtlebot3's worlds). Note: if you're using moveit, the launch file here does not spawn the srdf that describes the groups necessary for moveit so you'll need to manually include the launch file (move_group.launch) from the mobilearmbot_moveit pkg. 

Future plans:
- Add a kinect or other type of sensor to allow for perception purposes (will be following moveit tutorials for this)
- Setup the gazebo file (basically just have some worlds & launch files)
- Add a teleop in mobilearmbot_teleop. Control will be done via keyboard: drive will be W,A,S,D. Currently need to come up with a plan for controlling the manipulator
- Setup ROS' navigation stack & try to add SLAM to the robot (I also have to learn how SLAM works)

Resources & tutorials I consulted:
- http://wiki.ros.org/urdf/Tutorials
- http://docs.ros.org/melodic/api/moveit_tutorials/html/index.html# 
- https://github.com/AS4SR/general_info/wiki/Basic-ROS-MoveIt!-and-Gazebo-Integration


