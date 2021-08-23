# drone_rtps
A ROS2 node that interfaces to PX4 using Fast RTPS and published enough to get he navigation stack running.
This thing runs in the Gazebo simulator, but I have not bable to test it against a flight controller yet.  This is planned for September 2021

# Dependencies
This node depends on the drone_interfaces package elsewhere in my repository

# Outstanding
This node needs to be tested in a simulator, and then compiled and tested on a flight controller.
Further enhancements could include a similar node that deals with world frames (GPS coordinates)
