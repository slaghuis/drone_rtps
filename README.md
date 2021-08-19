# xdrone-Fast-RTPS
A ROS2 node that interfaces to PX4 using Fast RTPS and published enough to get he navigation stack running.
This thing runs in the Gazebo simulator, but I have not bable to test it against a flight controller yet.  I could not get the RTPS/ROS2 PX4-FastRTPS Bridge installed on the Raspberry Pi yet.   

# Features
This node publishes tf2 transform, although, since it is a static trasform I think it is better done in the lanch file.

# Outstanding
This node needs to be lifted to an action server, with actions for arm, takeoff, land and disarm.
