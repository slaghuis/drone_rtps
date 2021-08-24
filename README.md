# drone_rtps
A ROS2 node that interfaces to PX4 using Fast RTPS and published enough to get the navigation stack running.
This thing runs in the Gazebo simulator, but I have not bable to test it against a flight controller yet.  This is planned for September 2021

## Dependencies
This node depends on the [drone_interfaces package](https://github.com/slaghuis/drone_interfaces).

## Coordinate Systems
PX4 uses a Foreward Right Down coordinate system, and ROS2 uses a Foreward Left Up coodinate system.  To align the two, a static transform boradcaster has to be started.  This is demonstrated in the launch file.

# node info
```
eric@simulator:~/ros_ws$ ros2 node info /drone_node
/drone_node
  Subscribers:
    /drone/cmd_vel: geometry_msgs/msg/Twist
  Publishers:
    /drone/odom: nav_msgs/msg/Odometry
  Service Servers:
    /drone/arm: drone_interfaces/srv/Arm
    /drone/offboard: drone_interfaces/srv/Offboard
  Service Clients:  
  
  Action Servers:
    /drone/land: drone_interfaces/action/Land
    /drone/takeoff: drone_interfaces/action/Takeoff
  Action Clients:
  
eric@simulator:~/ros_ws$
```
# Status
- The code compiles and runs on a Virtual Box machine running Ubuntu 20.04 LTS
- The code compiles and runs on a Raspberry Pi 4 4Mb running Ubuntu 20.04 LTS
- Code needs to be tested in the smulator (functional testing)
- Code neets to be tested against a real flight controller (Pixhawk 4 Mini)

## Test Scripts
Arm the drone
```
ros2 service call /drone/arm drone_interfaces/srv/Arm
```

Takeoff
```
ros2 action send_goal /drone/takeoff "drone_interfaces/action/Takeoff" "{target_altitude: 5.0}"
```

Land
```
ros2 action send_goal /drone/land "drone_interfaces/action/Land" "{gear_down: true}"
```

Enable offboard mode
```
ros2 service call /drone/offboard drone_interfaces/srv/Offboard "{enable: 0}"
```

Fly in a circle
```
ros2 topic pub --once /drone/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.5}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

# Caution
This code has not flown.
