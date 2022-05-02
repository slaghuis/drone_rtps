// Copyright 2022 Eric Slaghuis
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DRONE_CONTROLLER_NODE_HPP_
#define DRONE_CONTROLLER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/buffer.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_gps_position.hpp>
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include "drone_rtps/visibility_control.h"

# define M_PIl          3.141592653589793238462643383279502884L /* pi */

class ControllerNode : public rclcpp::Node
{
public:
  DRONE_PUBLIC ControllerNode(rclcpp::NodeOptions options);

private:
  // Variables
  px4_msgs::msg::TrajectorySetpoint setpoint_msg_;
  uint8_t arming_state_;
  uint8_t navigation_state_;
  uint8_t old_navigation_state_;
  
  // ROS2 Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_subscription_;
  
  // ROS2 Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr nav_sat_publisher_;
    
  // TF2 Boradcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  
  rclcpp::TimerBase::SharedPtr timer_;
  
  //PX4_Publishers
	rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
  
  // PX$ Subscribers 
	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_odom_subscription_;
  rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr px4_battery_subscription_;
  rclcpp::Subscription<px4_msgs::msg::VehicleGpsPosition>::SharedPtr px4_gps_subscription_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_subscription_;
  
	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

  void px4_odom_callback(const px4_msgs::msg::VehicleOdometry::UniquePtr px4_msg);
  void px4_battery_callback(const px4_msgs::msg::BatteryStatus::UniquePtr px4_msg);  
  void px4_gps_callback(const px4_msgs::msg::VehicleGpsPosition::UniquePtr px4_msg);
	void publish_offboard_control_mode() const;
	void publish_trajectory_setpoint() ;

  void setpoint_callback(const geometry_msgs::msg::PoseStamped::UniquePtr msg);

};

#endif  // DRONE_CONTROLLER_NODE_HPP_