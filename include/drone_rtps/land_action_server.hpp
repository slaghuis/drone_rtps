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

#include <functional>
#include <memory>
#include <thread>
#include <cmath>    // abs()

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "drone_interfaces/action/land.hpp"

#include "drone_rtps/visibility_control.h"

class LandActionServer : public rclcpp::Node
{
public:
  using Land = drone_interfaces::action::Land;
  using GoalHandleLand = rclcpp_action::ServerGoalHandle<Land>;
  
  DRONE_PUBLIC explicit LandActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:  

  uint8_t arming_state_;
  uint8_t navigation_state_;
  uint8_t old_navigation_state_;
  
  rclcpp_action::Server<Land>::SharedPtr action_server_;
  
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
  
  rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_subscription_;
  
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_publisher_;
  
	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
  
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Land::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleLand> goal_handle);
  void execute(const std::shared_ptr<GoalHandleLand> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleLand> goal_handle);
  
  void vehicle_command_Land() const;
  void publish_offboard_control_mode() const;
  bool read_position(float *x, float *y, float *z, float *w);
  void set_land_altitude(float altitude);
  void publish_vehicle_command(uint16_t command, float param1, float param2, float param3) const;
};  // class LandActionServer