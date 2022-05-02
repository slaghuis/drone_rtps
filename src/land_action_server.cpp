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

/* ************************************************************************
 * ROS2 Action Server that will issue a Land command to PX4 and wait
 * untill:
 *   The correct flight altitude has been reached
 *   a timout occurs.
 *
 * An alternative strategy might be to monitor the PX4 state untill the 
 * craft has switched out of Land state.  Think about it.
 * ***********************************************************************/
#include "drone_rtps/land_action_server.hpp"

//using Land = drone_interfaces::action::Land;
//using GoalHandleLand = rclcpp_action::ServerGoalHandle<Land>;

LandActionServer::LandActionServer(const rclcpp::NodeOptions & options)
  : Node("Land_action_server", options)
{
  using namespace std::placeholders;

  tf_buffer_ =
    std::make_unique<tf2_ros::Buffer>(this->get_clock());
  transform_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
         
  // get common timestamp
		timesync_sub_ =
			this->create_subscription<px4_msgs::msg::Timesync>("fmu/timesync/out", 10,
				[this](const px4_msgs::msg::Timesync::UniquePtr msg) {
					timestamp_.store(msg->timestamp);
				});   
      
  // PX4 Subscribers
  arming_state_ = 0;  // ARMING_STATE_INIT
  navigation_state_ = 0;  // NAVIGATION_STATE_MANUAL 
  old_navigation_state_ = 0;  
    
  vehicle_status_subscription_ = create_subscription<px4_msgs::msg::VehicleStatus>(
    "/fmu/vehicle_status/out",
    10,
    [this](px4_msgs::msg::VehicleStatus::UniquePtr msg) {
      arming_state_ = msg->arming_state;
      navigation_state_ = msg->nav_state;
    });  
    
  // PX4 Publishers
    
  vehicle_command_publisher_ =
			this->create_publisher<px4_msgs::msg::VehicleCommand>("fmu/vehicle_command/in", 10);
  
  setpoint_publisher_ =
			this->create_publisher<geometry_msgs::msg::PoseStamped>("drone/setpoint", 10);     
    
  this->action_server_ = rclcpp_action::create_server<Land>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
      "drone/land",
    std::bind(&LandActionServer::handle_goal, this, _1, _2),
    std::bind(&LandActionServer::handle_cancel, this, _1),
    std::bind(&LandActionServer::handle_accepted, this, _1));
  }

rclcpp_action::GoalResponse LandActionServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const Land::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received request to land.");
  (void)uuid;
  (void)goal;
     
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse LandActionServer::handle_cancel(
  const std::shared_ptr<GoalHandleLand> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel Land.");  // Over to the flight controller to handle this one I guess?
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void LandActionServer::execute(const std::shared_ptr<GoalHandleLand> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing Land");
  rclcpp::Rate loop_rate(1); 
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<Land::Feedback>();
  auto &current_altitude = feedback->current_altitude;
  auto result = std::make_shared<Land::Result>();

  std::string fromFrame = "base_link_ned"; // As published by the controller node in this package;
  std::string toFrame = "map";
  
  // Initialise the feedback.
  current_altitude = 0.0;   // Assume we do a takoff from the ground   
  rclcpp::Time timeout = this->get_clock()->now() + rclcpp::Duration(25, 0);  // 25 second timeout  
  
  // Change to Offboard mode after 10 setpoints
  // See https://github.com/PX4/PX4-Autopilot/blob/6de5d24e0056df71f2416586d83766a340039899/src/modules/commander/px4_custom_mode.h

  // PX4_CUSTOM_MAIN_MODE_AUTO = 4
  // PX4_CUSTOM_SUB_MODE_AUTO_LAND = 6  
  this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 4, 6);

// The controller node has been priming the flight controller,
  // Publish the desired setpoint
  this->set_land_altitude(0.0);
      
  // Now, monitor the flight
  while ( (this->get_clock()->now() < timeout)  && 
          (arming_state_ == 2) ) {
    
    if(navigation_state_ != old_navigation_state_) {
      RCLCPP_WARN(this->get_logger(), "Navigation state changed to : %d - Arm state %d", navigation_state_, arming_state_);
      old_navigation_state_ = navigation_state_;
    }  
    RCLCPP_DEBUG(this->get_logger(), "Current altitude %.2f", current_altitude);
   
    // Check if there is a cancel request
    if (goal_handle->is_canceling()) {
      result->result = true;
      goal_handle->canceled(result);
      RCLCPP_WARN(this->get_logger(), "Land canceled");
      return;
    }

    // Look up for the transformation between map and base_link frame to determine altitude
    geometry_msgs::msg::TransformStamped transformStamped;
    try {
      transformStamped = tf_buffer_->lookupTransform(
        toFrame, fromFrame,
        tf2::TimePointZero);
      
      current_altitude = transformStamped.transform.translation.z;
      
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(),"Failed to transform to %s from %s: %s", toFrame.c_str(), fromFrame.c_str(), ex.what());
      // return;  Let the fligt continue.  Eventually the transform might come through or we will timout.  PX4 has got this one
    }

    // Publish feedback
    goal_handle->publish_feedback(feedback);

    loop_rate.sleep();
  }          

  // Check if goal is done
  if (rclcpp::ok()) {
    result->result = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Land succeeded");
  }
}

void LandActionServer::handle_accepted(const std::shared_ptr<GoalHandleLand> goal_handle)
{
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&LandActionServer::execute, this, _1), goal_handle}.detach();
}

// UTILITY FUNCTIONS /////////////////////////////////////////////////////////////////////////////////
void LandActionServer::vehicle_command_Land() const {
	px4_msgs::msg::VehicleCommand msg{};
	msg.timestamp = timestamp_.load();
	msg.command = msg.VEHICLE_CMD_NAV_LAND;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;

	vehicle_command_publisher_->publish(msg);
}

bool LandActionServer::read_position(float *x, float *y, float *z, float *w)
{  
  std::string from_frame = "base_link_ned"; 
  std::string to_frame = "map";
      
  geometry_msgs::msg::TransformStamped transformStamped;
    
  try {
    transformStamped = tf_buffer_->lookupTransform(
      to_frame, from_frame,
      tf2::TimePointZero);
      *x = transformStamped.transform.translation.x;
      *y = transformStamped.transform.translation.y;
      *z = transformStamped.transform.translation.z;
      
      tf2::Quaternion q{
        transformStamped.transform.rotation.x,
        transformStamped.transform.rotation.y,
        transformStamped.transform.rotation.z,
        transformStamped.transform.rotation.w};
  
      // 3x3 Rotation matrix from quaternion
      tf2::Matrix3x3 m(q);

      // Roll Pitch and Yaw from rotation matrix
      double roll, pitch, yaw; 
      m.getRPY(roll, pitch, yaw);  
      *w = yaw;  
  } catch (tf2::TransformException & ex) {
    RCLCPP_DEBUG(
      this->get_logger(), "Could not transform %s to %s: %s",
      to_frame.c_str(), from_frame.c_str(), ex.what());
    return false;  
  }
  return true;
}

void LandActionServer::set_land_altitude(float altitude) {

  rclcpp::Time now = this->get_clock()->now();
  auto message = geometry_msgs::msg::PoseStamped();
  message.header.stamp = now; 
  message.header.frame_id ="map";

  float x,y,z,yaw;
  if (read_position(&x, &y, &z, &yaw)) {  
    message.pose.position.x = x;
    message.pose.position.y = y;
    message.pose.position.z = altitude;
    
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    message.pose.orientation.x = q[0];
    message.pose.orientation.y = q[1];
    message.pose.orientation.z = q[2];
    message.pose.orientation.w = q[3];    
  } else {  
    message.pose.position.x = 0.0;
    message.pose.position.y = 0.0;
    message.pose.position.z = altitude;
    message.pose.orientation.x = 0;
    message.pose.orientation.y = 0;
    message.pose.orientation.z = 0;
    message.pose.orientation.w = 1;
  }  
  RCLCPP_WARN(this->get_logger(), "Set land position [%.2f,%.2f,%.2f,%.2f]", 
              message.pose.position.x, 
              message.pose.position.y, 
              message.pose.position.z, 
              yaw);
  
  setpoint_publisher_->publish(message);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 * @param param2    Command parameter 3
 */
void LandActionServer::publish_vehicle_command(uint16_t command, float param1,
					      float param2, float param3) const {
	px4_msgs::msg::VehicleCommand msg{};
	msg.timestamp = timestamp_.load();
	msg.param1 = param1;
	msg.param2 = param2;
	msg.param3 = param3;  
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;

	vehicle_command_publisher_->publish(msg);
}


