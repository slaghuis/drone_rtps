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

/* *****************************************************************************
 * This is how you fly a drone via PX4 FastRTPS
 * This node perpetually publishes:
 *   - Offboard Control Mode (as setpoint messages)
 *   - The required trajectory setpoint
 * at a frequency of 10 Hz. If the drone has to move to another location
 * then send a message to this node , and tis will ater the setpoint.  
 * The moment this node stops publishing, the flight controller will tigger
 * a faisafe (no RC and no offboard) and land on the spot.  I guess if you 
 * have a Remote Controller then the flight controller will change to HOLD mode.
 * *****************************************************************************/
#include "drone_rtps/drone_controller_node.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;

ControllerNode::ControllerNode(rclcpp::NodeOptions options)
: Node("drone_controller_node", options)
{
  // TF2 
  tf_buffer_ =
    std::make_unique<tf2_ros::Buffer>(this->get_clock());

  tf_broadcaster_ =
    std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  
  // ROS2 Publishers
  odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("drone/odom", 10);
  battery_publisher_ = this->create_publisher<sensor_msgs::msg::BatteryState>("drone/battery", 5);
  nav_sat_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("drone/gps", 5);
    
  // ROS2_Subscribers
  setpoint_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "drone/setpoint", 10, std::bind(&ControllerNode::setpoint_callback, this, std::placeholders::_1));
  
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

  px4_odom_subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
		  "fmu/vehicle_odometry/out",10, std::bind(&ControllerNode::px4_odom_callback, this, std::placeholders::_1));
  px4_gps_subscription_ = this->create_subscription<px4_msgs::msg::VehicleGpsPosition>(
		  "fmu/vehicle_odometry/out",10, std::bind(&ControllerNode::px4_gps_callback, this, std::placeholders::_1));
  px4_battery_subscription_ = this->create_subscription<px4_msgs::msg::BatteryStatus>(
		  "fmu/vehicle_odometry/out",10, std::bind(&ControllerNode::px4_battery_callback, this, std::placeholders::_1));
  
  offboard_control_mode_publisher_ =
    this->create_publisher<px4_msgs::msg::OffboardControlMode>("fmu/offboard_control_mode/in", 10);
  trajectory_setpoint_publisher_ =
    this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("fmu/trajectory_setpoint/in", 10);

  // get common timestamp
  timesync_sub_ =
    this->create_subscription<px4_msgs::msg::Timesync>("fmu/timesync/out", 10,
      [this](const px4_msgs::msg::Timesync::UniquePtr msg) {
        timestamp_.store(msg->timestamp);
      });

    offboard_setpoint_counter_ = 0;

    auto timer_callback = [this]() -> void {

      // offboard_control_mode needs to be paired with trajectory_setpoint
      if(navigation_state_ != 18) {   // 18 is a member of PX4_CUSTOM_MAIN_MODE_AUTO, in this case PX4_CUSTOM_SUB_MODE_AUTO_LAND
        publish_offboard_control_mode();
        publish_trajectory_setpoint();
      }  

    };
    timer_ = this->create_wall_timer(100ms, timer_callback);
  
}

void ControllerNode::publish_offboard_control_mode() const {
	px4_msgs::msg::OffboardControlMode msg{};
	msg.timestamp = timestamp_.load();
	msg.position = true;
	msg.velocity = true;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;

	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void ControllerNode::publish_trajectory_setpoint() {
	setpoint_msg_.timestamp = timestamp_.load();

	trajectory_setpoint_publisher_->publish(setpoint_msg_);
}

// PX4 Subscribers //////////////////////////////////////////////////////////////////////
void ControllerNode::px4_odom_callback(const px4_msgs::msg::VehicleOdometry::UniquePtr px4_msg)  
{
  
  rclcpp::Time now = this->get_clock()->now();
          
  geometry_msgs::msg::TransformStamped t;

  // Read message content and assign it to
  // corresponding tf variables
  t.header.stamp = now;
  t.header.frame_id = "odom_ned";
  t.child_frame_id = "base_link_ned";
          
  t.transform.translation.x = px4_msg->x;
  t.transform.translation.y = px4_msg->y;
  t.transform.translation.z = px4_msg->z; 
         
  // Adopt the roll pitch and yaw from the drone.
  t.transform.rotation.x = px4_msg->q[0];
  t.transform.rotation.y = px4_msg->q[1];
  t.transform.rotation.z = px4_msg->q[2];
  t.transform.rotation.w = px4_msg->q[3];
  tf_broadcaster_->sendTransform(t);    

  auto message = nav_msgs::msg::Odometry();
    
  message.header.stamp = now; 
  message.header.frame_id ="odom_ned";

  message.pose.pose.position.x = px4_msg->x;
  message.pose.pose.position.y = px4_msg->y;
  message.pose.pose.position.z = px4_msg->z;
    
  message.pose.pose.orientation.x = px4_msg->q[0];  //Does PX4 and ROS use the same sequence?
  message.pose.pose.orientation.y = px4_msg->q[1];
  message.pose.pose.orientation.z = px4_msg->q[2];
  message.pose.pose.orientation.w = px4_msg->q[3]; 
        
  message.pose.covariance[0] = px4_msg->pose_covariance[0];
  message.pose.covariance[7] = px4_msg->pose_covariance[6];
  message.pose.covariance[14] = px4_msg->pose_covariance[11];      
  message.pose.covariance[21] = px4_msg->pose_covariance[15];
  message.pose.covariance[28] = px4_msg->pose_covariance[18];
  message.pose.covariance[35] = px4_msg->pose_covariance[20];
        
  message.twist.twist.linear.x = px4_msg->vx;
  message.twist.twist.linear.y = px4_msg->vy;
  message.twist.twist.linear.z = px4_msg->vz;
            
  message.twist.twist.angular.x = px4_msg->pitchspeed;;
  message.twist.twist.angular.y = px4_msg->rollspeed;
  message.twist.twist.angular.z = px4_msg->yawspeed;

  message.twist.covariance[0] = px4_msg->velocity_covariance[0];
  message.twist.covariance[7] = px4_msg->velocity_covariance[6];
  message.twist.covariance[14] = px4_msg->velocity_covariance[11];      
  message.twist.covariance[21] = px4_msg->velocity_covariance[15];
  message.twist.covariance[28] = px4_msg->velocity_covariance[18];
  message.twist.covariance[35] = px4_msg->velocity_covariance[20];

    
  odom_publisher_->publish(message);
}

void ControllerNode::px4_gps_callback(const px4_msgs::msg::VehicleGpsPosition::UniquePtr px4_msg)  
{
  rclcpp::Time now = this->get_clock()->now();

  auto message = sensor_msgs::msg::NavSatFix();
    
  message.header.stamp = now; 
  message.header.frame_id ="odom_ned";
  
  message.latitude = px4_msg->lat;
  message.longitude = px4_msg->lon;
  message.altitude = px4_msg->alt;
  
  message.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  
  nav_sat_publisher_->publish(message);
}

void ControllerNode::px4_battery_callback(const px4_msgs::msg::BatteryStatus::UniquePtr px4_msg)  
{
  rclcpp::Time now = this->get_clock()->now();

  auto message = sensor_msgs::msg::BatteryState();
    
  message.header.stamp = now; 
  message.header.frame_id ="odom_ned";
  
  message.voltage = px4_msg->voltage_v;
  message.temperature =  px4_msg->temperature;
  message.current = px4_msg->current_a;
  message.charge = px4_msg->remaining_capacity_wh / px4_msg->nominal_voltage;
  message.capacity = (float)px4_msg->capacity;
  message.design_capacity = px4_msg->full_charge_capacity_wh / px4_msg->nominal_voltage;
  message.percentage = px4_msg->remaining;
  message.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  message.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  message.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
  message.present = true;
  for(auto i = 0; i<6; i++) {
    message.cell_voltage[i] = px4_msg->voltage_cell_v[i];
    message.cell_temperature[i] = NAN;
  }
  battery_publisher_->publish(message);
}

// ROS2 SUBSCRIBERS /////////////////////////////////////////////////////////////////////////////////////////
void ControllerNode::setpoint_callback(const geometry_msgs::msg::PoseStamped::UniquePtr msg) 
{
  // Receive messages from PX4 in ENU x=East, y=North, z=Up
  // in the map frame.
  
  // pass to PX4 in NED x=North, y=East, z=Down
  setpoint_msg_.x = msg->pose.position.y;
  setpoint_msg_.y = msg->pose.position.x;
  setpoint_msg_.z = -msg->pose.position.z;
  
  tf2::Quaternion q{
    msg->pose.orientation.x,
    msg->pose.orientation.y,
    msg->pose.orientation.z,
    msg->pose.orientation.w};
  
  // 3x3 Rotation matrix from quaternion
  tf2::Matrix3x3 m(q);

  // Roll Pitch and Yaw from rotation matrix
  double roll, pitch, yaw; 
  m.getRPY(roll, pitch, yaw);
    
  // CCW Yaw in ROS2 is positive; CCW yaw in PX4 is negative
  setpoint_msg_.yaw = yaw-1.570796;  // Subtract Pi/2 (90 degrees) because ROS 0 degrees is east, PX4 0 degrees is north. 
  if(setpoint_msg_.yaw < -M_PI) {    // [-PI:PI] 
    setpoint_msg_.yaw = M_PI + (setpoint_msg_.yaw + M_PI);
  }  
  
  setpoint_msg_.vx = NAN;
  setpoint_msg_.vy = NAN;
  setpoint_msg_.vz = NAN;
  setpoint_msg_.yawspeed = NAN;

  setpoint_msg_.acceleration[0] = NAN;
  setpoint_msg_.acceleration[1] = NAN;  
  setpoint_msg_.acceleration[2] = NAN;

  setpoint_msg_.jerk[0] = NAN;
  setpoint_msg_.jerk[1] = NAN;  
  setpoint_msg_.jerk[2] = NAN;

  RCLCPP_INFO(this->get_logger(), "Set takoff altitude %.2f and yaw %.2f", setpoint_msg_.z, setpoint_msg_.yaw);

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ControllerNode)