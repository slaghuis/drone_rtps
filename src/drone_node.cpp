#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

// From drone_interfaces
#include "drone_interfaces/action/takeoff.hpp"
#include "drone_interfaces/action/land.hpp"
#include "drone_interfaces/srv/arm.hpp"
#include "drone_interfaces/srv/offboard.hpp"

// PX4 RTPS
#include <px4_msgs/msg/vehicle_gps_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/commander_state.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>

// ROS2 Message Types
#include "std_msgs/msg/string.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "drone_rtps/visibility_control.h"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

namespace drone_node {
  
class DroneNode : public rclcpp::Node
{
public:
  using Takeoff = drone_interfaces::action::Takeoff;
  using GoalHandleTakeoff = rclcpp_action::ServerGoalHandle<Takeoff>;
  using Land = drone_interfaces::action::Land;
  using GoalHandleLand = rclcpp_action::ServerGoalHandle<Land>;

  DRONE_NODE_PUBLIC
  explicit DroneNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("drone_commander", options)
  {
    last_altitude = 0.0;
    
    using namespace std::placeholders;
    // Services
    arm_service_ = this->create_service<drone_interfaces::srv::Arm>("drone/arm", 
      std::bind(&DroneNode::arm, this, _1, _2));

    offboard_service_ = this->create_service<drone_interfaces::srv::Offboard>("drone/offboard", 
      std::bind(&DroneNode::offboard, this, _1, _2));
    
    // Action servers
    this->takeoff_action_server_ = rclcpp_action::create_server<Takeoff>(
      this,
      "drone/takeoff",
      std::bind(&DroneNode::takeoff_handle_goal, this, _1, _2),
      std::bind(&DroneNode::takeoff_handle_cancel, this, _1),
      std::bind(&DroneNode::takeoff_handle_accepted, this, _1));

    this->land_action_server_ = rclcpp_action::create_server<Land>(
      this,
      "drone/land",
      std::bind(&DroneNode::land_handle_goal, this, _1, _2),
      std::bind(&DroneNode::land_handle_cancel, this, _1),
      std::bind(&DroneNode::land_handle_accepted, this, _1));


    // Subscribers
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "drone/cmd_vel", 10, std::bind(&DroneNode::cmd_vel_topic_callback, this, _1));

    // Publishers
    using namespace std::chrono_literals;
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("drone/odom", 10);
    //odom_timer_ = this->create_wall_timer( 500ms, std::bind(&DroneNode::odom_timer_callback, this));
    
    // PX4 Publishers
    offboard_control_mode_publisher_ =
        this->create_publisher<OffboardControlMode>("fmu/offboard_control_mode/in", 10);
    trajectory_setpoint_publisher_ =
        this->create_publisher<TrajectorySetpoint>("fmu/trajectory_setpoint/in", 10);
    vehicle_command_publisher_ =
        this->create_publisher<VehicleCommand>("fmu/vehicle_command/in", 10);
        
    // PX4 Subscribers   
    px4_odom_subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
		  "fmu/vehicle_odometry/out",10, std::bind(&DroneNode::px4_odom_callback, this, _1));
      
     // get common timestamp
     timesync_sub_ = this->create_subscription<px4_msgs::msg::Timesync>("fmu/timesync/out", 10,
         [this](const px4_msgs::msg::Timesync::UniquePtr msg) {
               timestamp_.store(msg->timestamp);
             });

  } // Constructor  

private:
  // General Variables
  float last_altitude;
  
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
  rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
  std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

  void publish_offboard_control_mode() const;
  void publish_trajectory_setpoint() const;
  void publish_vehicle_command(uint16_t command, float param1 = 0.0,
                               float param2 = 0.0) const;

  // PX4 Subscriptions
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_odom_subscription_;
  void px4_odom_callback(const px4_msgs::msg::VehicleOdometry::UniquePtr px4_msg);
  
  // ROS Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;

  // Subscriptions
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  void cmd_vel_topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const;

  // Action Servers
  rclcpp_action::Server<Takeoff>::SharedPtr takeoff_action_server_; 
  rclcpp_action::GoalResponse takeoff_handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Takeoff::Goal> goal);
  rclcpp_action::CancelResponse takeoff_handle_cancel(
    const std::shared_ptr<GoalHandleTakeoff> goal_handle);
  void takeoff_handle_accepted(const std::shared_ptr<GoalHandleTakeoff> goal_handle);
  void takeoff_execute(const std::shared_ptr<GoalHandleTakeoff> goal_handle);

  rclcpp_action::Server<Land>::SharedPtr land_action_server_; 
  rclcpp_action::GoalResponse land_handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Land::Goal> goal);
  rclcpp_action::CancelResponse land_handle_cancel(
    const std::shared_ptr<GoalHandleLand> goal_handle);
  void land_handle_accepted(const std::shared_ptr<GoalHandleLand> goal_handle);
  void land_execute(const std::shared_ptr<GoalHandleLand> goal_handle);


  // Services
  rclcpp::Service<drone_interfaces::srv::Arm>::SharedPtr arm_service_;
  void arm(const std::shared_ptr<drone_interfaces::srv::Arm::Request> request,
               std::shared_ptr<drone_interfaces::srv::Arm::Response> response);

  rclcpp::Service<drone_interfaces::srv::Offboard>::SharedPtr offboard_service_;
  void offboard(const std::shared_ptr<drone_interfaces::srv::Offboard::Request> request,
               std::shared_ptr<drone_interfaces::srv::Offboard::Response> response);             

};  // class DroneNode

// ROS Subscriptions //////////////////////////////////////////////////////////////////
void DroneNode::cmd_vel_topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
{
  // Offboard_control_mode needs to be paired with trajectory_setpoint
  
  // Enable Offboard Mode for velocity control
  OffboardControlMode px4_obc_msg{};
  px4_obc_msg.timestamp = timestamp_.load();
  px4_obc_msg.position = false;
  px4_obc_msg.velocity = true;
  px4_obc_msg.acceleration = false;
  px4_obc_msg.attitude = false;
  px4_obc_msg.body_rate = false;

  offboard_control_mode_publisher_->publish(px4_obc_msg);
  
  // Receive meaages in ROS base_link FLU ; X->Foreward, Y->Left Z->Up
  // Send to PX4 in FRD : X->Foreward, Y->Rightl Z->Down.
  
  TrajectorySetpoint px4_tsp_msg{};
  px4_tsp_msg.timestamp = timestamp_.load();
  px4_tsp_msg.x = msg->linear.x;
  px4_tsp_msg.y = -(msg->linear.y);
  px4_tsp_msg.z = -(msg->linear.z);
  px4_tsp_msg.yawspeed = -(msg->angular.z);  // CCW Yaw in ros is positive; CCW Yaw in PX4 is negative.
                                     // YawSpeed is set in radians per second.

  trajectory_setpoint_publisher_->publish(px4_tsp_msg);
}

// Action Servers /////////////////////////////////////////////////////////////////
// Takeoff Action Server - Start
  rclcpp_action::GoalResponse DroneNode::takeoff_handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Takeoff::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with target altitude %d", goal->target_altitude);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse DroneNode::takeoff_handle_cancel(
    const std::shared_ptr<GoalHandleTakeoff> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void DroneNode::takeoff_handle_accepted(const std::shared_ptr<GoalHandleTakeoff> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&DroneNode::takeoff_execute, this, _1), goal_handle}.detach();
  }

  void DroneNode::takeoff_execute(const std::shared_ptr<GoalHandleTakeoff> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Takeoff::Feedback>();
    auto &current_altitude = feedback->current_altitude;
    auto result = std::make_shared<Takeoff::Result>();

    // Initialise the feedback.  (For takeoff this should be 0.0
    current_altitude = last_altitude;  
    
    // Offboard_control_mode needs to be paired with trajectory_setpoint  
    // Enable Offboard Mode for position and altitude control
    OffboardControlMode msg{};
    msg.timestamp = timestamp_.load();
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;

    offboard_control_mode_publisher_->publish(msg);
  
    // Receive meaages in ROS base_link FLU ; X->Foreward, Y->Left Z->Up
    // Send to PX4 in FRD : X->Foreward, Y->Rightl Z->Down.
  
    TrajectorySetpoint px4_tsp_msg{};
    px4_tsp_msg.timestamp = timestamp_.load();
    px4_tsp_msg.x = 0.0;
    px4_tsp_msg.y = 0.0;
    px4_tsp_msg.z = -((float)goal->target_altitude);
    px4_tsp_msg.yaw = 0.0; 
        
    trajectory_setpoint_publisher_->publish(px4_tsp_msg);

    // TODO Put a timeout on this one
    while (last_altitude < goal->target_altitude) {
      
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->result = true;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      
      // Update feedback (obtined from the PX4 odomitry messages - see px4_odom_callback() )
      current_altitude = last_altitude;

      // Publish feedback
      goal_handle->publish_feedback(feedback);

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->result = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }

// Takeoff Action Server End
// Land Action Server Start

  rclcpp_action::GoalResponse DroneNode::land_handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Land::Goal> goal)
  {
    if (goal->gear_down) {
      // Set the gear down first
      // Not implimented yet
    };
    
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse DroneNode::land_handle_cancel(
    const std::shared_ptr<GoalHandleLand> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel landing");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
    // TODO:: Do we place the drone back in Offboard Mode??
  }

  void DroneNode::land_handle_accepted(const std::shared_ptr<GoalHandleLand> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&DroneNode::land_execute, this, _1), goal_handle}.detach();
  }

  void DroneNode::land_execute(const std::shared_ptr<GoalHandleLand> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing landing goal");
    
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Land::Feedback>();
    auto &current_altitude = feedback->current_altitude;
    
    auto result = std::make_shared<Land::Result>();

    if (goal->gear_down) {
      // Set the gear down first
      // Not implimented yet
    };
    
    // uint16 VEHICLE_CMD_NAV_LAND = 21			# Land at location |Empty| Empty| Empty| Desired yaw angle.| Latitude| Longitude| Altitude|
    // set MAV_CMD_DO_SET_MODE
    // as defined in px4_msgs/msg/CommanderState.msg
    // MAIN_STATE_AUTO_LAND          = 11
    this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 11, 0);

    // Update feedback (obtined from the PX4 odomitry messages - see px4_odom_callback() )
    current_altitude = last_altitude;
    // TODO What about monitoring a timeout
    bool landed_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_sub_;
    land_sub_ =
			this->create_subscription<px4_msgs::msg::VehicleLandDetected>("fmu/timesync/out", 10,
				[&](const px4_msgs::msg::VehicleLandDetected::UniquePtr msg) {
          landed_ = msg->landed;
				});
        
    while(!landed_ && rclcpp::ok()) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->result = true;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // Update feedback (obtined from the PX4 odomitry messages - see px4_odom_callback() )
      current_altitude = last_altitude;
      goal_handle->publish_feedback(feedback);

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->result = landed_;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }

// Land Action Server End

// Services /////////////////////////////////////////////////////////////////////

void DroneNode::arm(const std::shared_ptr<drone_interfaces::srv::Arm::Request> request,
               std::shared_ptr<drone_interfaces::srv::Arm::Response> response) {

  // CONFUSED Why does the example pubish Parameter 1 = 1 and Parameter 2 = 1
  // MAV_CMD states in px4_msgs/msg/VehicleCommand.msg
  // uint16 VEHICLE_CMD_DO_SET_MODE = 176			# Set system mode. |Mode, as defined by ENUM MAV_MODE| Empty| Empty| Empty| Empty| Empty| Empty|
  // and in px4_msgs/msg/VehicleCommand.msg
  // MAIN_STATE_OFFBOARD           = 7
  // I say this is wrong  
  // this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
  this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 7, 0);
  
  // Arm the vehecle
  this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, (float)request->arm);

  response->result = true;
}

void DroneNode::offboard(const std::shared_ptr<drone_interfaces::srv::Offboard::Request> request,
               std::shared_ptr<drone_interfaces::srv::Offboard::Response> response) {

    if (request->enable == 0) {
      // Disable Offboard Mode.
      // EXPERIMENTAL. Will this diable offboard control?
        OffboardControlMode msg{};
        msg.timestamp = timestamp_.load();
        msg.position = false;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;

        offboard_control_mode_publisher_->publish(msg);
    } else {
      // Enable Offboard Mode for velocity control
        OffboardControlMode msg{};
        msg.timestamp = timestamp_.load();
        msg.position = false;
        msg.velocity = true;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;

        offboard_control_mode_publisher_->publish(msg);

    }

  response->result = true;
}

  
// PX4 Subscribers //////////////////////////////////////////////////////////////////////
//void DroneNode::px4_odom_callback(px4_msgs::msg::VehicleOdometry px4_msg)
void DroneNode::px4_odom_callback(const px4_msgs::msg::VehicleOdometry::UniquePtr px4_msg)  
  {
    auto message = nav_msgs::msg::Odometry();
    
      message.header.stamp = rclcpp::Node::now(); //timestamp_.load(); 
      message.header.frame_id ="odom";

      message.pose.pose.position.x = px4_msg->x;
      message.pose.pose.position.y = px4_msg->y;
      message.pose.pose.position.z = px4_msg->z;
    
    last_altitude = -(px4_msg->z);  // Save for feedback on takoff action server
      
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
  
  /**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 * **/

void DroneNode::publish_vehicle_command(uint16_t command, float param1,
                                              float param2) const {
        VehicleCommand msg{};
        msg.timestamp = timestamp_.load();
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;

        vehicle_command_publisher_->publish(msg);
}
  
}  // namespace drone_node

RCLCPP_COMPONENTS_REGISTER_NODE(drone_node::DroneNode)

