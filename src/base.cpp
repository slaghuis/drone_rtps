#include <chrono>
#include <cmath>         //M_PI and M_PI_2
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include <nav_msgs/msg/odometry.hpp>

#include <px4_msgs/msg/vehicle_gps_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/commander_state.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>

#include "tf2_ros/buffer.h"
#include "tf2_ros/buffer_interface.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using std::placeholders::_1;
using namespace std::chrono;
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class BaseController : public rclcpp::Node
{

  public:
    BaseController()
    : Node("base_controller"), count_(0)
    {
    
      offboard_control_mode_publisher_ =
        this->create_publisher<px4_msgs::msg::OffboardControlMode>("OffboardControlMode_PubSubTopic", 10);
      trajectory_setpoint_publisher_ =
        this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("TrajectorySetpoint_PubSubTopic", 10);
      vehicle_command_publisher_ =
        this->create_publisher<px4_msgs::msg::VehicleCommand>("VehicleCommand_PubSubTopic", 10);

      // get common timestamp
      timesync_sub_ =
        this->create_subscription<px4_msgs::msg::Timesync>("Timesync_PubSubTopic", 10,
          [this](const px4_msgs::msg::Timesync::UniquePtr msg) {
          timestamp_.store(msg->timestamp);});
          
      px4_gps_sub_ = this->create_subscription<px4_msgs::msg::VehicleGpsPosition>(
      "VehicleGpsPosition_PubSubTopic", 10, std::bind(&BaseController::px4_gps_callback, this, _1));

      px4_odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
      "VehicleOdometry_PubSubTopic", 10, std::bind(&BaseController::px4_odometry_callback, this, _1));
      
      px4_state_sub_ = this->create_subscription<px4_msgs::msg::CommanderState>(
      "CommanderState_PubSubTopic", 10, std::bind(&BaseController::px4_state_callback, this, _1));
        
      cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/pixhawk1/cmd_vel", 10, std::bind(&BaseController::cmd_vel_callback, this, _1));
    
      odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("pixhawk1/odometry", 10);
      gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("pixhawk1/gps", 10);

      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

      clock=std::make_shared< rclcpp::Clock >(RCL_SYSTEM_TIME); 

      tf_buffer.reset(new tf2_ros::Buffer(clock)); 
      tf_listner.reset(new tf2_ros::TransformListener(*tf_buffer)); 
      
      //Temporary code to get the bird in the sky.  Use this to test the TF2 tranformation and all the topics published.
      
      offboard_setpoint_counter_ = 0;

      auto timer_callback = [this]() -> void {

			if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				// Arm the vehicle
				this->arm();
			}

            		// offboard_control_mode needs to be paired with trajectory_setpoint
			publish_offboard_control_mode();
			publish_trajectory_setpoint();

           		 // stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
			}
		};
		timer_ = this->create_wall_timer(100ms, timer_callback);

            
    }
    
    void arm() const;
    void disarm() const;

  private:
  
    void px4_gps_callback(const px4_msgs::msg::VehicleGpsPosition::SharedPtr msg)
    {
	// Read data received from PX4 VehicleGpsPosition_PubSubTopic and publish
	
	auto message = sensor_msgs::msg::NavSatFix();
      
        message.header.stamp = rclcpp::Node::now(); //timestamp_.load();
        message.header.frame_id ="world";
      
        message.latitude = (float)(msg->lat) * 0.00000001;
        message.longitude = (float)(msg->lon) * 1e-7;
        message.altitude = (float)msg->alt_ellipsoid * 0.0001;
      
        message.status.status = 0;   //unaugmented fix
        message.status.service = 1;  //gps
      
        message.position_covariance_type = 0; //Unknown;

        gps_pub_->publish(message);
 			
    }
    rclcpp::Subscription<px4_msgs::msg::VehicleGpsPosition>::SharedPtr px4_gps_sub_;
    
    void px4_odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
      
      /* ***
       * Publish a tf2 transformation.
       * ***/
            
//      if (msg->local_frame != msg->velocity_frame) {
//                RCLCPP_WARN(this->get_logger(), "PX4 local_frame and velocity_frame differ.  What now?");
//      }      
            
      /**
       * @brief Static quaternion needed for rotating between ENU and NED frames
       * NED to ENU: +PI/2 rotation about Z (Down) followed by a +PI rotation around X (old North/new East)
       * ENU to NED: +PI/2 rotation about Z (Up) followed by a +PI rotation about X (old East/new North)
       */
      // static const auto NED_ENU_Q = quaternion_from_rpy(M_PI, 0.0, M_PI_2);
      // Publish the transform over tf
      // PX4 uses FRD (x=Front, y=Right, z=Down)
      // ROS uses FLU (x=front, y=Left, z=Up)
      // This implies we should roll over 180 degrees, or PI radians.
      geometry_msgs::msg::TransformStamped transformStamped;

      transformStamped.header.stamp = rclcpp::Node::now(); //timestamp_.load();
      transformStamped.header.frame_id = "odom";
      transformStamped.child_frame_id = "base_link";
      transformStamped.transform.translation.x = 0.0;
      transformStamped.transform.translation.y = 0.0;
      transformStamped.transform.translation.z = 0.0;
      
      tf2::Quaternion quat;
      if (msg->local_frame == msg->LOCAL_FRAME_NED) {
        /**
         * @brief Static quaternion needed for rotating between ENU and NED frames
         * NED to ENU: +PI/2 rotation about Z (Down) followed by a +PI rotation around X (old North/new East)
         * ENU to NED: +PI/2 rotation about Z (Up) followed by a +PI rotation about X (old East/new North)
         */
        //static const auto NED_ENU_Q = quaternion_from_rpy(M_PI, 0.0, M_PI_2);
        quat.setRPY(M_PI, 0.0, M_PI_2);        
      } 
      else if ((msg->local_frame == msg->LOCAL_FRAME_FRD)  || (msg->local_frame == msg->BODY_FRAME_FRD)){
        /**
         * @brief Static quaternion needed for rotating between aircraft and base_link frames
         * +PI rotation around X (Forward) axis transforms from Forward, Right, Down (aircraft)
         * Fto Forward, Left, Up (base_link) frames.
         */
        //  static const auto AIRCRAFT_BASELINK_Q = quaternion_from_rpy(M_PI, 0.0, 0.0);
        quat.setRPY(M_PI, 0.0, 0.0);
      } else {
        RCLCPP_WARN(this->get_logger(), "PIXHAWK position frame not aligned with the std frames of referenceodometry. ");
        quat.setRPY(0.0, 0.0, 0.0);
      }
      
      transformStamped.transform.rotation.x = quat.x();
      transformStamped.transform.rotation.y = quat.y();
      transformStamped.transform.rotation.z = quat.z();
      transformStamped.transform.rotation.w = quat.w(); 
      tf_broadcaster_->sendTransform(transformStamped);
              
      
      /*** 
       * Push this data into a ROS Odometry message and publish
       * ***/
       
      auto message = nav_msgs::msg::Odometry();

      message.header.stamp = rclcpp::Node::now();//timestamp_.load();
      message.header.frame_id ="odom";

      message.pose.pose.position.x = msg->x;
      message.pose.pose.position.y = msg->y;
      message.pose.pose.position.z = msg->z;
      
      message.pose.pose.orientation.x = msg->q[0];
      message.pose.pose.orientation.y = msg->q[1]; 
      message.pose.pose.orientation.z = msg->q[2];
      message.pose.pose.orientation.w = msg->q[3]; 
        
      message.pose.covariance[0]=msg->pose_covariance[msg->COVARIANCE_MATRIX_X_VARIANCE];
      message.pose.covariance[7]=msg->pose_covariance[msg->COVARIANCE_MATRIX_Y_VARIANCE];
      message.pose.covariance[14]=msg->pose_covariance[msg->COVARIANCE_MATRIX_Z_VARIANCE];      
      message.pose.covariance[21]=msg->pose_covariance[msg->COVARIANCE_MATRIX_ROLL_VARIANCE];
      message.pose.covariance[28]=msg->pose_covariance[msg->COVARIANCE_MATRIX_PITCH_VARIANCE];
      message.pose.covariance[35]=msg->pose_covariance[msg->COVARIANCE_MATRIX_YAW_VARIANCE];
        
      message.twist.twist.linear.x = msg->vx;
      message.twist.twist.linear.y = msg->vy;
      message.twist.twist.linear.z = msg->vz;
      
      message.twist.twist.angular.x = msg->pitchspeed;
      message.twist.twist.angular.y = msg->rollspeed;
      message.twist.twist.angular.z = msg->yawspeed;

      message.twist.covariance[0]=msg->velocity_covariance[msg->COVARIANCE_MATRIX_VX_VARIANCE];
      message.twist.covariance[7]=msg->velocity_covariance[msg->COVARIANCE_MATRIX_VY_VARIANCE];
      message.twist.covariance[14]=msg->velocity_covariance[msg->COVARIANCE_MATRIX_VZ_VARIANCE];      
      message.twist.covariance[21]=msg->velocity_covariance[msg->COVARIANCE_MATRIX_ROLLRATE_VARIANCE];
      message.twist.covariance[28]=msg->velocity_covariance[msg->COVARIANCE_MATRIX_PITCHRATE_VARIANCE];
      message.twist.covariance[35]=msg->velocity_covariance[msg->COVARIANCE_MATRIX_YAWRATE_VARIANCE];
      
      // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      odom_pub_->publish(message);

    }
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_odometry_sub_;


    void px4_state_callback(const px4_msgs::msg::CommanderState::SharedPtr msg)
    {
	// Read data received from PX4 CommanderState_PubSubTopic and publish
	
	/*
	auto message = sensor_msgs::msg::NavSatFix();
           
        now = rclcpp::Node::now();
      
        message.header.stamp = now;//   clock->now();
        message.header.frame_id ="world";
      
        message.latitude = (float)(msg->lat) * 0.00000001;
        message.longitude = (float)(msg->lon) * 1e-7;
        message.altitude = (float)msg->alt_ellipsoid * 0.0001;
      
        message.status.status = 0;   //unaugmented fix
        message.status.service = 1;  //gps
      
        message.position_covariance_type = 0; //Unknown;

        gps_pub_->publish(message);
        */
        
        main_state = msg->main_state;
        MAIN_STATE_OFFBOARD = msg->MAIN_STATE_OFFBOARD;
        		
    }
    rclcpp::Subscription<px4_msgs::msg::CommanderState>::SharedPtr px4_state_sub_;


    
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
    {
      // Message received from ROS,
      // Push this message through to PX4
      RCLCPP_INFO(this->get_logger(), "I heard a twist %f", msg->linear.x);
       
      if (main_state != MAIN_STATE_OFFBOARD) {
        RCLCPP_INFO(this->get_logger(), "Not in OFFBOARD mode.  Message ignored.");
        return;
      }
       
      try {
      
        // Read the transform from base_link to odom        
        
        geometry_msgs::msg::TransformStamped base_link_to_odom;   // A Tranformation listener to translate 'base_link' to 'odom'        
        base_link_to_odom = tf_buffer->lookupTransform("odom", "base_link", this->now());
                
        px4_msgs::msg::TrajectorySetpoint message{};
        message.timestamp = timestamp_.load();
        message.x = base_link_to_odom.transform.translation.x; // msg->linear.x;
        message.y = base_link_to_odom.transform.translation.y; // msg->linear.y;
        message.z = base_link_to_odom.transform.translation.z; // msg->linear.z;
        
        tf2::Quaternion q(
          base_link_to_odom.transform.rotation.x,
          base_link_to_odom.transform.rotation.y,
          base_link_to_odom.transform.rotation.z,
          base_link_to_odom.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        message.yaw = yaw; //msg->angular.z; // [-PI:PI]

        trajectory_setpoint_publisher_->publish(message);

      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "%s", ex.what());
        //  rclcpp::sleep_for(1);
      }
        
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
    
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;

    std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

    uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

    void publish_offboard_control_mode() const;
    void publish_trajectory_setpoint() const;
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0) const;

    size_t count_;
    uint8_t main_state = 0;
    uint8_t MAIN_STATE_OFFBOARD = 7;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer; 
    std::shared_ptr<tf2_ros::TransformListener> tf_listner; 
        
    rclcpp::Clock::SharedPtr clock; 
};


/**
 * @brief Send a command to Arm the vehicle
 */
void BaseController::arm() const {
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void BaseController::disarm() const {
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void BaseController::publish_offboard_control_mode() const {
	px4_msgs::msg::OffboardControlMode msg{};
	msg.timestamp = timestamp_.load();
	msg.position = true;
	msg.velocity = false;
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
void BaseController::publish_trajectory_setpoint() const {
	px4_msgs::msg::TrajectorySetpoint msg{};
	msg.timestamp = timestamp_.load();
	msg.x = 0.0;
	msg.y = 0.0;
	msg.z = -5.0;
	msg.yaw = -3.14; // [-PI:PI]

	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void BaseController::publish_vehicle_command(uint16_t command, float param1,
					      float param2) const {
	px4_msgs::msg::VehicleCommand msg{};
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


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BaseController>());
  rclcpp::shutdown();
  return 0;
}
