#ifndef ODOM_BROADCASTER_NODE_HPP_
#define ODOM_BROADCASTER_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

//#include <nav_msgs/msg/odometry.hpp>

//#include <geometry_msgs/msg/twist.hpp>
//#include <geometry_msgs/msg/transform_stamped.hpp>

#include "drone_rtps/visibility_control.h"

class OdomBroadcasterNode : public rclcpp::Node
{
public:
  DRONE_PUBLIC OdomBroadcasterNode(rclcpp::NodeOptions options);

private:
  void on_timer();

  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

#endif  // ODOM_BROADCASTER_NODE_HPP_
