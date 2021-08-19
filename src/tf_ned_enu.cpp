/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_gps_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "tf2_ros/buffer.h"

#define M_PI 3.14159265358979323846

int main(int argc, char ** argv)
{
  // Initialize ROS
  std::vector<std::string> args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  rclcpp::NodeOptions options;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> node;

  // If I yaw -90degrees and roll 180 degrees I am there.
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;   // Down becomes up.
  double rx, ry, rz, rw;
  std::string frame_id, child_id;

  // grab parameters from yaw, pitch, roll
  tf2::Quaternion quat;
  quat.setRPY(-M_PI/2.0, 0.0, M_PI);
  rx = quat.x();
  ry = quat.y();
  rz = quat.z();
  rw = quat.w();
  frame_id = "odom_ned";
  child_id = "base_frame";
  
  // override default parameters with the desired transform
  options.parameter_overrides(
  {
    {"translation.x", x},
    {"translation.y", y},
    {"translation.z", z},
    {"rotation.x", rx},
    {"rotation.y", ry},
    {"rotation.z", rz},
    {"rotation.w", rw},
    {"frame_id", frame_id},
    {"child_frame_id", child_id},
  });

  node = std::make_shared<tf2_ros::StaticTransformBroadcaster>(options);

//  RCLCPP_INFO(
//    node->get_logger(), "Spinning until killed publishing transform from '%s' to '%s'",
//    frame_id.c_str(), child_id.c_str());
  rclcpp::spin(node);
  return 0;
}
