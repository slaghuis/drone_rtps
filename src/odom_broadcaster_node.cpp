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
 * Transform listeneer for base_link_ned -> odom transforms and publishes
 * a tranform odom->base_link.
 * This thing should have been called the base_link broadcaster :-(
 *
 *  See: https://answers.ros.org/question/248783/do-the-odom-topic-and-odom-base_link-tf-provide-duplicate-information/
 * ***********************************************************************/

#include "drone_rtps/odom_broadcaster_node.hpp"

using namespace std::chrono_literals;

OdomBroadcasterNode::OdomBroadcasterNode(rclcpp::NodeOptions options)
: Node("odom_broadcaster_node", options)
{
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize the transform broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  
  timer_ = create_wall_timer(
    250ms, std::bind(&OdomBroadcasterNode::on_timer, this));
}

void OdomBroadcasterNode::on_timer()
{
  // Lookup the transform from base_link_ned to odom
  // Publish a odom->base_link transform
  std::string fromFrameRel = "base_link_ned";
  std::string toFrameRel = "odom";
    
  geometry_msgs::msg::TransformStamped transformStamped;
    
  try {
    transformStamped = tf_buffer_->lookupTransform(
      toFrameRel, fromFrameRel,
      tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_DEBUG(
      this->get_logger(), "Could not transform %s to %s: %s",
        toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
      return;
  }
    
  RCLCPP_DEBUG(this->get_logger(), "Transformed from '%s' frame to '%s' frame: %0.1f, %0.1f, %0.1f", 
      fromFrameRel.c_str(),
      toFrameRel.c_str(),
      transformStamped.transform.translation.x, 
      transformStamped.transform.translation.y, 
      transformStamped.transform.translation.z);
    
  transformStamped.header.frame_id = "odom";
  transformStamped.child_frame_id = "base_link";
    
  // Send the transformation
  tf_broadcaster_->sendTransform(transformStamped);
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(OdomBroadcasterNode)