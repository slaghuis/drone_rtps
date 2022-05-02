#include <memory>
#include "drone_rtps/odom_broadcaster_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomBroadcasterNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}