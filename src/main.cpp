#include <rclcpp/rclcpp.hpp>
#include "px4_bridge.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_bridge::Px4Bridge>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
