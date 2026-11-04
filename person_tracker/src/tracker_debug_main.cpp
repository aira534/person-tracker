#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "person_tracker/TrackerDebugNode.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<person_tracker::TrackerDebugNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
