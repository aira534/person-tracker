#include "rclcpp/rclcpp.hpp"

#include "person_tracker/PersonTrackerNode.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<person_tracker::PersonTrackerNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
