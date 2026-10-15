#include "rclcpp/rclcpp.hpp"

#include "person_tracker/LidarTrackerNode.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<person_tracker::LidarTrackerNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
