#include <memory>

#include "person_tracker/DetectionTo3DfromDepthNode.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node_converter = std::make_shared<person_tracker::DetectionTo3DfromDepthNode>();

  rclcpp::spin(node_converter);

  rclcpp::shutdown();
  return 0;
}