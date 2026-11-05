#ifndef PERSON_TRACKER__TRACKER_DEBUG_NODE_HPP_
#define PERSON_TRACKER__TRACKER_DEBUG_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include "visualization_msgs/msg/marker_array.hpp"
#include "person_tracker_interfaces/msg/detected_entities.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <cstdint>
#include <string>
#include "vision_msgs/msg/detection3_d_array.hpp"

namespace person_tracker
{

class TrackerDebugNode : public rclcpp::Node
{
public:
  TrackerDebugNode();
  void camera_detections_callback(vision_msgs::msg::Detection3DArray::ConstSharedPtr msg);

private:
  void entitiesCallback(const person_tracker_interfaces::msg::DetectedEntities::SharedPtr msg);

  rclcpp::Subscription<person_tracker_interfaces::msg::DetectedEntities>::SharedPtr entities_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr camera_detections_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr camera_markers_pub_;
};

}  // namespace person_tracker

#endif  // PERSON_TRACKER__TRACKER_DEBUG_NODE_HPP_