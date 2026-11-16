#ifndef PERSON_TRACKER__PERSONTRACKERNODE_HPP_
#define PERSON_TRACKER__PERSONTRACKERNODE_HPP_

#include <random>

#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "rclcpp/rclcpp.hpp"
#include "person_tracker_interfaces/msg/entity.hpp"
#include "person_tracker_interfaces/msg/detected_entities.hpp"

#include "vision_msgs/msg/detection3_d_array.hpp"

namespace person_tracker
{

class PersonTrackerNode : public rclcpp::Node
{
public:
  PersonTrackerNode();

  void people_callback(vision_msgs::msg::Detection3DArray::SharedPtr people);

private:
  rclcpp::Publisher<person_tracker_interfaces::msg::DetectedEntities>::SharedPtr entities_pub_;
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr yolo_detections3d_subscriber_;
};

}  //  namespace person_tracker

#endif  // PERSON_TRACKER__PERSONTRACKERNODE_HPP_
