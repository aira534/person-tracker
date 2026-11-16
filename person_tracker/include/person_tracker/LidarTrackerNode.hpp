#ifndef PERSON_TRACKER__LIDARTRACKERNODE_HPP_
#define PERSON_TRACKER__LIDARTRACKERNODE_HPP_

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "person_tracker_interfaces/msg/detected_entities.hpp"
#include "person_tracker_interfaces/msg/entity.hpp"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/transform_datatypes.h"
#include <vector>
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "rclcpp/rclcpp.hpp"

namespace person_tracker
{

class LidarTrackerNode : public rclcpp::Node
{
public:
  LidarTrackerNode();
  void centers_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  void updateEntities(const rclcpp::Time & stamp);
  void entities_callback(person_tracker_interfaces::msg::DetectedEntities::SharedPtr entities);

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr centers_subscriber_;
  rclcpp::Publisher<person_tracker_interfaces::msg::DetectedEntities>::SharedPtr assoc_entities_pub_;
  std::vector<geometry_msgs::msg::Point> points_odom_;
  rclcpp::Subscription<person_tracker_interfaces::msg::DetectedEntities>::SharedPtr entities_sub_;

  person_tracker_interfaces::msg::DetectedEntities entities_;

  int max_misses_;

  void publishEntities(const rclcpp::Time & stamp);
};

}  //  namespace person_tracker

#endif  // PERSON_TRACKER__LIDARTRACKERNODE_HPP_
