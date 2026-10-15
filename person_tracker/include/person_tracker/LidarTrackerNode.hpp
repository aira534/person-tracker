#ifndef PERSON_TRACKER__LIDARTRACKERNODE_HPP_
#define PERSON_TRACKER__LIDARTRACKERNODE_HPP_

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "person_tracker_interfaces/msg/detected_entities.hpp"

#include "rclcpp/rclcpp.hpp"

namespace person_tracker
{

class LidarTrackerNode : public rclcpp::Node
{
public:
  LidarTrackerNode();
  void centers_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr centers_subscriber_;
  rclcpp::Publisher<person_tracker_interfaces::msg::DetectedEntities>::SharedPtr entities_pub_;
};

}  //  namespace person_tracker

#endif  // PERSON_TRACKER__LIDARTRACKERNODE_HPP_
