#ifndef PERSON_TRACKER__PERSONTRACKERNODE_HPP_
#define PERSON_TRACKER__PERSONTRACKERNODE_HPP_

#include <random>

#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "rclcpp/rclcpp.hpp"
#include "person_tracker_interfaces/msg/entity.hpp"
#include "person_tracker_interfaces/msg/detected_entities.hpp"

namespace person_tracker
{

class PersonTrackerNode : public rclcpp::Node
{
public:
  PersonTrackerNode();

private:
  void generate_tf();
  void publish_tf();

  rclcpp::TimerBase::SharedPtr timer_generate_;
  rclcpp::TimerBase::SharedPtr timer_publish_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::TransformStamped transform_;

  rclcpp::Subscription<person_tracker_interfaces::msg::DetectedEntities>::SharedPtr entities_subscriber_;
};

}  //  namespace person_tracker

#endif  // PERSON_TRACKER__PERSONTRACKERNODE_HPP_
