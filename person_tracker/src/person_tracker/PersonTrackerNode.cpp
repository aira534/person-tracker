#include "person_tracker/PersonTrackerNode.hpp"

namespace person_tracker {

using std::placeholders::_1;
using std::placeholders::_2;

PersonTrackerNode::PersonTrackerNode()
: Node("sensor_fusion") {
  entities_subscriber_ = std::make_shared<message_filters::Subscriber<person_tracker_interfaces::msg::DetectedEntities>>(
    this, "/tracked_entities", rclcpp::SensorDataQoS().reliable().get_rmw_qos_profile());
  yolo_detections3d_subscriber_ =
    std::make_shared<message_filters::Subscriber<vision_msgs::msg::Detection3DArray>>(
    this, "/detections_3d_odom", rclcpp::SensorDataQoS().reliable().get_rmw_qos_profile());
  sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(
    MySyncPolicy(10), *entities_subscriber_, *yolo_detections3d_subscriber_);
  sync_->registerCallback(std::bind(&PersonTrackerNode::callback_sync, this, _1, _2));
}

void PersonTrackerNode::callback_sync(
  const person_tracker_interfaces::msg::DetectedEntities::ConstSharedPtr & lidar_detections,
  const vision_msgs::msg::Detection3DArray::ConstSharedPtr & yolo_detections)
{
  void();
}

}
