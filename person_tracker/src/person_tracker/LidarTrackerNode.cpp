#include "person_tracker/LidarTrackerNode.hpp"

using std::placeholders::_1;

namespace person_tracker {

LidarTrackerNode::LidarTrackerNode()
: Node("detections_tracking_node") {
  centers_subscriber_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/lidar_objects", 10,
      std::bind(&LidarTrackerNode::centers_callback, this, _1));

  entities_pub_ = create_publisher<person_tracker_interfaces::msg::DetectedEntities>("/entities_info", 10);
}

void
LidarTrackerNode::centers_callback(geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  void();
}
}
