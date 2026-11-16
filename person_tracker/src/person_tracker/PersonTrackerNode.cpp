#include "person_tracker/PersonTrackerNode.hpp"

namespace person_tracker {

using std::placeholders::_1;
using std::placeholders::_2;

PersonTrackerNode::PersonTrackerNode()
: Node("sensor_fusion") {
  entities_pub_ = create_publisher<person_tracker_interfaces::msg::DetectedEntities>("/tracked_entities", 10);
  yolo_detections3d_subscriber_ = create_subscription<vision_msgs::msg::Detection3DArray>(
      "/lidar_objects", 10,
      std::bind(&PersonTrackerNode::people_callback, this, _1));
}

void PersonTrackerNode::people_callback(vision_msgs::msg::Detection3DArray::SharedPtr people) {
  void();
}

}
