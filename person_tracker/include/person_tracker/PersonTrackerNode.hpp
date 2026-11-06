#ifndef PERSON_TRACKER__PERSONTRACKERNODE_HPP_
#define PERSON_TRACKER__PERSONTRACKERNODE_HPP_

#include <random>

#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "rclcpp/rclcpp.hpp"
#include "person_tracker_interfaces/msg/entity.hpp"
#include "person_tracker_interfaces/msg/detected_entities.hpp"

#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/subscriber.h"
#include "vision_msgs/msg/detection3_d_array.hpp"

namespace person_tracker
{

class PersonTrackerNode : public rclcpp::Node
{
public:
  PersonTrackerNode();

private:
  void callback_sync(
    const person_tracker_interfaces::msg::DetectedEntities::ConstSharedPtr & lidar_detections,
    const vision_msgs::msg::Detection3DArray::ConstSharedPtr & yolo_detections);

  typedef message_filters::sync_policies::ApproximateTime<
      person_tracker_interfaces::msg::DetectedEntities, vision_msgs::msg::Detection3DArray> MySyncPolicy;

  std::shared_ptr<message_filters::Subscriber<person_tracker_interfaces::msg::DetectedEntities>> entities_subscriber_;
  std::shared_ptr<message_filters::Subscriber<vision_msgs::msg::Detection3DArray>> yolo_detections3d_subscriber_;
  std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
};

}  //  namespace person_tracker

#endif  // PERSON_TRACKER__PERSONTRACKERNODE_HPP_
