#ifndef PERSONTRACKER__DETECTIONTO3DFROMDEPTHNODE_HPP_
#define PERSONTRACKER__DETECTIONTO3DFROMDEPTHNODE_HPP_

#include <memory>

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "image_geometry/pinhole_camera_model.hpp"
#include "yolo_msgs/msg/detection_array.hpp"

#include "rclcpp/rclcpp.hpp"

// for tf
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace person_tracker
{

class DetectionTo3DfromDepthNode : public rclcpp::Node
{
public:
  DetectionTo3DfromDepthNode();

  void to_odom(vision_msgs::msg::Detection3DArray & detections_3d);

private:
  void callback_sync(
    const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
    const yolo_msgs::msg::DetectionArray::ConstSharedPtr & detection_msg);
  void callback_info(sensor_msgs::msg::CameraInfo::UniquePtr msg);

  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image, yolo_msgs::msg::DetectionArray> MySyncPolicy;

  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;
  std::shared_ptr<message_filters::Subscriber<yolo_msgs::msg::DetectionArray>> detection_sub_;
  std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  std::shared_ptr<image_geometry::PinholeCameraModel> model_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr detection_pub_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr detection_odom_pub_;
  vision_msgs::msg::Detection3DArray detections_3d_odom_;

  std::string fixed_frame_;

  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace person_tracker

#endif  // PERSONTRACKER__DETECTIONTO3DFROMDEPTHNODE_HPP_