// desarrollado a partir de código obtenido e impartido en clase de Arquitecturas Software para Robots por Francisco Martín Rico
// en la Universidad Rey Juan Carlos. Se han adaptado algunos topics y tipos de mensajes para hacer la codificación más eficiente
// pero se mantiene la estructura y la lógica primordialmente

#include <memory>

#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "person_tracker/DetectionTo3DfromDepthNode.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "image_geometry/pinhole_camera_model.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "depth_image_proc/depth_traits.hpp"

#include "rclcpp/rclcpp.hpp"

namespace person_tracker
{

using std::placeholders::_1;
using std::placeholders::_2;

DetectionTo3DfromDepthNode::DetectionTo3DfromDepthNode()
: Node("detection_to_3d_from_depth_node"),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
    this, "input_depth", rclcpp::SensorDataQoS().reliable().get_rmw_qos_profile());
  detection_sub_ =
    std::make_shared<message_filters::Subscriber<yolo_msgs::msg::DetectionArray>>(
    this, "input_detection_2d", rclcpp::SensorDataQoS().reliable().get_rmw_qos_profile());
  sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(
    MySyncPolicy(10), *depth_sub_, *detection_sub_);
  sync_->registerCallback(std::bind(&DetectionTo3DfromDepthNode::callback_sync, this, _1, _2));

  info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    "camera_info", 1, std::bind(&DetectionTo3DfromDepthNode::callback_info, this, _1));
  detection_pub_ = create_publisher<vision_msgs::msg::Detection3DArray>(
    "output_detection_3d", rclcpp::SensorDataQoS().reliable());

  detection_odom_pub_ = create_publisher<vision_msgs::msg::Detection3DArray>(
    "/detections_3d_odom", rclcpp::SensorDataQoS().reliable());

  this->declare_parameter<std::string>("fixed_frame", "odom");
  fixed_frame_ = this->get_parameter("fixed_frame").as_string();
}

void
DetectionTo3DfromDepthNode::callback_info(sensor_msgs::msg::CameraInfo::UniquePtr msg)
{
  RCLCPP_INFO(get_logger(), "Camera info received");

  model_ = std::make_shared<image_geometry::PinholeCameraModel>();
  model_->fromCameraInfo(*msg);

  info_sub_ = nullptr;
}

void
DetectionTo3DfromDepthNode::callback_sync(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
  const yolo_msgs::msg::DetectionArray::ConstSharedPtr & detection_msg)
{
  if (model_ == nullptr) {
    RCLCPP_WARN(get_logger(), "Camera Model not yet available");
    return;
  }

  if (image_msg->encoding != "16UC1" && image_msg->encoding != "32FC1") {
    RCLCPP_ERROR(get_logger(), "The image type has not depth info");
    return;
  }

  if (  detection_pub_->get_subscription_count() > 0 || detection_odom_pub_->get_subscription_count() > 0) {
    vision_msgs::msg::Detection3DArray detections_3d_msg;
    detections_3d_msg.header = detection_msg->header;

    cv_bridge::CvImagePtr cv_depth_ptr = cv_bridge::toCvCopy(*image_msg, image_msg->encoding);

    for (const auto & detection : detection_msg->detections) {
      vision_msgs::msg::Detection3D detection_3d_msg;
      detection_3d_msg.header = detection_msg->header;

      vision_msgs::msg::ObjectHypothesisWithPose obj_msg;
      obj_msg.hypothesis.class_id = detection.class_name;
      obj_msg.hypothesis.score = detection.score;
      detection_3d_msg.results.push_back(obj_msg);

      float depth = 0.0;
      if (image_msg->encoding == "16UC1") {
        depth = depth_image_proc::DepthTraits<uint16_t>::toMeters(
        cv_depth_ptr->image.at<uint16_t>(
          cv::Point2d(detection.bbox.center.position.x, detection.bbox.center.position.y)));
      } else {
        depth = cv_depth_ptr->image.at<float>(
          cv::Point2d(detection.bbox.center.position.x, detection.bbox.center.position.y));
      }

      if (std::isnan(depth)) {
        continue;
      }

      cv::Point3d ray = model_->projectPixelTo3dRay(
        model_->rectifyPoint(
          cv::Point2d(
            detection.bbox.center.position.x, detection.bbox.center.position.y)));

      ray = ray / ray.z;
      cv::Point3d point = ray * depth;

      detection_3d_msg.bbox.center.position.x = point.x;
      detection_3d_msg.bbox.center.position.y = point.y;
      detection_3d_msg.bbox.center.position.z = point.z;

      detections_3d_msg.detections.push_back(detection_3d_msg);
    }

    if (!detections_3d_msg.detections.empty()) {
      detection_pub_->publish(detections_3d_msg);
      to_odom(detections_3d_msg);
    }
  }
}

void
DetectionTo3DfromDepthNode::to_odom(vision_msgs::msg::Detection3DArray & detections_3d)
{
  detections_3d_odom_.header = detections_3d.header;
  detections_3d_odom_.header.frame_id = fixed_frame_;
  detections_3d_odom_.detections.clear();

  tf2::Stamped<tf2::Transform> camera2odom;
  std::string error;

  if (tf_buffer_.canTransform(fixed_frame_, detections_3d.header.frame_id, tf2::TimePointZero, &error))
  {
    auto camera2odom_msg = tf_buffer_.lookupTransform(fixed_frame_, detections_3d.header.frame_id, tf2::TimePointZero);

    tf2::fromMsg(camera2odom_msg, camera2odom);

    for (const auto & detection : detections_3d.detections) {
      tf2::Transform pose_camera;
      tf2::fromMsg(detection.bbox.center, pose_camera);

      tf2::Transform pose_odom = camera2odom * pose_camera;

      vision_msgs::msg::Detection3D detection_odom;

      detection_odom.header = detection.header;
      detection_odom.results = detection.results;
      detection_odom.header.frame_id = fixed_frame_;  // "odom"

      tf2::toMsg(pose_odom, detection_odom.bbox.center);

      detections_3d_odom_.detections.push_back(detection_odom);
    }

    detection_odom_pub_->publish(detections_3d_odom_);
  }
}

}  // namespace person_tracker