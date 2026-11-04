#ifndef PERSON_TRACKER__OBJECTPROCESSINGNODE_HPP_
#define PERSON_TRACKER__OBJECTPROCESSINGNODE_HPP_

#include "sensor_msgs/msg/laser_scan.hpp"
#include "laser_geometry/laser_geometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

// for tf
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "rclcpp/rclcpp.hpp"

namespace person_tracker
{

class ObjectProcessingNode : public rclcpp::Node
{
public:
  ObjectProcessingNode();
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void calc_object_center(const std::vector<geometry_msgs::msg::Point>& object,
                                                  geometry_msgs::msg::PoseArray& detections_centers,
                                                    float min_size, float max_size);
  void to_odom(geometry_msgs::msg::PoseArray& detections_centers);

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr objects_pub_;
  laser_geometry::LaserProjection laser_projector_;
  std::string fixed_frame_;

  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  geometry_msgs::msg::PoseArray object_positions_odom_;

  float MAX_DIST_;
  float MIN_OBJ_SIZE_;
  float MAX_OBJ_SIZE_;
};

}  //  namespace person_tracker

#endif  // PERSON_TRACKER__OBJECTPROCESSINGNODE_HPP_
