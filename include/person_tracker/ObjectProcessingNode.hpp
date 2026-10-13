#ifndef PERSON_TRACKER__OBJECTPROCESSINGNODE_HPP_
#define PERSON_TRACKER__OBJECTPROCESSINGNODE_HPP_

#include <random>

#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "laser_geometry/laser_geometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

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

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr objects_pub_;
  laser_geometry::LaserProjection laser_projector_;
};

}  //  namespace person_tracker

#endif  // PERSON_TRACKER__OBJECTPROCESSINGNODE_HPP_
