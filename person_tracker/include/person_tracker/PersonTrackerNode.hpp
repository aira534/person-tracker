#ifndef PERSON_TRACKER__PERSONTRACKERNODE_HPP_
#define PERSON_TRACKER__PERSONTRACKERNODE_HPP_

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "person_tracker_interfaces/msg/detected_entities.hpp"
#include "person_tracker_interfaces/msg/entity.hpp"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/transform_datatypes.h"
#include <vector>
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "rclcpp/rclcpp.hpp"

namespace person_tracker
{

class PersonTrackerNode : public rclcpp::Node
{
public:
  PersonTrackerNode();
  void centers_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  void updateEntities(const rclcpp::Time & stamp);
  void camera_callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg);

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr centers_subscriber_;
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr camera_subscriber_;
  rclcpp::Publisher<person_tracker_interfaces::msg::DetectedEntities>::SharedPtr entities_pub_;
  std::vector<geometry_msgs::msg::Point> points_odom_;

  // for every entity that is detected by the lidar
  struct Entity
  {
    int id = -1;
    geometry_msgs::msg::Point position;

    // direction vector
    double vx = 0.0;
    double vy = 0.0;

    // for aging
    rclcpp::Time last_seen{0, 0, RCL_ROS_TIME};
    int misses = 0;

    // false if the position was set by the camera: then it can't be used to compute
    // the velocity with the next lidar position (different sensors, different errors)
    bool position_from_lidar = false;
  };

  // global greedy association: all (entity, point) pairs closer than margin, from closest
  // to farthest, each entity and each point used at most once. Returns (entity, point) indices
  std::vector<std::pair<std::size_t, std::size_t>>
    associate_closest(const std::vector<geometry_msgs::msg::Point> & entity_positions,
                                                                      const std::vector<geometry_msgs::msg::Point> & points,
                                                                      double margin);

  std::vector<Entity> entities_;
  int new_id;
  int max_misses_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    entities_markers_pub_;

  void publishEntities(const rclcpp::Time & stamp);
};

}  //  namespace person_tracker

#endif  // PERSON_TRACKER__PERSONTRACKERNODE_HPP_
