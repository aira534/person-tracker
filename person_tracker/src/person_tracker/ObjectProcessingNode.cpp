// the aim of this node is to classify the detected points into objects, publishing a PoseArray containing their centers,
// in order to be able in the future to create a direction vector

#include "person_tracker/ObjectProcessingNode.hpp"

using std::placeholders::_1;

namespace person_tracker {

ObjectProcessingNode::ObjectProcessingNode()
: Node("object_processing_node") {
  laser_subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan_filtered", 10,
      std::bind(&ObjectProcessingNode::laser_callback, this, _1));

  objects_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/lidar_objects", 10);

  this->declare_parameter<double>("max_dist", 0.50);
  this->declare_parameter<double>("min_obj_size", 0.05);
  this->declare_parameter<double>("max_obj_size", 1.00);
}

void
ObjectProcessingNode::laser_callback(sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // convert to point cloud using laser_geometry
  sensor_msgs::msg::PointCloud2 point_cloud;
  laser_projector_.projectLaser(*msg, point_cloud);

  // array to store the center of all the entities detected
  geometry_msgs::msg::PoseArray objects_pos;
  objects_pos.header = point_cloud.header;

  std::vector<geometry_msgs::msg::Point> current_object;

  float MAX_DIST = this->get_parameter("max_dist").as_double();
  float MIN_OBJ_SIZE = this->get_parameter("min_obj_size").as_double();
  float MAX_OBJ_SIZE = this->get_parameter("max_obj_size").as_double();

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(point_cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(point_cloud, "y");

  geometry_msgs::msg::Point prev_point;
  prev_point.x = 0.0;
  prev_point.y = 0.0;
  prev_point.z = 0.0;
  bool has_prev_point = false;

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
    geometry_msgs::msg::Point current_point;
    current_point.x = *iter_x;
    current_point.y = *iter_y;
    current_point.z = 0.0;

    // eliminate inf points
    if (std::isnan(current_point.x) || std::isnan(current_point.y)) {
        continue;
    }

    // filter detections a certain distance from the robot
    float dist_from_robot = std::sqrt((current_point.x * current_point.x) + (current_point.y * current_point.y));
    if (dist_from_robot > 5.0) { 
        continue; 
    }

    if (has_prev_point) {
      float dist_x = current_point.x - prev_point.x;
      float dist_y = current_point.y - prev_point.y;
      float dist_to_prev = std::sqrt((dist_x * dist_x) + (dist_y * dist_y)); 

      // if they are further away from each other then its another object
      if (dist_to_prev > MAX_DIST) {
        // calculate center of the object
        calc_object_center(current_object, objects_pos, MIN_OBJ_SIZE, MAX_OBJ_SIZE);
        current_object.clear();
      }
    }

    current_object.push_back(current_point);
    prev_point = current_point;
    has_prev_point = true;
  }

  calc_object_center(current_object, objects_pos, MIN_OBJ_SIZE, MAX_OBJ_SIZE);
  objects_pub_->publish(objects_pos);
}


void
ObjectProcessingNode::calc_object_center(const std::vector<geometry_msgs::msg::Point>& object,
                                          geometry_msgs::msg::PoseArray& detections_centers, float min_size, float max_size)
{
  if (object.empty()) return;

  geometry_msgs::msg::Point first = object.front();
  geometry_msgs::msg::Point last = object.back();
  
  float dist_x = last.x - first.x;
  float dist_y = last.y - first.y;
  float width = std::sqrt((dist_x * dist_x) + (dist_y * dist_y));

  if (width >= min_size && width <= max_size) {
      float sum_x = 0, sum_y = 0;
      for (const auto& point : object) {
          sum_x += point.x;
          sum_y += point.y;
      }

      geometry_msgs::msg::Pose center_pose;
      center_pose.position.x = sum_x / object.size();
      center_pose.position.y = sum_y / object.size();
      center_pose.position.z = 0.0;
      
      center_pose.orientation.w = 1.0;

      detections_centers.poses.push_back(center_pose);
  }
}

}
