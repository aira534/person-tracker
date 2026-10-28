#include "person_tracker/LidarTrackerNode.hpp"

using std::placeholders::_1;

namespace person_tracker {

LidarTrackerNode::LidarTrackerNode()
: Node("detections_tracking_node"),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  centers_subscriber_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/lidar_objects", 10,
      std::bind(&LidarTrackerNode::centers_callback, this, _1));

  entities_pub_ = create_publisher<person_tracker_interfaces::msg::DetectedEntities>("/entities_info", 10);
  this->declare_parameter<std::string>("fixed_frame", "odom");

  fixed_frame_ = this->get_parameter("fixed_frame").as_string();
  // debug_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
  //   "/objects_odom_debug", 10);
}

void
LidarTrackerNode::centers_callback(geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  tf2::Stamped<tf2::Transform> odom2laser;
  std::string error;

  if (tf_buffer_.canTransform(fixed_frame_, msg->header.frame_id, tf2::TimePointZero, &error)) {
    auto odom2laser_msg = tf_buffer_.lookupTransform(
      fixed_frame_, msg->header.frame_id, tf2::TimePointZero);

    tf2::fromMsg(odom2laser_msg, odom2laser);

    points_odom_.clear();
    for (const auto & pose : msg->poses) {
      tf2::Vector3 p_laser(pose.position.x, pose.position.y, pose.position.z);
      tf2::Vector3 p_odom = odom2laser * p_laser;
      geometry_msgs::msg::Point p;

      p.x = p_odom.x();
      p.y = p_odom.y();
      p.z = 0;
      points_odom_.push_back(p);
    }
  }

  updateEntities(rclcpp::Time(msg->header.stamp));

  // for (size_t i = 0; i < points_odom_.size(); i++) {
  //   for (size_t j = i + 1; j < points_odom_.size(); j++) {
  //     double dx = points_odom_[j].x - points_odom_[i].x;
  //     double dy = points_odom_[j].y - points_odom_[i].y;
  //     double dist = sqrt(dx *dx + dy * dy);

  //     if (dist < 0.8) {
  //       RCLCPP_INFO(
  //         get_logger(), "Puntos %zu y %zu: distancia %.3f m",
  //         i, j, dist);
  //     }
  //   }
  // }

  // geometry_msgs::msg::PoseArray dbg;
  // dbg.header.stamp = msg->header.stamp;
  // dbg.header.frame_id = fixed_frame_;

  // for (const auto & pt : points_odom_) {
  //   geometry_msgs::msg::Pose pose;
  //   pose.position = pt;
  //   pose.orientation.w = 1.0;
  //   dbg.poses.push_back(pose);
  // }

  // debug_pub_->publish(dbg);
}

void LidarTrackerNode::updateEntities(const rclcpp::Time & stamp)
{
  std::vector<bool> detection_associated(points_odom_.size(), false);

  for (Entity & entity: entities_) {
    double dt = (stamp - entity.last_seen).seconds();
    double pred_x = entity.position.x + entity.vx * dt;
    double pred_y = entity.position.y + entity.vy * dt;

    double margin = 0.25;
    double best_dist = margin * margin;
    std::size_t best_index = points_odom_.size();

    for (std::size_t i = 0; i < points_odom_.size(); i++) {
      if (!detection_associated[i]) {
        double dist_x = pred_x - points_odom_[i].x;
        double dist_y = pred_y - points_odom_[i].y;
        double dist = sqrt(dist_x * dist_x + dist_y * dist_y);

        if (dist < best_dist) {
          best_dist = dist;
          best_index = i;
        }
      }
    }

    if (best_index != points_odom_.size()) {
      // update vectors with new info
      entity.vx = (points_odom_[best_index].x - entity.position.x) / dt;
      entity.vy = (points_odom_[best_index].y - entity.position.y) / dt;

      entity.last_seen = stamp;

      entity.position = points_odom_[best_index];
      detection_associated[best_index] = true;
    } else {
      entity.misses++;
    }

    for (std::size_t i = 0; i < detection_associated.size(); i++) {
      if (!detection_associated[i]) {
        /* continuar generar nueva entidad*/
        void();
      }

    
    }
  }
}
}
