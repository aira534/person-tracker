#include "person_tracker/LidarTrackerNode.hpp"

#include <cstdint>
#include <string>

using std::placeholders::_1;

namespace person_tracker {

LidarTrackerNode::LidarTrackerNode()
: Node("detections_tracking_node"),
  tf_buffer_(),
  tf_listener_(tf_buffer_),
  new_id(0)
{
  centers_subscriber_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/lidar_objects", 10,
      std::bind(&LidarTrackerNode::centers_callback, this, _1));

  entities_pub_ = create_publisher<person_tracker_interfaces::msg::DetectedEntities>("/entities_info", 10);
  this->declare_parameter<std::string>("fixed_frame", "odom");

  fixed_frame_ = this->get_parameter("fixed_frame").as_string();

  this->declare_parameter<int>("max_misses", 10);
  max_misses_ = this->get_parameter("max_misses").as_int();
  // debug_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
  //   "/objects_odom_debug", 10);

  entities_markers_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "tracked_entities_markers", 10);
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
    double pred_x = entity.position.x;
    double pred_y = entity.position.y;

    // Solo proyectar si la vimos en el frame anterior (misses == 0)
    if (entity.misses == 0 && dt > 0.0 && dt < 0.4) {
      pred_x += entity.vx * dt;
      pred_y += entity.vy * dt;
    }

    double margin = 0.6;
    double best_dist = margin * margin;
    std::size_t best_index = points_odom_.size();

    for (std::size_t i = 0; i < points_odom_.size(); i++) {
      if (!detection_associated[i]) {
        double dist_x = pred_x - points_odom_[i].x;
        double dist_y = pred_y - points_odom_[i].y;
        double dist_sq = dist_x * dist_x + dist_y * dist_y;

        if (dist_sq < best_dist) {
          best_dist = dist_sq;
          best_index = i;
        }
      }
    }

    if (best_index != points_odom_.size()) {
      // update vectors with new info

      if (dt > 0) {
        entity.vx = (points_odom_[best_index].x - entity.position.x) / dt;
        entity.vy = (points_odom_[best_index].y - entity.position.y) / dt;
      }

      entity.last_seen = stamp;

      entity.position = points_odom_[best_index];
      entity.misses = 0;
      detection_associated[best_index] = true;
    } else {
      entity.misses++;
      entity.vx = 0.0;
      entity.vy = 0.0;
    }
  }

  for (std::size_t i = 0; i < detection_associated.size(); i++) {
    if (!detection_associated[i]) {
      Entity new_entity;
      new_entity.id = ++new_id;
      new_entity.last_seen = stamp;
      new_entity.misses = 0;
      new_entity.position = points_odom_[i];
      new_entity.vx = 0.0;
      new_entity.vy = 0.0;

      entities_.push_back(new_entity);
    }
  }

  for (auto it = entities_.begin(); it != entities_.end(); )
  {
    if (it->misses >= max_misses_)
    {
      it = entities_.erase(it);
    }
    else
    {
      ++it;
    }
  }
  publishEntities(stamp);
}

//debug

void LidarTrackerNode::publishEntities(const rclcpp::Time & stamp)
{
  using Marker = visualization_msgs::msg::Marker;

  visualization_msgs::msg::MarkerArray markers;
  markers.markers.reserve(1 + 2 * entities_.size());

  // Limpiar la visualización anterior.
  Marker clear;
  clear.header.frame_id = "odom";
  clear.header.stamp = stamp;
  clear.action = Marker::DELETEALL;
  markers.markers.push_back(clear);

  for (const Entity & entity : entities_)
  {
    // Esfera en la posición del cluster.
    Marker sphere;
    sphere.header.frame_id = "odom";
    sphere.header.stamp = stamp;

    sphere.ns = "entity_positions";
    sphere.id = static_cast<std::int32_t>(entity.id);
    sphere.type = Marker::SPHERE;
    sphere.action = Marker::ADD;

    sphere.pose.position = entity.position;
    sphere.pose.orientation.w = 1.0;

    sphere.scale.x = 0.12;
    sphere.scale.y = 0.12;
    sphere.scale.z = 0.12;

    // Verde: asociada en este escaneo.
    // Naranja: conservada, pero con detecciones perdidas.
    sphere.color.r = entity.misses == 0 ? 0.1f : 1.0f;
    sphere.color.g = entity.misses == 0 ? 1.0f : 0.5f;
    sphere.color.b = 0.1f;
    sphere.color.a = 1.0f;

    markers.markers.push_back(sphere);

    // Texto encima de la esfera.
    Marker text;
    text.header = sphere.header;

    text.ns = "entity_ids";
    text.id = static_cast<std::int32_t>(entity.id);
    text.type = Marker::TEXT_VIEW_FACING;
    text.action = Marker::ADD;

    text.pose = sphere.pose;
    text.pose.position.x += 0.10;
    text.pose.position.z += 0.10;

    text.scale.z = 0.10;

    text.color.r = 1.0f;
    text.color.g = 1.0f;
    text.color.b = 1.0f;
    text.color.a = 1.0f;

    text.text = std::to_string(entity.id);

    markers.markers.push_back(text);
  }

  entities_markers_pub_->publish(markers);
}
}
