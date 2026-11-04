#include "person_tracker/LidarTrackerNode.hpp"

#include <cstdint>
#include <string>

using std::placeholders::_1;

namespace person_tracker {

LidarTrackerNode::LidarTrackerNode()
: Node("detections_tracking_node"),
  new_id(0)
{
  centers_subscriber_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/lidar_objects", 10,
      std::bind(&LidarTrackerNode::centers_callback, this, _1));

  entities_pub_ = create_publisher<person_tracker_interfaces::msg::DetectedEntities>("/tracked_entities", 10);

  this->declare_parameter<int>("max_misses", 10);
  max_misses_ = this->get_parameter("max_misses").as_int();
}

void
LidarTrackerNode::centers_callback(geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  points_odom_.clear();
  for (const auto & pose : msg->poses) {
    geometry_msgs::msg::Point p;

    p.x = pose.position.x;
    p.y = pose.position.y;
    p.z = 0;
    points_odom_.push_back(p);
  }

  updateEntities(rclcpp::Time(msg->header.stamp));
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

// puts actual struct into ros message and publishes it
void LidarTrackerNode::publishEntities(const rclcpp::Time & stamp)
{
  person_tracker_interfaces::msg::DetectedEntities entities_msg;
  entities_msg.header.stamp = stamp;
  entities_msg.header.frame_id = "odom";
  entities_msg.entities.reserve(entities_.size());

  for (const auto & e : entities_)
  {
    person_tracker_interfaces::msg::Entity entity_msg;
    entity_msg.id = e.id;
    entity_msg.position = e.position;
    entity_msg.velocity.x = e.vx;
    entity_msg.velocity.y = e.vy;
    entity_msg.velocity.z = 0.0;
    entity_msg.misses = e.misses;
    entity_msg.last_seen = e.last_seen;

    entities_msg.entities.push_back(entity_msg);
  }

  entities_pub_->publish(entities_msg);
}
}
