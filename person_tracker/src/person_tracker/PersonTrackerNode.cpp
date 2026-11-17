#include "person_tracker/PersonTrackerNode.hpp"

#include <algorithm>
#include <cstdint>
#include <string>
#include <tuple>

using std::placeholders::_1;

namespace person_tracker {

PersonTrackerNode::PersonTrackerNode()
: Node("person_tracker_node"),
  new_id(0)
{
  centers_subscriber_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/lidar_objects", 10,
      std::bind(&PersonTrackerNode::centers_callback, this, _1));

  camera_subscriber_ = create_subscription<vision_msgs::msg::Detection3DArray>(
      "/detections_3d_odom", 10,
      std::bind(&PersonTrackerNode::camera_callback, this, _1));

  entities_pub_ = create_publisher<person_tracker_interfaces::msg::DetectedEntities>("/tracked_entities", 10);

  this->declare_parameter<int>("max_misses", 10);
  max_misses_ = this->get_parameter("max_misses").as_int();
}

void
PersonTrackerNode::centers_callback(geometry_msgs::msg::PoseArray::SharedPtr msg)
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

// only camera creates entities
// a person detected by yolo that is not close to any existing entity is a new person
void
PersonTrackerNode::camera_callback(vision_msgs::msg::Detection3DArray::SharedPtr msg)
{
  double margin = 1.0;  // bigger than the lidar, camera depth is less precise
  rclcpp::Time stamp(msg->header.stamp);

  // retreive both new people camera detections, and the previous ones to see if we have to create a new entity/id
  std::vector<geometry_msgs::msg::Point> people;
  for (const auto & detection : msg->detections) {
    geometry_msgs::msg::Point p = detection.bbox.center.position;
    p.z = 0;
    people.push_back(p);
  }

  std::vector<geometry_msgs::msg::Point> entity_positions;
  for (const Entity & entity : entities_) {
    entity_positions.push_back(entity.position);
  }

  std::vector<bool> person_associated(people.size(), false);

  std::vector<std::pair<std::size_t, std::size_t>> pairs = associate_closest(entity_positions, people, margin);

  for (const auto & pair : pairs) {
    std::size_t e = pair.first;   // entity index
    std::size_t i = pair.second;  // person index
    person_associated[i] = true;

    // the camera still sees this person
    // if the lidar lost it, the camera keeps it alive and gives its position
    Entity & entity = entities_[e];
    if (entity.misses > 0) {
      entity.position = people[i];
      entity.position_from_lidar = false; // position is given by camera now
      entity.last_seen = stamp;
      entity.misses = 0;
    }
  }

  // create a new entity for each not associated
  for (std::size_t i = 0; i < people.size(); i++) {
    if (!person_associated[i]) {
      Entity new_entity;
      new_entity.id = ++new_id;
      new_entity.last_seen = stamp;
      new_entity.misses = 0;
      new_entity.position = people[i];
      new_entity.vx = 0.0;
      new_entity.vy = 0.0;

      entities_.push_back(new_entity);
    }
  }
}

// associates entities to the closest centroid to them with greedy algorithm, which pairs them according to distance
std::vector<std::pair<std::size_t, std::size_t>>
  PersonTrackerNode::associate_closest(const std::vector<geometry_msgs::msg::Point> & entity_positions,
                                      const std::vector<geometry_msgs::msg::Point> & points,
                                      double margin)
{
  // find all (entity, point) pairs that are closer than the margin allowed (dist^2, entity, point)
  std::vector<std::tuple<double, std::size_t, std::size_t>> candidates;
  for (std::size_t e = 0; e < entity_positions.size(); e++) {
    for (std::size_t i = 0; i < points.size(); i++) {
      double dist_x = entity_positions[e].x - points[i].x;
      double dist_y = entity_positions[e].y - points[i].y;
      double dist_sq = dist_x * dist_x + dist_y * dist_y;
      if (dist_sq < margin * margin) {
        candidates.emplace_back(dist_sq, e, i);
      }
    }
  }

  // put closest pairs first
  std::sort(candidates.begin(), candidates.end());

  // accept a pair only if neither the entity nor the point is already used
  std::vector<bool> entity_used(entity_positions.size(), false);
  std::vector<bool> point_used(points.size(), false);

  std::vector<std::pair<std::size_t, std::size_t>> result;

  for (const auto & [dist_sq, e, i] : candidates) {
    // if they are not paired yet, add them to result as a pair and mark them as used so they cannot be used again
    if (!entity_used[e] && !point_used[i]) {
      entity_used[e] = true;
      point_used[i] = true;
      result.emplace_back(e, i);
    }
  }
  return result;
}

void PersonTrackerNode::updateEntities(const rclcpp::Time & stamp)
{
  // predicted position of every entity
  std::vector<geometry_msgs::msg::Point> predicted;
  for (const Entity & entity: entities_) {
    double dt = (stamp - entity.last_seen).seconds();
    geometry_msgs::msg::Point pred = entity.position;

    // predict also while the entity is not seen (occlusions), as long as it exists
    // (the maximum prediction time is limited by max_misses, after that the entity is deleted)
    if (dt > 0.0) {
      pred.x += entity.vx * dt;
      pred.y += entity.vy * dt;
    }
    predicted.push_back(pred);
  }

  double margin = 0.6;
  std::vector<bool> entity_associated(entities_.size(), false);

  std::vector<std::pair<std::size_t, std::size_t>> pairs = associate_closest(predicted, points_odom_, margin);

  for (const auto & pair : pairs) {
    std::size_t e = pair.first;            // entity index
    std::size_t best_index = pair.second;  // lidar point index
    Entity & entity = entities_[e];
    double dt = (stamp - entity.last_seen).seconds();

    // update vectors with new speed only if their position was updated with lidar (camera position is less reliable)
    if (dt > 0 && entity.position_from_lidar) {
      double vx = (points_odom_[best_index].x - entity.position.x) / dt;
      double vy = (points_odom_[best_index].y - entity.position.y) / dt;

      // in case an unreliable velocity comes: it trusts the new velocity a 30% and the old one a 70%
      double alpha = 0.3;
      entity.vx = alpha * vx + (1.0 - alpha) * entity.vx;
      entity.vy = alpha * vy + (1.0 - alpha) * entity.vy;
    }

    entity.last_seen = stamp;
    entity.position = points_odom_[best_index];
    entity.position_from_lidar = true;
    entity.misses = 0;
    entity_associated[e] = true;
  }

  // if it hasnt been seen in this scan increase misses (velocity is kept, it is not used for prediction while misses > 0)
  for (std::size_t e = 0; e < entities_.size(); e++) {
    if (!entity_associated[e]) {
      entities_[e].misses++;
    }
  }

  // lidar clusters not associated to any entity are ignored:
  // the lidar does not create entities (they are created in camera_callback)

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
void PersonTrackerNode::publishEntities(const rclcpp::Time & stamp)
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
}   // namespace person_tracker
