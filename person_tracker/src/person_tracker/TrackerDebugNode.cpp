#include "person_tracker/TrackerDebugNode.hpp"

namespace person_tracker
{

TrackerDebugNode::TrackerDebugNode()
: Node("tracker_debug_node")
{
  entities_sub_ = this->create_subscription<person_tracker_interfaces::msg::DetectedEntities>(
    "/tracked_entities", 10,
    std::bind(&TrackerDebugNode::entitiesCallback, this, std::placeholders::_1));

  markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "tracked_entities_markers", 10);

  camera_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/debug/camera_detections", 10);

  camera_detections_sub_ = create_subscription<vision_msgs::msg::Detection3DArray>("/detection_3d", 10, std::bind(
    &TrackerDebugNode::camera_detections_callback, this, std::placeholders::_1));
}

void TrackerDebugNode::entitiesCallback(const person_tracker_interfaces::msg::DetectedEntities::SharedPtr msg)
{
  using Marker = visualization_msgs::msg::Marker;

  visualization_msgs::msg::MarkerArray markers;
  markers.markers.reserve(1 + 2 * msg->entities.size());

  // Limpiar marcadores anteriores
  Marker clear;
  clear.header = msg->header;
  clear.action = Marker::DELETEALL;
  markers.markers.push_back(clear);

  for (const auto & e : msg->entities)
  {
    // Esfera para la posicion
    Marker sphere;
    sphere.header = msg->header;
    sphere.ns = "entity_positions";
    sphere.id = static_cast<std::int32_t>(e.id);
    sphere.type = Marker::SPHERE;
    sphere.action = Marker::ADD;

    sphere.pose.position = e.position;
    sphere.pose.orientation.w = 1.0;

    sphere.scale.x = 0.12;
    sphere.scale.y = 0.12;
    sphere.scale.z = 0.12;

    sphere.color.r = e.misses == 0 ? 0.1f : 1.0f;
    sphere.color.g = e.misses == 0 ? 1.0f : 0.5f;
    sphere.color.b = 0.1f;
    sphere.color.a = 1.0f;

    markers.markers.push_back(sphere);

    // Texto para el ID
    Marker text;
    text.header = msg->header;
    text.ns = "entity_ids";
    text.id = static_cast<std::int32_t>(e.id);
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

    text.text = std::to_string(e.id);

    markers.markers.push_back(text);
  }

  markers_pub_->publish(markers);
}

void
TrackerDebugNode::camera_detections_callback(vision_msgs::msg::Detection3DArray::ConstSharedPtr msg)
{
  visualization_msgs::msg::MarkerArray markers;

  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);

  int id = 0;

  for (const auto & detection : msg->detections) {
    visualization_msgs::msg::Marker marker;
    marker.header = msg->header;
    marker.ns = "camera_detections";
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position = detection.bbox.center.position;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    markers.markers.push_back(marker);
  }

  camera_markers_pub_->publish(markers);
}

}  // namespace person_tracker