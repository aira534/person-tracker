#ifndef PERSON_TRACKER__OBJECTPROCESSINGNODE_HPP_
#define PERSON_TRACKER__OBJECTPROCESSINGNODE_HPP_

#include <random>

#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "rclcpp/rclcpp.hpp"

namespace person_tracker
{

class ObjectProcessingNode : public rclcpp::Node
{
public:
  ObjectProcessingNode();

private:

};

}  //  namespace person_tracker

#endif  // PERSON_TRACKER__OBJECTPROCESSINGNODE_HPP_
