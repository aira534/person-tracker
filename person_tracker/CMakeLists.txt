cmake_minimum_required(VERSION 3.8)
project(person_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yolo_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(person_tracker_interfaces REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(dependencies
    rclcpp
    std_msgs
    rclcpp_lifecycle
    yolo_msgs
    vision_msgs
    sensor_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    laser_geometry
    person_tracker_interfaces
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Using classes
# add_executable(logger_class src/logger_class.cpp)
# ament_target_dependencies(logger_class ${dependencies})

# Using a library
include_directories(include)
add_library(${PROJECT_NAME} SHARED
  src/person_tracker/PersonTrackerNode.cpp
  src/person_tracker/ObjectProcessingNode.cpp
  src/person_tracker/LidarTrackerNode.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(person_tracker_main src/person_tracker_main.cpp)
ament_target_dependencies(person_tracker_main ${dependencies})
target_link_libraries(person_tracker_main ${PROJECT_NAME})

add_executable(object_processing_main src/object_processing_main.cpp)
ament_target_dependencies(object_processing_main ${dependencies})
target_link_libraries(object_processing_main ${PROJECT_NAME})

add_executable(lidar_tracker_main src/lidar_tracker_main.cpp)
ament_target_dependencies(lidar_tracker_main ${dependencies})
target_link_libraries(lidar_tracker_main ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  person_tracker_main
  object_processing_main
  lidar_tracker_main
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})
ament_package()
