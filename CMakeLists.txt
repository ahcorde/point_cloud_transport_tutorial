cmake_minimum_required(VERSION 3.10.2)
project(point_cloud_transport_tutorial)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(point_cloud_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(my_publisher
	src/my_publisher.cpp
)
ament_target_dependencies(my_publisher
	pluginlib
  point_cloud_transport
  rclcpp
	sensor_msgs
)

add_executable(my_subscriber
	src/my_subscriber.cpp
)

ament_target_dependencies(my_subscriber
	pluginlib
  point_cloud_transport
  rclcpp
	sensor_msgs
)

add_executable(my_encoder
	src/my_encoder.cpp
)

ament_target_dependencies(my_encoder
	pluginlib
  point_cloud_transport
  rclcpp
	sensor_msgs
)

# Install executables
install(
  TARGETS my_publisher my_subscriber my_encoder
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
# find_package(catkin REQUIRED COMPONENTS cras_cpp_common point_cloud_transport rosbag rosgraph_msgs sensor_msgs topic_tools)
#
# catkin_package()
#
# include_directories(${catkin_INCLUDE_DIRS})
#
# # publisher
# add_executable(publisher_test src/my_publisher.cpp)
# add_dependencies(publisher_test ${catkin_EXPORTED_TARGETS})
# target_link_libraries(publisher_test ${catkin_LIBRARIES})
#
# # subscriber
# add_executable(subscriber_test src/my_subscriber.cpp)
# add_dependencies(subscriber_test ${catkin_EXPORTED_TARGETS})
# target_link_libraries(subscriber_test ${catkin_LIBRARIES})
#
# # encoder
# add_executable(encoder_test src/my_encoder.cpp)
# add_dependencies(encoder_test ${catkin_EXPORTED_TARGETS})
# target_link_libraries(encoder_test ${catkin_LIBRARIES})
