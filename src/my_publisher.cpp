/*
 * Copyright (c) 2023, Open Source Robotics Foundation, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 *    * Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>
#include <fstream>
#include <ctime>
#include <iomanip>

// ROS
#include <point_cloud_transport/point_cloud_transport.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/typesupport_helpers.hpp"
#include "rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp"
#include "rosbag2_cpp/types/introspection_message.hpp"
#include <rosbag2_storage/storage_options.hpp>

// ROS MSG
#include "sensor_msgs/msg/point_cloud2.hpp"

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rosbag2_storage::StorageFilter storage_filter;

  if (argc < 2) {
    std::cout << "Wrong number of arguments. Usage: " << argv[0] << " <ROS2bag directory>" <<
      std::endl;
    return -1;
  }

  rosbag2_cpp::readers::SequentialReader reader;
  rosbag2_storage::StorageOptions storage_options{};

  storage_options.uri = std::string(argv[1]);
  storage_options.storage_id = "mcap";

  rosbag2_cpp::ConverterOptions converter_options{};
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";
  reader.open(storage_options, converter_options);

  std::vector<rosbag2_storage::TopicMetadata> topics = reader.get_all_topics_and_types();
  std::map<std::string, std::string> nameTypeMap;
  for (auto & t : topics) {
    std::cout << "meta name: " << t.name << std::endl;
    std::cout << "meta type: " << t.type << std::endl;
    std::cout << "meta serialization_format: " << t.serialization_format << std::endl;
    nameTypeMap[t.name] = t.type;
  }

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("point_cloud_publisher");

  point_cloud_transport::PointCloudTransport pct(node);
  point_cloud_transport::Publisher pub = pct.advertise(
    "pct/point_cloud",
    rmw_qos_profile_sensor_data);

  rclcpp::Rate loop_rate(1);

  auto library_point_cloud2 = rosbag2_cpp::get_typesupport_library(
    "sensor_msgs/msg/PointCloud2",
    "rosidl_typesupport_cpp");
  auto type_support_point_cloud2 = rosbag2_cpp::get_typesupport_handle(
    "sensor_msgs/msg/PointCloud2", "rosidl_typesupport_cpp", library_point_cloud2);
  auto ros_message = std::make_shared<rosbag2_cpp::rosbag2_introspection_message_t>();
  rosbag2_cpp::SerializationFormatConverterFactory factory;
  // todo: check if the deserialization format is really cdr
  std::unique_ptr<rosbag2_cpp::converter_interfaces::SerializationFormatDeserializer>
  cdr_deserializer;
  cdr_deserializer = factory.load_deserializer("cdr");
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> serialized_message;
  ros_message->allocator = rcutils_get_default_allocator();

  while (reader.has_next()) {
    // read serialized data into the message
    serialized_message = reader.read_next();

    ros_message->time_stamp = 0;
    ros_message->message = nullptr;

    if (strcmp(
        nameTypeMap[serialized_message->topic_name].c_str(),
        "sensor_msgs/msg/PointCloud2") == 0)
    {
      sensor_msgs::msg::PointCloud2 message;
      ros_message->message = &message;
      cdr_deserializer->deserialize(serialized_message, type_support_point_cloud2, ros_message);
      std::cout << "PointCloud2 type " << nameTypeMap[serialized_message->topic_name] << std::endl;
      pub.publish(message);
    }

    loop_rate.sleep();

    if (!rclcpp::ok()) {
      break;
    }
  }
  reader.close();
  rclcpp::shutdown();
  return 0;
}
