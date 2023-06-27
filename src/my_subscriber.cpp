// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague .. 2019, paplhjak

#include <point_cloud_transport/point_cloud_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

void Callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
{
  std::cout << "Message received, number of points is: " << msg->width * msg->height << std::endl;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("point_cloud_subscriber");

  point_cloud_transport::PointCloudTransport pct(node);
  point_cloud_transport::Subscriber sub =
    pct.subscribe("pct/point_cloud", 10, Callback);
  rclcpp::spin(node);

  return 0;
}
