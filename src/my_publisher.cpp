// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague .. 2019, paplhjak

#include <point_cloud_transport/point_cloud_transport.hpp>
#include <rclcpp/rclcpp.hpp>
// #include <rosbag/bag.h>
// #include <rosbag/view.h>
#include <sensor_msgs/msg/point_cloud2.h>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto g_node = rclcpp::Node::make_shared("point_cloud_publisher");

  point_cloud_transport::PointCloudTransport pct(g_node);
  point_cloud_transport::Publisher pub = pct.advertise("pct/point_cloud", rmw_qos_profile_sensor_data);
  //
  // rosbag::Bag bag;
  // bag.open(argv[1], rosbag::bagmode::Read);
  //
  rclcpp::Rate loop_rate(5);
  // for (const auto& m: rosbag::View(bag))
  while(true)
  {
  //   sensor_msgs::PointCloud2::ConstPtr i = m.instantiate<sensor_msgs::PointCloud2>();
  //   if (i != nullptr)
  //   {
  //     pub.publish(i);
      rclcpp::spin_some(g_node);
      loop_rate.sleep();
  //   }
  //
    if (!rclcpp::ok())
      break;
  }
  return 0;
}
