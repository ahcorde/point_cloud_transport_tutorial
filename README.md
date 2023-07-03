# \<POINT CLOUD TRANSPORT TUTORIAL>
 **v0.1.**

_**Contents**_

  * [Writing a Simple Publisher](#writing-a-simple-publisher)
    * [Code of the Publisher](#code-of-the-publisher)
    * [Code Explained](#code-of-publisher-explained)
    * [Example of Running the Publisher](#example-of-running-the-publisher)
  * [Writing a Simple Subscriber](#writing-a-simple-subscriber)
    * [Code of the Subscriber](#code-of-the-subscriber)
    * [Code Explained](#code-of-subscriber-explained)
    * [Example of Running the Subscriber](#example-of-running-the-subscriber)
  * [Using Publishers And Subscribers With Plugins](#using-publishers-and-subscribers-with-plugins)
    * [Running the Publisher and Subsriber](#running-the-publisher-and-subsriber)
    * [Changing the Transport Used](#changing-the-transport-used)
    * [Changing Transport Behavior](#changing-transport-behavior)
  * [Implementing Custom Plugins](#implementing-custom-plugins)

# Writing a Simple Publisher
In this section, we'll see how to create a publisher node, which opens a .bag file and publishes PointCloud2 messages from it.

This tutorial assumes, that you have created your workspace during [<point_cloud_transport>](https://github.com/john-maidbot/point_cloud_transport) [installation](https://github.com/john-maidbot/point_cloud_transport#installation).

Before we start, change to the directory and clone this repository:
~~~~~ bash
$ cd ~/point_cloud_transport_ws/src
$ git clone https://github.com/ahcorde/point_cloud_transport_tutorial.git -b ros2
~~~~~

## Code of the Publisher
Take a look at [my_publisher.cpp](src/my_publisher.cpp):
```cpp
// ROS
#include <point_cloud_transport/point_cloud_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <rosbag2_cpp/types/introspection_message.hpp>
#include <rosbag2_storage/storage_options.hpp>

// ROS MSG
#include <sensor_msgs/msg/point_cloud2.hpp>

int main(int argc, char** argv)
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
```
## Code of Publisher Explained
Now we'll break down the code piece by piece.

Header for including [<point_cloud_transport>](https://github.com/john-maidbot/point_cloud_transport):

```cpp
#include <point_cloud_transport/point_cloud_transport.hpp>
```
Headers for opening rosbag2 file with `sensor_msgs::msg::PointCloud2` messages:
```cpp
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <rosbag2_cpp/types/introspection_message.hpp>
#include <rosbag2_storage/storage_options.hpp>
```

Initializing the ROS node:

```cpp
rclcpp::init(argc, argv);
auto node = rclcpp::Node::make_shared("point_cloud_publisher");
```
Creates *PointCloudTransport* instance and initializes it with our *Node*. Methods of *PointCloudTransport* can later be used to create point cloud publishers and subscribers similar to how methods of *Node* are used to create generic publishers and subscribers.

```cpp
point_cloud_transport::PointCloudTransport pct(node);
```

Uses *PointCloudTransport* method to create a publisher on base topic *"pct/point_cloud"*. Depending on whether more plugins are built, additional (per-plugin) topics derived from the base topic may also be advertised. The second argument is the size of our publishing queue.

```cpp
point_cloud_transport::Publisher pub = pct.advertise(
  "pct/point_cloud",
  rmw_qos_profile_sensor_data);
```

Opens .bag file given as an argument to the program:
```cpp
rosbag2_cpp::readers::SequentialReader reader;
rosbag2_storage::StorageOptions storage_options{};

storage_options.uri = std::string(argv[1]);
storage_options.storage_id = "mcap";

rosbag2_cpp::ConverterOptions converter_options{};
converter_options.input_serialization_format = "cdr";
converter_options.output_serialization_format = "cdr";
reader.open(storage_options, converter_options);
```

Publishes `sensor_msgs::msg::PointCloud2` message from the specified rosbag2 with frequency of 5Hz:

```cpp
  rclcpp::Rate loop_rate(1);
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
      pub.publish(message);
    }

    loop_rate.sleep();

    if (!rclcpp::ok()) {
      break;
    }
  }
```

## Example of Running the Publisher

To run [my_publisher.cpp](src/my_publisher.cpp) open terminal in the root of workspace and run the following:

```bash
colcon build --merge-install --event-handlers console_direct+
source install/setup.bash
ros2 run point_cloud_transport_tutorial my_publisher <dir to rosbag2 directory>
```

# Writing a Simple Subscriber

In this section, we'll see how to create a subscriber node, which receives `PointCloud2` messages and prints the number of points in them.

## Code of the Subscriber
Take a look at [my_subscriber.cpp](src/my_subscriber.cpp):

```cpp
#include <point_cloud_transport/point_cloud_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

void Callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  std::cout << "Message received, number of points is: " << msg->width * msg->height << std::endl;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("point_cloud_subscriber");

  point_cloud_transport::PointCloudTransport pct(node);
  point_cloud_transport::Subscriber sub =
    pct.subscribe("pct/point_cloud", 10, Callback);
  rclcpp::spin(node);

  return 0;
}
```

## Code of Subscriber Explained

Now we'll break down the code piece by piece.

Header for including [<point_cloud_transport>](https://github.com/john-maidbot/point_cloud_transport):

```cpp
#include <point_cloud_transport/point_cloud_transport.hpp>
```
A callback function, which we will bind to the subscriber. Whenever our subscriber receives a message, the Callback function gets executed and number of points in the message (*msg->width * msg->height*) is printed.

```cpp
void Callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  std::cout << "Message received, number of points is: " << msg->width * msg->height << std::endl;
}
```

Initializes the ROS node:

```cpp
rclcpp::init(argc, argv);
auto node = rclcpp::Node::make_shared("point_cloud_subscriber");
```

Creates *PointCloudTransport* instance and initializes it with our *Node*. Methods of *PointCloudTransport* can later be used to create point cloud publishers and subscribers similar to how methods of *Node* are used to create generic publishers and subscribers.

```cpp
point_cloud_transport::PointCloudTransport pct(node);
```

Uses *PointCloudTransport* method to create a subscriber on base topic *"pct/point_cloud"*. The second argument is the size of our subscribing queue. The third argument tells the subscriber to execute Callback function, whenever a message is received.

```cpp
point_cloud_transport::Subscriber sub =
  pct.subscribe("pct/point_cloud", 10, Callback);
```

## Example of Running the Subscriber
To run [my_subscriber.cpp](src/my_subscriber.cpp), open terminal in the root of workspace and run the following:

```bash
colcon build --merge-install --event-handlers console_direct+
source install/setup.bash
ros2 run point_cloud_transport_tutorial my_subscriber
```

# Using Publishers And Subscribers With Plugins

In this section, we'll first make sure that the nodes are running properly. Later on, we'll change the transport to use Draco compressed format.

## Running the Publisher and Subsriber

We can run the Publisher/Subsriber nodes. To run both start two terminal tabs and enter commands:

```bash
source install/setup.bash
ros2 run point_cloud_transport_tutorial my_subscriber
```

And in the second tab:
```bash
source install/setup.bash
ros2 run point_cloud_transport_tutorial my_publisher <dir to rosbag2 directory>
```

If both nodes are running properly, you should see the subscriber node start printing out messages similar to:
```bash
Message received, number of points is: 76800
```

To list the topics, which are being published and subscribed to, enter command:

```bash
ros2 topic list -v
```

The output should look similar to this:

```bash
Published topics:
 * /parameter_events [rcl_interfaces/msg/ParameterEvent] 2 publishers
 * /pct/point_cloud [sensor_msgs/msg/PointCloud2] 1 publisher
 * /pct/point_cloud/draco [point_cloud_interfaces/msg/CompressedPointCloud2] 1 publisher
 * /rosout [rcl_interfaces/msg/Log] 2 publishers

Subscribed topics:
 * /parameter_events [rcl_interfaces/msg/ParameterEvent] 1 subscribe
```

To display the ROS computation graph, enter command:

```bash
rqt_graph
```

You should see a graph similar to this:

![Graph1](https://github.com/paplhjak/point_cloud_transport_tutorial/blob/master/readme_images/rosgraph1.png)

## Changing the Transport Used
Currently our nodes are communicating raw sensor_msgs/PointCloud2 messages, so we are not gaining anything over using basic ros::Publisher and ros::Subscriber. We can change that by introducing a new transport.

The [<draco_point_cloud_transport>](https://github.com/paplhjak/draco_point_cloud_transport) package provides plugin for   [<point_cloud_transport>](https://github.com/paplhjak/point_cloud_transport), which sends point clouds over the wire encoded through kd-tree or sequantial compression. Notice that draco_point_cloud_transport is not a dependency of your package; point_cloud_transport automatically discovers all transport plugins built in your ROS system.

Assuming you have followed [<point_cloud_transport>](https://github.com/paplhjak/point_cloud_transport) [installation](https://github.com/paplhjak/point_cloud_transport#installation), you should already have [<draco_point_cloud_transport>](https://github.com/paplhjak/draco_point_cloud_transport) built.

To check which plugins are built on your machine, enter command:

```bash
ros2 run point_cloud_transport list_transports
```
You should see output similar to:

``` bash
Lookup name: point_cloud_transport/draco_pub
Transport name: point_cloud_transport/draco
Lookup name: point_cloud_transport/raw_pub
Transport name: point_cloud_transport/raw
Declared transports:
point_cloud_transport/draco

point_cloud_transport/raw


Details:
----------
"point_cloud_transport/draco"
 - Provided by package: draco_point_cloud_transport
 - Publisher:
            This plugin publishes a CompressedPointCloud2 using KD tree compression.

 - Subscriber:
            This plugin decompresses a CompressedPointCloud2 topic.

----------
"point_cloud_transport/raw"
 - Provided by package: point_cloud_transport
 - Publisher:
            This is the default publisher. It publishes the PointCloud2 as-is on the base topic.

 - Subscriber:
            This is the default pass-through subscriber for topics of type sensor_msgs/PointCloud2.
```

Shut down your publisher node and restart it. If you list the published topics again and have [<draco_point_cloud_transport>](https://github.com/john-maidbot/point_cloud_transport_plugins) installed, you should see a new one:

```bash
 * /pct/point_cloud/draco [draco_point_cloud_transport/CompressedPointCloud2] 1 publisher
```

Now let's start up a new subscriber, this one using draco transport. The key is that [<point_cloud_transport>](https://github.com/john-maidbot/point_cloud_transport) subscribers check the parameter ~point_cloud_transport for the name of a transport to use in place of "raw". Let's set this parameter and start a subscriber node with name "draco_listener":

```bash
ros2 run point_cloud_transport_tutorial my_subscriber --ros-args -r __node:=draco_listener
ros2 param set /point_cloud_subscriber  point_cloud_transport draco
```

If we check the node graph again:

```bash
rqt_graph
```

![Graph2](https://github.com/paplhjak/point_cloud_transport_tutorial/blob/master/readme_images/rosgraph2.png)

We can see, that draco_listener is listening to a separate topic carrying compressed messages.

Note that if you just want the draco messages, you can change the parameter globally in-line:

```bash
ros2 run point_cloud_transport_tutorial my_subscriber --ros-args -r __node:=draco_listener -p point_cloud_transport:=draco
```

## Changing Transport Behavior

For a particular transport, we may want to tweak settings such as compression level and speed, quantization of particular attributes of point cloud, etc. Transport plugins can expose such settings through rqt_reconfigure. For example, /point_cloud_transport/draco/ allows you to change multiple parameters of the compression on the fly.

For now let's adjust the position quantization. By default, "draco" transport uses quantization of 14 bits, allowing 16384 distinquishable positions in each axis; let's change it to 8 bits (256 positions):

``` bash
ros2 run rqt_reconfigure rqt_reconfigure
```

Now pick `/pct/point_cloud/draco` in the drop-down menu and move the quantization_POSITION slider down to 8. If you visualize the messages, such as in RVIZ, you should be able to see the level of detail of the point cloud drop.

Dynamic Reconfigure has updated the dynamically reconfigurable parameter `/pct/point_cloud/draco/quantization_POSITION`. You can verify this by running:

``` bash
ros2 param get /point_cloud_subscriber /pct/point_cloud/draco/quantization_POSITION
```

This should display 8.

Full explanation of the reconfigure parameters and an example of how to use them can be found at [<draco_point_cloud_transport>](https://github.com/john-maidbot/point_cloud_transport_plugins) repository.


# Implementing Custom Plugins
The process of implementing your own plugins is described in a separate [repository](https://github.com/john-maidbot/templateplugin_point_cloud_transport).
