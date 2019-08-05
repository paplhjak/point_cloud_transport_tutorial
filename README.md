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
  * [Using Publishers And Subsribers With Plugins](#using-publishers-and-subscribers-with-plugins)
  
  * [Managing Plugins](#managing-plugins)
    * [Implementing Custom Plugins](#implementing-custom-plugins)

# Writing a Simple Publisher
In this section, we'll see how to create a publisher node, which opens a .bag file and publishes PointCloud2 messages from it.

This tutorial assumes, that you have created your workspace during point_cloud_transport [installation](https://github.com/paplhjak/point_cloud_transport#installation). 

Before we start, change to the directory and clone this repository:
~~~~~ bash
$ cd ~/point_cloud_transport_ws/src
$ git clone https://github.com/paplhjak/point_cloud_transport_tutorial.git
~~~~~

## Code of the Publisher
Take a look at my_publisher.cpp:
```cpp
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <point_cloud_transport/point_cloud_transport.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_publisher");
    ros::NodeHandle nh;

    point_cloud_transport::PointCloudTransport pct(nh);
    point_cloud_transport::Publisher pub = pct.advertise("pct/point_cloud", 10);

    rosbag::Bag bag;
    bag.open(argv[1], rosbag::bagmode::Read);

    ros::Rate loop_rate(5);
    for(rosbag::MessageInstance const m: rosbag::View(bag))
    {
        sensor_msgs::PointCloud2::ConstPtr i = m.instantiate<sensor_msgs::PointCloud2>();
        if (i != nullptr)
        {
            pub.publish(i);
            ros::spinOnce();
            loop_rate.sleep();
        }
        if (!nh.ok())
        {
         break;
        }
    }
}
```
## Code of Publisher Explained
Now we'll break down the code piece by piece.

Header for including point_cloud_transport:
```cpp
#include <point_cloud_transport/point_cloud_transport.h>
```
Headers for opening .bag file with sensor_msgs::PointCloud2 messages:
```cpp
#include <rosbag/bag.h>
#include <rosbag/view.h>
```
Initializing the ROS node:
```cpp
ros::init(argc, argv, "point_cloud_publisher");
ros::NodeHandle nh;
```
Creates *PointCloudTransport* instance and initializes it with our *NodeHandle*. Methods of *PointCloudTransport* can later be used to create point cloud publishers and subscribers similar to how methods of *NodeHandle* are used to create generic publishers and subscribers.
```cpp
point_cloud_transport::PointCloudTransport pct(nh);
```
Uses *PointCloudTransport* method to create a publisher on base topic *"pct/point_cloud"*. Depending on whether more plugins are built, additional (per-plugin) topics derived from the base topic may also be advertised. The second argument is the size of our publishing queue.
```cpp
point_cloud_transport::Publisher pub = pct.advertise("pct/point_cloud", 10);
```
Opens .bag file given as an argument to the program:
```cpp
rosbag::Bag bag;
bag.open(argv[1], rosbag::bagmode::Read);
```

Publishes sensor_msgs::PointCloud2 message from the specified .bag with frequency of 5Hz:
```cpp
ros::Rate loop_rate(5);
    for(rosbag::MessageInstance const m: rosbag::View(bag))
    {
        sensor_msgs::PointCloud2::ConstPtr i = m.instantiate<sensor_msgs::PointCloud2>();
        if (i != nullptr)
        {
            pub.publish(i);
            ros::spinOnce();
            loop_rate.sleep();
        }
        if (!nh.ok())
        {
         break;
        }
    }
```

## Example of Running the Publisher
To run my_publisher.cpp, open terminal in the root of workspace and run the following:
~~~~~ bash
$ catkin_make_isolated
$ source devel_isolated/setup.bash
$ rosrun point_cloud_transport_tutorial publisher_test /path/to/my_bag.bag
~~~~~
Of course, roscore/master must also be running.

# Writing a Simple Subscriber
In this section, we'll see how to create a subscriber node, which receives PointCloud2 messages and prints the number of points in them.

## Code of the Subsriber
Take a look at my_subscriber.cpp:
```cpp
#include <ros/ros.h>
#include <point_cloud_transport/point_cloud_transport.h>
#include <sensor_msgs/PointCloud2.h>

void Callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    std::cout << "Message received, number of points is: " << msg->width*msg->height << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_cloud_subscriber", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    point_cloud_transport::PointCloudTransport pct(nh);
    point_cloud_transport::Subscriber sub = pct.subscribe("pct/point_cloud", 100, Callback);
    ros::spin();

    return 0;
}
```
## Code of Subscriber Explained
Now we'll break down the code piece by piece.

Header for including point_cloud_transport:
```cpp
#include <point_cloud_transport/point_cloud_transport.h>
```
A callback function, which we will bind to the subscriber. Whenever our subscriber receives a message, the Callback function gets executed and number of points in the message (*msg->width * msg->height*) is printed.
```cpp
void Callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    std::cout << "Message received, number of points is: " << msg->width*msg->height << std::endl;
}
```

Initializes the ROS node:
```cpp
ros::init(argc, argv, "point_cloud_subscriber", ros::init_options::AnonymousName);
ros::NodeHandle nh;
```
Creates *PointCloudTransport* instance and initializes it with our *NodeHandle*. Methods of *PointCloudTransport* can later be used to create point cloud publishers and subscribers similar to how methods of *NodeHandle* are used to create generic publishers and subscribers.
```cpp
point_cloud_transport::PointCloudTransport pct(nh);
```
Uses *PointCloudTransport* method to create a subscriber on base topic *"pct/point_cloud"*. The second argument is the size of our subscribing queue. The third argument tells the subscriber to execute Callback function, whenever a message is received.
```cpp
point_cloud_transport::Subscriber sub = pct.subscribe("pct/point_cloud", 100, Callback);
```

## Example of Running the Subscriber
To run my_subscriber.cpp, open terminal in the root of workspace and run the following:
~~~~~ bash
$ catkin_make_isolated
$ source devel_isolated/setup.bash
$ rosrun point_cloud_transport_tutorial subscriber_test
~~~~~
Of course, roscore/master must also be running.

# Using Publishers And Subsribers With Plugins
In this section, we'll first make sure that the nodes are running properly. Later on, we'll change the transport to use Draco compressed format.
## Running the Publisher and Subsriber
Make sure that roscore/master is up and running:
~~~~~ bash
$ roscore
~~~~~
Now we can run the Publisher/Subsriber nodes. To run both start two terminal tabs and enter commands:
~~~~~ bash
$ source devel_isolated/setup.bash
$ rosrun point_cloud_transport_tutorial subscriber_test
~~~~~
And in the second tab:
~~~~~ bash
$ source devel_isolated/setup.bash
$ rosrun point_cloud_transport_tutorial publisher_test /path/to/my_bag.bag
~~~~~
If both nodes are running properly, you should see the subscriber node start printing information about the point cloud.

To list the topics, which are being published and subscribed to, enter command:
~~~~~ bash
$ rostopic list -v
~~~~~
The output should look similar to this:
~~~~~ bash
Published topics:
 * /rosout [rosgraph_msgs/Log] 1 publisher
 * /pct/point_cloud [sensor_msgs/PointCloud2] 1 publisher
 * /rosout_agg [rosgraph_msgs/Log] 1 publisher

Subscribed topics:
 * /rosout [rosgraph_msgs/Log] 1 subscriber
 * /pct/point_cloud [sensor_msgs/PointCloud2] 1 subscriber
~~~~~

# Managing Plugins

## Implementing Custom Plugins
