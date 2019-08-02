
//
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <point_cloud_transport/point_cloud_transport.h>

//
#include <rosbag/bag.h>
#include <rosbag/view.h>

int message_counter;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_publisher");
    ros::NodeHandle nh;

    point_cloud_transport::PointCloudTransport pct(nh);
    point_cloud_transport::Publisher pub = pct.advertise("pct/point_cloud", 100);

    message_counter = 0;

    rosbag::Bag bag;
    bag.open("/home/user/Downloads/husky.bag", rosbag::bagmode::Read);

    ros::Rate loop_rate(5);

    for(rosbag::MessageInstance const m: rosbag::View(bag))
    {
        if (message_counter>=1000)
        {
         break;
        }
        sensor_msgs::PointCloud2::ConstPtr i = m.instantiate<sensor_msgs::PointCloud2>();
        if (i != nullptr)
        {
            pub.publish(i);

            message_counter++;

            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    return 0;
}
