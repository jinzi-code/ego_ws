//
// Created by Hyungtae Lim on 6/23/21.
//

// For disable PCL complile lib, to use PointXYZILID
#define PCL_NO_PRECOMPILE
#include "patchwork/patchwork.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cstdlib>


using PointType = pcl::PointXYZ;
using namespace std;

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr CloudPublisher;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr PositivePublisher;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr NegativePublisher;
rclcpp::Node::SharedPtr g_node;

pcl::PointCloud<PointType> pc_curr;
pcl::PointCloud<PointType> pc_ground;
pcl::PointCloud<PointType> pc_non_ground;
static double time_taken;

boost::shared_ptr<PatchWork<PointType> > PatchworkGroundSeg;

std::string output_filename;
std::string acc_filename, pcd_savepath;
string      algorithm;
string      mode;
string      seq;
bool        save_flag;

template<typename T>
pcl::PointCloud<T> cloudmsg2cloud(const sensor_msgs::msg::PointCloud2 &cloudmsg)
{
    pcl::PointCloud<T> cloudresult;
    pcl::fromROSMsg(cloudmsg,cloudresult);
    return cloudresult;
}

template<typename T>
sensor_msgs::msg::PointCloud2 cloud2msg(const pcl::PointCloud<T>& cloud, const std_msgs::msg::Header& header)
{
    sensor_msgs::msg::PointCloud2 cloud_ROS;
    pcl::toROSMsg(cloud, cloud_ROS);
    cloud_ROS.header = header;
    return cloud_ROS;
}

void callbackNode(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::fromROSMsg(*msg,pc_curr);
    PatchworkGroundSeg->estimate_ground(pc_curr, pc_ground, pc_non_ground, time_taken);
    CloudPublisher->publish(cloud2msg(pc_curr, msg->header));
    PositivePublisher->publish(cloud2msg(pc_ground, msg->header));
    NegativePublisher->publish(cloud2msg(pc_non_ground, msg->header));
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<rclcpp::Node>("benchmark");
    g_node = nh;
    nh->declare_parameter<std::string>("algorithm", "patchwork");
    nh->get_parameter("algorithm", algorithm);

    auto NodeSubscriber = nh->create_subscription<sensor_msgs::msg::PointCloud2>("/registered_scan", rclcpp::SensorDataQoS(), callbackNode);

    rclcpp::Rate loop_rate(10);

    PatchworkGroundSeg.reset(new PatchWork<PointType>(nh));

    CloudPublisher  = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/benchmark/cloud", 100);
    PositivePublisher     = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/benchmark/P", 100);
    NegativePublisher     = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/benchmark/N", 100);

    string filename = "/home/dango/dango_file2/graduation_project/catkin_ws/005269.pcd";

    // An example for Loading own data
    //pcl::PointCloud<PointType> pc_curr;
    //if (pcl::io::loadPCDFile<PointType> (filename, pc_curr) == -1) //* load the file
    //{
    //    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    //    return (-1);
    //}

    
    cout << "Operating patchwork..." << endl;

    while (rclcpp::ok()){
        rclcpp::spin_some(nh);
        loop_rate.sleep();
    }






    rclcpp::shutdown();
    return 0;
}
