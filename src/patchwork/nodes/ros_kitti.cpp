// For disable PCL complile lib, to use PointXYZIR
#define PCL_NO_PRECOMPILE
#include <patchwork/msg/node.hpp>
#include "patchwork/patchwork.hpp"
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "tools/kitti_loader.hpp"
#include "tools/pcd_loader.hpp"

using namespace std;

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr CloudPublisher;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr TPPublisher;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr FPPublisher;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr FNPublisher;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr PrecisionPublisher;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr RecallPublisher;
rclcpp::Node::SharedPtr g_node;

using PointType = PointXYZILID;
boost::shared_ptr<PatchWork<PointType> > PatchworkGroundSeg;

std::string output_filename;
std::string acc_filename, pcd_savepath;
string      algorithm;
string      mode;
string      seq;
bool        save_flag;

void pub_score(std::string mode, double measure) {
    static int                 SCALE = 5;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id                  = "map";
    marker.header.stamp                     = g_node->now();
    marker.ns                               = "my_namespace";
    marker.id                               = 0;
    marker.type                             = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action                           = visualization_msgs::msg::Marker::ADD;
    if (mode == "p") marker.pose.position.x = 28.5;
    if (mode == "r") marker.pose.position.x = 25;
    marker.pose.position.y                  = 30;

    marker.pose.position.z    = 1;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x            = SCALE;
    marker.scale.y            = SCALE;
    marker.scale.z            = SCALE;
    marker.color.a            = 1.0; // Don't forget to set the alpha!
    marker.color.r            = 0.0;
    marker.color.g            = 1.0;
    marker.color.b            = 0.0;
    //only if using a MESH_RESOURCE marker type:
    marker.text               = mode + ": " + std::to_string(measure);
    if (mode == "p") PrecisionPublisher->publish(marker);
    if (mode == "r") RecallPublisher->publish(marker);

}

template<typename T>
pcl::PointCloud<T> cloudmsg2cloud(const sensor_msgs::msg::PointCloud2 &cloudmsg)
{
    pcl::PointCloud<T> cloudresult;
    pcl::fromROSMsg(cloudmsg,cloudresult);
    return cloudresult;
}

template<typename T>
sensor_msgs::msg::PointCloud2 cloud2msg(pcl::PointCloud<T> cloud, std::string frame_id = "map")
{
    sensor_msgs::msg::PointCloud2 cloud_ROS;
    pcl::toROSMsg(cloud, cloud_ROS);
    cloud_ROS.header.frame_id = frame_id;
    return cloud_ROS;
}

void callbackNode(const patchwork::msg::Node::SharedPtr msg) {
    static int frame_idx = 0;
    cout << frame_idx << "th node come" << endl;
    pcl::PointCloud<PointType> pc_curr = cloudmsg2cloud<PointType>(msg->lidar);
    pcl::PointCloud<PointType> pc_ground;
    pcl::PointCloud<PointType> pc_non_ground;

    static double time_taken;

    cout << "Operating patchwork..." << endl;
    PatchworkGroundSeg->estimate_ground(pc_curr, pc_ground, pc_non_ground, time_taken);

    // Estimation
    double precision, recall, precision_naive, recall_naive;
    calculate_precision_recall(pc_curr, pc_ground, precision, recall);
    calculate_precision_recall(pc_curr, pc_ground, precision_naive, recall_naive, false);

    cout << "\033[1;32m" << frame_idx << "th, " << " takes : " << time_taken << " | " << pc_curr.size() << " -> " << pc_ground.size()
         << "\033[0m" << endl;

    cout << "\033[1;32m P: " << precision << " | R: " << recall << "\033[0m" << endl;

//    output_filename = "/home/shapelim/patchwork_debug.txt";
//    ofstream sc_output(output_filename, ios::app);
//    sc_output << msg->header.seq << "," << time_taken << "," << precision << "," << recall << "," << precision_naive << "," << recall_naive;
//
//    sc_output << std::endl;
//    sc_output.close();

    // Publish msg
    pcl::PointCloud<PointType> TP;
    pcl::PointCloud<PointType> FP;
    pcl::PointCloud<PointType> FN;
    pcl::PointCloud<PointType> TN;
    discern_ground(pc_ground, TP, FP);
    discern_ground(pc_non_ground, FN, TN);

    if (save_flag) {
        std::map<int, int> pc_curr_gt_counts, g_est_gt_counts;
        double             accuracy;
        save_all_accuracy(pc_curr, pc_ground, acc_filename, accuracy, pc_curr_gt_counts, g_est_gt_counts);

        std::string count_str        = std::to_string(frame_idx);
        std::string count_str_padded = std::string(NUM_ZEROS - count_str.length(), '0') + count_str;
        std::string pcd_filename     = pcd_savepath + "/" + count_str_padded + ".pcd";

//    pc2pcdfile(TP, FP, FN, TN, pcd_filename);
    }
    // Write data
    CloudPublisher->publish(cloud2msg(pc_curr));
    TPPublisher->publish(cloud2msg(TP));
    FPPublisher->publish(cloud2msg(FP));
    FNPublisher->publish(cloud2msg(FN));
    pub_score("p", precision);
    pub_score("r", recall);
    ++frame_idx;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<rclcpp::Node>("benchmark");
    g_node = nh;
    nh->declare_parameter<std::string>("algorithm", "patchwork");
    nh->declare_parameter<std::string>("seq", "00");
    nh->get_parameter("algorithm", algorithm);
    nh->get_parameter("seq", seq);

    PatchworkGroundSeg.reset(new PatchWork<PointType>(nh));

    CloudPublisher  = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/benchmark/cloud", 100);
    TPPublisher     = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/benchmark/TP", 100);
    FPPublisher     = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/benchmark/FP", 100);
    FNPublisher     = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/benchmark/FN", 100);

    PrecisionPublisher = nh->create_publisher<visualization_msgs::msg::Marker>("/precision", 1);
    RecallPublisher    = nh->create_publisher<visualization_msgs::msg::Marker>("/recall", 1);

    auto NodeSubscriber = nh->create_subscription<patchwork::msg::Node>("/registered_scan", rclcpp::QoS(5000), callbackNode);
    (void)NodeSubscriber;
    rclcpp::spin(nh);
    rclcpp::shutdown();
    return 0;
}
