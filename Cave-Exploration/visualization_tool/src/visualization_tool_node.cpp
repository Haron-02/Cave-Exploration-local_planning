#include <ros/ros.h>
#include <ros/package.h>
#include <ros/time.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <std_msgs/Float64.h>
#include <string>

#include "time_utils/time_utils.h"
#include "file_utils/file_rw.h"

// 全局变量
ros::Publisher covered_cloud_pub, downsample_scene_pub, coverage_rate_pub; 
pcl::PointCloud<pcl::PointXYZ>::Ptr covered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud(new pcl::PointCloud<pcl::PointXYZ>());
bool receive_scene = false;
std::string map_path, coverage_rate_txt;

// 点云回调函数
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    *covered_cloud += cloud;

    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(covered_cloud);
    voxel_filter.setLeafSize(0.4f, 0.4f, 0.4f); // 调整分辨率
    voxel_filter.filter(*covered_cloud);
}

// 定时器回调函数：定期发布累积点云
void pubCallback(const ros::TimerEvent& e)
{
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*covered_cloud, msg);
    msg.header.frame_id = "world";  // 设置点云的参考坐标系
    msg.header.stamp = ros::Time::now();

    // 发布累积点云
    covered_cloud_pub.publish(msg);

    std_msgs::Float64 rate_msg;
    rate_msg.data = static_cast<double>(covered_cloud->points.size())/scene_cloud->points.size();
    coverage_rate_pub.publish(rate_msg);
    
    file_utils::writeToFileByAdd(coverage_rate_txt, "\t",
                                 rate_msg.data,
                                 covered_cloud->points.size(),
                                 ros::Time::now()); 
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "coverage_rate");
    ros::NodeHandle nh;

    ros::Subscriber cloud_sub = nh.subscribe("/sdf_map/local_cloud", 10, cloudCallback);
    // /quad0_pcl_render_node/cloud

    covered_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/covered_cloud", 1);
    downsample_scene_pub = nh.advertise<sensor_msgs::PointCloud2>("/downsample_scene_cloud", 1);
    coverage_rate_pub = nh.advertise<std_msgs::Float64>("/statistics/coverage_rate", 1);

    ros::Timer timer = nh.createTimer(ros::Duration(0.3), pubCallback);

    nh.param("/coverage_rate/map_name", map_path,  std::string("default_value"));

    std::string start_time, scene;
    nh.param("/coverage_rate/start_time", start_time, std::string("null"));
    nh.param("/coverage_rate/scene", scene, std::string("null"));

    std::time_t time = static_cast<std::time_t>(std::stoll(start_time));
    std::tm* local_time = std::localtime(&time);
    std::ostringstream oss;
    oss << std::put_time(local_time, "%Y_%m_%d-%H_%M_%S");
    start_time =  oss.str();

    std::string pkg_path = ros::package::getPath("exploration_manager");
    std::string dir_path = pkg_path + "/../../exploration_data_files/" + scene + "/" + start_time;
    file_utils::createDirectory(dir_path);
    
    coverage_rate_txt = dir_path + "/coverage_rate.txt";
    file_utils::writeToFileByTrunc(coverage_rate_txt, std::string("\t"),
      "coverage_rate", "point_num", "cur_time");


    if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path, *scene_cloud) == -1) 
    {
        ROS_ERROR("Failed to load PCD file: %s", map_path.c_str());
        return 0;
    }

    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(scene_cloud);
    voxel_filter.setLeafSize(0.4f, 0.4f, 0.4f); // 调整分辨率
    voxel_filter.filter(*scene_cloud);
    ROS_WARN("scene_cloud size:%d", scene_cloud->points.size());

    ros::Duration(5.0).sleep();
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*scene_cloud, msg);
    msg.header.frame_id = "world";  
    msg.header.stamp = ros::Time::now();
    downsample_scene_pub.publish(msg);
    receive_scene = true;

    // 进入 ROS 循环
    ros::spin();

    return 0;
}