//
// Created by hjl on 2021/7/30.
//

#ifndef MY_PLANNER_WS_EXPLORATION_DATA_H
#define MY_PLANNER_WS_EXPLORATION_DATA_H
#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <mutex>

#include <iostream>
#include <fstream>

#include <visualization_tools/ExploredVolumeTime.h>
#include <visualization_tools/ExploredVolumeTravedDist.h>
#include <visualization_tools/TravedDistTime.h>
#include <visualization_tools/ExploredVolumeTravedDistTime.h>
#include <visualization_tools/IterationTime.h>

#include <active_perception/frontierStatistics.h>

class exploration_data
{
public:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber odom_sub;
    ros::Subscriber map_frontiers_sub;
    ros::Subscriber iteration_time_sub;
    ros::Subscriber init_sub;
    ros::Subscriber finish_sub;

    ros::Publisher explorated_volume_traved_dist_time_pub;
    ros::Publisher iteration_time_pub;
    ros::Publisher explore_finish_pub;

    ros::Timer pub_timer;

    double path_length_sum_;
    tf::Vector3 last_position_;

    std::size_t known_cell_num_;
    std::size_t known_plane_cell_num_;
    double known_map_resolution_;
    double known_space_volume_;
    double known_plane_space_volume_;

    bool system_inited_;
    double system_init_time_;

    double max_time_;
    double max_volume_;
    bool exploration_finish_;
    double finish_time_;

    double sum_iteration_time_;
    int iteration_num_;
    int seq_;

    std::string distance_volume_txt_name_;
    std::string iteration_time_txt_name_;
    std::string trajectory_txt_name_;

    explicit exploration_data(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

    void explorationInitCallback(const std_msgs::Float64ConstPtr &init);

    void explorationFinishCallback(const std_msgs::Float64ConstPtr &finish);

    void odomCallback(const nav_msgs::OdometryConstPtr &input);

    void pubExplorationData(const ros::TimerEvent &event);

    void mapAndFrontiersCallback(const active_perception::frontierStatisticsConstPtr &msg);

    void iterationTimeCallback(const visualization_tools::IterationTimeConstPtr &msg);

    void pathFindTimeCallback(const visualization_tools::IterationTimeConstPtr &msg);
};

#endif // MY_PLANNER_WS_EXPLORATION_DATA_H
