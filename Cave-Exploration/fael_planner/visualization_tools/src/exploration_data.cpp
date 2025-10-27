//
// Created by hjl on 2021/7/30.
//

#include "exploration_data.h"
#include <boost/filesystem.hpp>
#include "file_utils/file_rw.h"
#include "time_utils/time_utils.h"

namespace fs = boost::filesystem;

exploration_data::exploration_data(const ros::NodeHandle &nh,
                                   const ros::NodeHandle &nh_private) : nh_(nh), nh_private_(nh_private), path_length_sum_(0.0),
                                                                        sum_iteration_time_(0), iteration_num_(0), last_position_(0, 0, 0),
                                                                        known_map_resolution_(0), known_cell_num_(0), known_plane_cell_num_(0),
                                                                        known_space_volume_(0), known_plane_space_volume_(0),
                                                                        system_inited_(false), exploration_finish_(false), seq_(0)
{

    init_sub = nh_.subscribe<std_msgs::Float64>("explorer_start", 1, &exploration_data::explorationInitCallback, this);      // 接收开始时间
    finish_sub = nh_.subscribe<std_msgs::Float64>("explorer_finish", 1, &exploration_data::explorationFinishCallback, this); // 接收结束时间
    odom_sub = nh_.subscribe<nav_msgs::Odometry>("odometry", 1, &exploration_data::odomCallback, this);                      // 里程计信息
    map_frontiers_sub = nh_.subscribe<active_perception::frontierStatistics>("SDFMap_frontier_statistics", 1,
                                                                             &exploration_data::mapAndFrontiersCallback,
                                                                             this);
    iteration_time_sub = nh_.subscribe<visualization_tools::IterationTime>("iteration_time", 1,
                                                                           &exploration_data::iterationTimeCallback,
                                                                           this);

    explorated_volume_traved_dist_time_pub = nh_.advertise<visualization_tools::ExploredVolumeTravedDistTime>(
        "/exploration_data/explored_volume_traved_dist_time", 1);

    iteration_time_pub = nh_.advertise<visualization_tools::IterationTime>("/exploration_data/run_time", 1);
    explore_finish_pub = nh_.advertise<std_msgs::Bool>("/exploration_data/exploration_data_finish", 1); // 探索面积达到一定程度发布结束信号

    pub_timer = nh_private_.createTimer(ros::Duration(1.0), &exploration_data::pubExplorationData, this);

    const std::string &ns = ros::this_node::getName();

    max_volume_ = 100000000.0;
    if (!ros::param::get(ns + "/map_area", max_volume_))
    {
        ROS_WARN("No map_area specified. Looking for %s. Default is 100000000", (ns + "/map_area").c_str());
    }

    max_time_ = 100000000.0;
    if (!ros::param::get(ns + "/max_time", max_time_))
    {
        ROS_WARN("No max_time specified. Looking for %s. Default is 100000000", (ns + "/max_time").c_str());
    }

    std::string pkg_path = ros::package::getPath("visualization_tools");
    std::string dir_path = pkg_path + "/../../exploration_data_files/visualization_tools";
    distance_volume_txt_name_ = dir_path + "/time_distance_volume.txt";
    iteration_time_txt_name_ = dir_path + "/iteration_time.txt";
    trajectory_txt_name_ = dir_path + "/trajectory.txt";

    file_utils::createDirectory(dir_path);

    file_utils::writeToFileByTrunc(distance_volume_txt_name_, std::string("\t"),
                                   "explored_time", "distance/path-length", "explored_plane_volume(m2)",
                                   "explored_volume(m3)", "known_plane_cell_num", "known_voxel_num(3d)");
    file_utils::writeToFileByTrunc(iteration_time_txt_name_, std::string("\t"),
                                   "explored_time", "iteration_time", "iteration_num", "average_time(s)");
    file_utils::writeToFileByTrunc(trajectory_txt_name_, std::string("\t"),
                                   "explored_time", "x", "y", "z");
}

void exploration_data::explorationInitCallback(const std_msgs::Float64ConstPtr &init)
{
    if (!system_inited_)
    {
        system_init_time_ = init->data;
        system_inited_ = true;
    }
    ROS_INFO("visualization statics start.");
}

void exploration_data::explorationFinishCallback(const std_msgs::Float64ConstPtr &finish)
{

    exploration_finish_ = true;
    finish_time_ = finish->data;
}

void exploration_data::odomCallback(const nav_msgs::OdometryConstPtr &input)
{

    tf::Vector3 current_position(input->pose.pose.position.x, input->pose.pose.position.y, input->pose.pose.position.z);
    double dist_delta = current_position.distance(last_position_);
    last_position_ = current_position;

    if (system_inited_)
    {
        path_length_sum_ += dist_delta;
    }
}

void exploration_data::mapAndFrontiersCallback(const active_perception::frontierStatisticsConstPtr &msg)
{
    known_map_resolution_ = msg->resolution;
    known_cell_num_ = msg->known_cell_num;
    known_plane_cell_num_ = msg->known_plane_cell_num;
    known_space_volume_ =
        static_cast<double>(known_cell_num_) * known_map_resolution_ * known_map_resolution_ * known_map_resolution_;
    known_plane_space_volume_ =
        static_cast<double>(known_plane_cell_num_) * known_map_resolution_ * known_map_resolution_;
}

void exploration_data::iterationTimeCallback(const visualization_tools::IterationTimeConstPtr &msg)
{
    if (system_inited_ && !exploration_finish_)
    {

        visualization_tools::IterationTime time;
        time.timeConsumed = msg->current_time - system_init_time_;
        time.iterationTime = msg->iterationTime;
        iteration_time_pub.publish(time);

        sum_iteration_time_ += time.iterationTime;
        iteration_num_++;

        file_utils::writeToFileByAdd(iteration_time_txt_name_, "\t", time.timeConsumed,
                                     time.iterationTime, iteration_num_, sum_iteration_time_ / iteration_num_);
    }
}

void exploration_data::pubExplorationData(const ros::TimerEvent &event)
{

    if (system_inited_ && !exploration_finish_)
    {
        visualization_tools::ExploredVolumeTravedDistTime exploration_data;
        exploration_data.header.stamp = ros::Time::now();
        exploration_data.header.frame_id = "world";
        exploration_data.header.seq = ++seq_;

        // exploration_data.exploredPlaneVolume = known_plane_space_volume_;
        exploration_data.exploredPlaneVolume = 0;
        // exploration_data.exploredVolume = known_space_volume_;
        exploration_data.exploredVolume = 0;
        exploration_data.travedDist = path_length_sum_;
        double time_second = static_cast<double>(time_utils::Timer::GetTimeNow("us")) / 1e6;
        exploration_data.timeConsumed = time_second - system_init_time_;

        // if (exploration_data.exploredPlaneVolume > 0.95 * max_volume_)
        // {
        //     exploration_finish_ = true;
        //     std_msgs::Bool explore_finish;
        //     explore_finish.data = true;
        //     explore_finish_pub.publish(explore_finish);
        //     ROS_INFO("max volume threshold is reach...");
        // }

        // if (time_second - system_init_time_ > max_time_)
        // {
        //     exploration_finish_ = true;
        //     std_msgs::Bool explore_finish;
        //     explore_finish.data = true;
        //     explore_finish_pub.publish(explore_finish);
        //     ROS_INFO("max time is reach...");
        // }

        explorated_volume_traved_dist_time_pub.publish(exploration_data);

        file_utils::writeToFileByAdd(distance_volume_txt_name_, "\t",
                                     exploration_data.timeConsumed, exploration_data.travedDist,
                                     exploration_data.exploredPlaneVolume, exploration_data.exploredVolume,
                                     known_plane_cell_num_, known_cell_num_);

        file_utils::writeToFileByAdd(trajectory_txt_name_, "\t", exploration_data.timeConsumed,
                                     last_position_.x(), last_position_.y(), last_position_.z());
    }

    if (exploration_finish_)
    {
        std_msgs::Bool explore_finish;
        explore_finish.data = true;
        explore_finish_pub.publish(explore_finish);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "exploration_data");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    exploration_data data(nh, nh_private);
    ros::spin();

    return 0;
}
