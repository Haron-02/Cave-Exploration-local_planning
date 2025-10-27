//
// Created by hjl on 2022/1/10.
//

#ifndef ROBO_PLANNER_WS_PREPROCESS_H
#define ROBO_PLANNER_WS_PREPROCESS_H

#include <ros/ros.h>
#include <thread>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include "active_perception/frontier_finder.h"
#include "perception2grids/map_2d_manager.h"
#include "perception3d/map_3d_manager.h"
#include "preprocess/viewpoint_manager.h"
#include "preprocess/topo_graph_ikdtree.h"
#include "plan_env/sdf_map.h"

#include "file_utils/file_rw.h"
#include "time_utils/time_utils.h"

namespace preprocess
{
    using fast_planner::FrontierFinder;
    using fast_planner::SDFMap;

    class Preprocess
    {
    public:
        typedef std::shared_ptr<Preprocess> Ptr;

        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        int perception_dim_;

        // odometry
        ros::Subscriber odom_sub_;
        utils::Point3D current_position_;
        geometry_msgs::Pose current_pose_;

        utils::Point3D last_directory_position_;
        utils::Point3D forward_directory_;

        bool is_explorer_initilized_;

        // dynamic updating
        SDFMap::Ptr sdf_map_;
        FrontierFinder::Ptr frontier_finder_;
        perception::Map2DManager::Ptr map_2d_manager_;
        perception::Map3DManager::Ptr map_3d_manager_;
        preprocess::ViewpointManager::Ptr viewpoint_manager_;
        preprocess::IkdTreeTopoGraph::Ptr road_map_;

        ros::Timer preprocess_update_timer;
        
        // mutex
        std::mutex elements_update_mutex_;

        Preprocess(ros::NodeHandle &nh_private){};
        Preprocess(ros::NodeHandle &nh_private, SDFMap::Ptr &sdf_map) : nh_private_(nh_private),
                                                                        is_explorer_initilized_(false)
        {
            perception_dim_ = 3;

            sdf_map_ = sdf_map;
            odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/odom_world", 1, &Preprocess::odomCallback, this);

            init(nh_private);

            preprocess_update_timer = nh_private.createTimer(ros::Duration(0.10), &Preprocess::preprocessUpdateCallback, this);
        };

        void init(ros::NodeHandle &nh_private)
        {
            frontier_finder_ = std::make_shared<FrontierFinder>(sdf_map_, nh_private);
            
            viewpoint_manager_ = std::make_shared<preprocess::ViewpointManager>(nh_private, frontier_finder_);
            Eigen::Vector3d bmin, bmax;
            sdf_map_->getBox(bmin, bmax);
            viewpoint_manager_->setViewPointsBound(bmax, bmin);

            road_map_ = std::make_shared<preprocess::IkdTreeTopoGraph>(nh_private);
            road_map_->setViewpointManager(viewpoint_manager_);
            road_map_->setSamplePointsBound(bmax, bmin);

            frontier_finder_->setRoadMap(road_map_);

            map_3d_manager_ = std::make_shared<perception::Map3DManager>();
            map_3d_manager_->initMap3DManager(nh_private, sdf_map_);

            frontier_finder_->setMap3DManager(map_3d_manager_);
            viewpoint_manager_->setMap3DManager(map_3d_manager_);
            road_map_->setMap3DManager(map_3d_manager_);
        };

        void odomCallback(const nav_msgs::OdometryConstPtr &odom)
        {
            current_pose_ = odom->pose.pose;
            current_position_.x() = odom->pose.pose.position.x;
            current_position_.y() = odom->pose.pose.position.y;
            current_position_.z() = odom->pose.pose.position.z;

            frontier_finder_->setCurrentPose(current_pose_);

            map_3d_manager_->setCurrentPose(current_pose_);
   
            viewpoint_manager_->setCurrentPosition(current_position_);
            road_map_->setCurrentPosition(current_position_);

            forward_directory_[0] = odom->twist.twist.linear.x;
            forward_directory_[1] = odom->twist.twist.linear.y;
            forward_directory_[2] = odom->twist.twist.linear.z;
            Vector3d forward_directory(forward_directory_[0], forward_directory_[1], forward_directory_[2]);
            frontier_finder_->updateForwardDirectory(forward_directory);
        };

        void preprocessUpdateCallback(const ros::TimerEvent &event)
        {
            updateElements();
        };

        void updateElements()
        {
            // elements_update_mutex_.lock();
            // 更新local_grid_map
            map_3d_manager_->updateGridMap3D();

            frontier_finder_->simpleSearchFrontiers();
    
            is_explorer_initilized_ = true;
            if (is_explorer_initilized_)
            {
                // 更新viewpoints
                viewpoint_manager_->updateViewpoints(perception_dim_);
            }
            viewpoint_manager_->clearWorthlessViewPoints(perception_dim_);

            // 更新road_map
            road_map_->updateTopoGraphByMapAndViewpoints(perception_dim_);

            // elements_update_mutex_.unlock();
            // 可视化
            thread vis_thread(&Preprocess::pubPreprocessMarkers, this);
            vis_thread.detach();
        };

        void pubPreprocessMarkers()
        {
            map_3d_manager_->publishVisionMarkers();
            
            frontier_finder_->pubFrontiersMarkers();
            frontier_finder_->pubObstacleClustersMarkers();
            frontier_finder_->pubViewPoints();
            frontier_finder_->pubSurfaceFrontierClustersMarkers();
            frontier_finder_->pubOccupancyCloud();
            frontier_finder_->pubSurfaceFrontierNormalsMarkers();
            frontier_finder_->pubGlobalPathMarkers();
            frontier_finder_->pubViewpointWithFrontiers();
            frontier_finder_->pubEndVels();
            frontier_finder_->pubCurrentDirection();
            frontier_finder_->pubRoadMapPath();
            frontier_finder_->pubMergedFrontiers();
            frontier_finder_->pubClusterRelation();
            
            road_map_->pubGraphMarkers();
            viewpoint_manager_->pubMarkers();
        }
    };
}

#endif // ROBO_PLANNER_WS_PREPROCESS_H
