//
// Created by hjl on 2022/1/10.
//

#ifndef ROBO_PLANNER_WS_VIEWPOINT_MANAGER_H
#define ROBO_PLANNER_WS_VIEWPOINT_MANAGER_H

#include <ros/ros.h>
#include <ros/package.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/common/common.h>

#include <utils/viewpoint.h>
#include <utils/frontier.h>
#include <perception2grids/map_2d_manager.h>
#include <perception3d/map_3d_manager.h>
#include "active_perception/frontier_finder.h"
#include "utils/ikd_Tree.h"

namespace fast_planner
{
    class FrontierFinder;
    // typedef shared_ptr<FrontierFinder> Ptr;
}

namespace preprocess
{
    
    using namespace utils;
    using namespace perception;
    // using namespace fast_planner;
    using fast_planner::FrontierFinder;
    
    using PointType = ikdTree_PointType;
    using PointVector = KD_TREE<PointType>::PointVector;


    struct ViewpointManagerParams
    {
        std::string frame_id_;
        double init_z_;
        double sample_dist_h_;
        double sample_dist_v_;
        double points_dist_thre_h_;
        double points_dist_thre_v_;
        double points_dist_to_occupied_;
        double points_dist_to_unknown_;
        double points_clear_dist_thre_h_;
        double points_clear_dist_thre_v_;
        double bound_margin_h_;
        double bound_margin_v_;
        double frontier_viewpoint_dist_thre_;
        double viewpoint_gain_thre_;
        Eigen::Vector3d viewpoint_max_bound_;
        Eigen::Vector3d viewpoint_min_bound_;
        double lower_fov_;
        double upper_fov_;
    };

    class ViewpointManager
    {
    public:
        typedef std::shared_ptr<ViewpointManager> Ptr;

        // output
        ViewpointMap<FrontierQueue> viewpoints_attached_frontiers_; // store each representative point and which frontier it represents
        ViewpointSet candidate_viewpoints_;                         // store those frontier representative points

        // params
        ViewpointManagerParams vmp_;

        ViewpointManager(const ros::NodeHandle &nh_private, const std::shared_ptr<FrontierFinder> &frontier_finder);
        void getParamsFromRos();

        void setCurrentPosition(const utils::Point3D &current_position);
        void setMap2DManager(const Map2DManager::Ptr &map_2d_manager);
        void setMap3DManager(const Map3DManager::Ptr &map_3d_manager);

        void setViewPointsBound(const Eigen::Vector3d &max_bound, const Eigen::Vector3d &min_bound);
        void updateViewpoints(const int &perception_dim);
        void frontierAttachInSDFMap(const int &perception_dim);
        void clearWorthlessViewPoints(const int &perception_dim);
        ViewpointQueue samplePointsInGridMap2D();
        ViewpointQueue samplePointsInGridMap3D();
        bool getPointId(const utils::Point3D &point, int &point_id);

        void pubMarkers() const;
        visualization_msgs::MarkerArray generatePointsMarkers(const Point3DSet &sample_points) const;
        visualization_msgs::MarkerArray
        generateViewPointsWithFrontiersMarkers(const ViewpointMap<FrontierQueue> &viewpoints_attached_frontiers_) const;

    private:
        ros::NodeHandle nh_private_;

        ros::Publisher repre_points_pub_;
        ros::Publisher frontiers_viewpoints_pub_;

        std::shared_ptr<FrontierFinder> frontier_finder_;
        Map2DManager::Ptr map_2d_manager_;
        Map3DManager::Ptr map_3d_manager_;

        KD_TREE<PointType>::Ptr viewpoints_ikdtree_;
        utils::Point3D current_position_;

        Point3DSet local_points_;
        Point3DSet representative_points_;
        FrontierMap<Viewpoint> frontiers_viewpoints_; // store each boundary with the corresponding secondary point
        utils::Point3DMap<int> points_indices_;
        std::vector<utils::Point3D> points_vector_;
    };

}

#endif // ROBO_PLANNER_WS_VIEWPOINT_MANAGER_H
