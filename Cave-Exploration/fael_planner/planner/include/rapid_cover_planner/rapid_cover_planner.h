//
// Created by hjl on 2022/1/13.
//

#ifndef ROBO_PLANNER_WS_RAPID_COVER_PLANNER_H
#define ROBO_PLANNER_WS_RAPID_COVER_PLANNER_H

#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <visualization_tools/IterationTime.h>
#include <visualization_tools/ViewpointGain.h>

#include "preprocess/preprocess.h"
#include "tsp_solver/two_opt.h"
#include "graph/plan_graph_ikdtree.h"
#include "active_perception/frontier_finder.h"

namespace rapid_cover_planner
{
    using namespace utils;
    using namespace perception;
    using namespace preprocess;
    using fast_planner::FrontierFinder;

    typedef std::vector<utils::Point3D> Path;

    class RapidCoverPlanner
    {

    public:
        typedef std::shared_ptr<RapidCoverPlanner> Ptr;
        ros::NodeHandle nh_private_;

        ros::Publisher stop_move_pub_;

        // parameters required for planner preprocessing
        double init_x_, init_y_, init_z_;
        double stop_x_thre_, stop_y_thre_, stop_z_thre_;
        int max_tour_point_num_, planner_dim_;
        double viewpoint_ignore_thre_;
        double local_range_;
        double frontier_gain_;
        double tourpoint_ignore_distance_;
        double tourpoint_ignore_thre_;

        // data used for planner calculations
        fast_planner::FrontierFinder::Ptr frontier_finder_;
        Map2DManager::Ptr map_2d_manager_;
        Map3DManager::Ptr map_3d_manager_;
        ViewpointManager::Ptr viewpoint_manager_;
        IkdTreeTopoGraph::Ptr road_graph_;

        // data after initialization.
        graph::IkdTreePlanGraph plan_graph_;
        utils::Point3DSet tour_points_;
        utils::Point3DMap<double> tour_points_gains_;
        double max_gain_;
        std::vector<std::vector<Path>> path_matrix_;
        utils::Point3DMap<utils::Point3DMap<Path>> pre_paths_;

        // planner iterative output
        utils::Point3D init_point_, goal_point_;
        int init_point_id_;
        Path path_to_go_;
        Path tsp_path_;
        std::vector<Path> path_segments_;

        FrontierQueue goal_point_frontiers_;

        bool is_local_planning_;
        bool is_directory_;
        bool is_surface_follow_;
        double alpha_, lambda_;

        void singlePointPlanning(const geometry_msgs::Pose &current_pose, const utils::Point3D &current_directory, bool &is_successed);

        RapidCoverPlanner(ros::NodeHandle &nh_private);

        void setFrontierFinder(const FrontierFinder::Ptr &frontier_finder);

        void setMap2DManager(const Map2DManager::Ptr &map_2d_manager);

        void setMap3DManager(const Map3DManager::Ptr &map_3d_manager);

        void setViewpointManager(const ViewpointManager::Ptr &viewpoint_manager);

        void setTopoGraph(const IkdTreeTopoGraph::Ptr &road_graph);

        void setPlannerDim(const int &dim);

        void setParamsFromRos();

        void Initialize(const Point3D &current_position);

        void viewpointsFrontierGain(utils::Point3DSet &viewpoints, utils::Point3DMap<double> &tour_points_gains, double &max_gain);

        void planning(const geometry_msgs::Pose &current_pose, const utils::Point3D &current_directory, bool &is_successed);

        void findPathBackToOrigin(const geometry_msgs::Pose &current_pose);

        void findPathToSingleViewpoint(const geometry_msgs::Pose &current_pose, const geometry_msgs::Pose &viewpoint);

        bool two_opt_solve_planning(const geometry_msgs::Pose &current_pose, const utils::Point3D &current_directory);

        int addCurrentPositionToGraph(const Point3D &current_position);

        Path getPathFromInflateRegion(const Point3D &start_point, const Point3D &end_point);

        Path getPathInGraph(const int &start_point_id, const int &end_point_id);

        Path getPathInGridMap2D(const Point3D &start_point, const Point3D &end_point, const bool &is_clear_nearby);

        Path getPathInGridMap3D(const Point3D &start_point, const Point3D &end_point, const bool &is_clear_nearby);

        bool findClosestVirtualObstacle(const Point3D &current_position, Point3D &target);

        // debug
        void pubViewpointWithFrontiers(const Vector3d &target_viewpoint, const vector<int> &attach_frontiers);
        void pubViewpointWithGain();
        void pubPath(Path &path);

    private:
        // debug
        ros::Publisher viewpoint_with_path_pub_, viewpoints_gain_pub_, path_pub_;
        std::vector<Vector3d> viewpoint_with_path_;
        std::vector<double> viewpoints_gain_;

        ros::Publisher viewpoint_with_frontiers_pub_;

        // statistic
        std::string each_solving_txt_name_;
        double sum_solving_time_;
        int solving_num_;

        std::string each_tourpoints_initilize_txt_name_;
        double sum_initilize_time_;
        int initilize_num_;

        std::string two_opt_time_name_;
        double sum_two_opt_time;

        std::string frontier_gain_time_txt_;
        double sum_fontier_gains_time_;
        int fontier_gains_num_;

        std::vector<double> frontiers_gains_;
        std::vector<double> unmapped_gains_;
        std::vector<double> mean_errors_;
    };
}

#endif // ROBO_PLANNER_WS_RAPID_COVER_PLANNER_H
