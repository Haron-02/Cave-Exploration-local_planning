//
// Created by hjl on 2021/12/15.
//

#ifndef TOPO_PLANNER_WS_PATH_EXECUTION_H
#define TOPO_PLANNER_WS_PATH_EXECUTION_H

#include <Eigen/Eigen>
#include <iostream>
#include <memory>
#include <set>
#include <map>
#include <vector>
#include <queue>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/common/transforms.h>
#include <visualization_msgs/MarkerArray.h>

#include <actionlib/server/simple_action_server.h>
#include <control_planner_interface/VehicleExecuteAction.h>

#include "perception2grids/map_2d_manager.h"
#include "perception3d/map_3d_manager.h"

namespace path_execution
{
    using namespace std;
    using namespace perception;
    typedef std::vector<geometry_msgs::Pose> Path;
    typedef std::vector<Point2D> Path2D;

    class PathExecution
    {

    public:
        typedef std::shared_ptr<PathExecution> Ptr;
        typedef actionlib::SimpleActionServer<control_planner_interface::VehicleExecuteAction> JackalExecuteActionServer;

        PathExecution(const ros::NodeHandle &nh_private);

        void setMap2DManager(Map2DManager::Ptr &map2d_manager);

        void setMap3DManager(Map3DManager::Ptr &map3d_manager);

        void setPlannerDim(const int &planner_dim);

        void odomCallback(const nav_msgs::OdometryConstPtr &odom);

        void executeCallback(const control_planner_interface::VehicleExecuteGoalConstPtr &executed_path);

        void findLlookAheadPoint(const std::vector<control_planner_interface::Path> &path_segments, bool &success,
                                 double &executed_start_time, double &executed_path_length, geometry_msgs::Pose &goal_pose);

        void lookAheadPointControlLoop(bool &success, double &executed_start_time,
                                       double &executed_path_length, geometry_msgs::Pose &goal_pose);

        void stopMoveCallback(const control_planner_interface::VehicleExecuteGoalConstPtr &stop_goal);

        bool setPathToFollow(const std::vector<control_planner_interface::Path> &path_segments);

        Path interpolatePath(const Path &path) const;

        std::vector<Point2D> makeLocalPlan2D(geometry_msgs::Pose &goal, geometry_msgs::Pose &start, const bool &is_clear_nearby);

        std::vector<GridPoint3D> makeLocalPlan3D(geometry_msgs::Pose &goal, geometry_msgs::Pose &start, const bool &is_clear_nearby);

        Path generatePath(std::vector<Point2D> &path_2d);

        Path generatePath(std::vector<GridPoint3D> &path_3d);

        bool isStopped();

        void publishPath(const Path &path, const ros::Publisher &pub);

        visualization_msgs::MarkerArray generatePath2DListMarker(const std::vector<Path2D> &path_list) const;
        visualization_msgs::Marker generateSmoothPath2DMarker(const Path2D &path) const;

        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        JackalExecuteActionServer execute_server_;
        JackalExecuteActionServer stop_move_server_;

        ros::Subscriber odom_sub_;

        ros::Publisher local_path_pub_;
        ros::Publisher way_pose_pub_;
        ros::Publisher executed_path_pub_;
        ros::Publisher smooth_path_pub_;
        ros::Publisher look_ahead_goal_pub_;

        std::string global_frame_;
        std::string local_frame_;

        Map2DManager::Ptr map2d_manager_;
        Map3DManager::Ptr map3d_manager_;
        double grid_size_;
        double init_z_;
        int planner_dim_;

        geometry_msgs::Pose current_pose_;
        geometry_msgs::Pose last_pose_;

        nav_msgs::Odometry current_odom_;
        nav_msgs::Odometry last_odom_;

        Path global_path_to_follow_;
        std::vector<Path> path_segments_;
        int current_waypose_id_;

        double reach_dist_thres_;
        double reach_rot_thres_;
        double stop_vel_thres_;
        double stop_rot_thres_;
        double local_path_dist_;
    };

}

#endif // TOPO_PLANNER_WS_PATH_EXECUTION_H
