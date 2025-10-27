//
// Created by hjl on 2022/1/11.
//

#ifndef ROBO_PLANNER_WS_MAP_3D_MANAGER_H
#define ROBO_PLANNER_WS_MAP_3D_MANAGER_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/common/common.h>
#include <memory>
#include <eigen3/Eigen/Core>
#include <shared_mutex>

#include "grid_map_3d.h"
#include "plan_env/sdf_map.h"

namespace perception
{
    struct Map3DParams
    {
        std::string frame_id_;
        double init_z_;
        double grid_size_;
        Eigen::Vector3d map_length_;
        Eigen::Vector3i map_grid_num_;
        double inflate_radius_;
    };

    class Map3DManager
    {
    public:
        typedef std::shared_ptr<Map3DManager> Ptr;

        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        ros::Subscriber odom_sub_, update_map3d_sub_;
        ros::Publisher grid_map_3d_pub_;
        tf::TransformListener tf_listener_;

        geometry_msgs::Pose current_pose_;

        fast_planner::SDFMap::Ptr sdf_map_;

        Map3DParams m3dp_;
        bool is_map_updated_;

        mutable std::shared_timed_mutex map_3d_mutex_;

        Map3DManager();
        void initMap3DManager(ros::NodeHandle &nh_private, fast_planner::SDFMap::Ptr &sdf_map);
        void getParamsFromRos();
        void setCurrentPose(const geometry_msgs::Pose &current_pose);

        void updateGridMap3D();
        void publishVisionMarkers();

        bool isCollisionFreeStraight(const GridPoint3D &source, const GridPoint3D &target) const;
        bool getShortestPath(const GridPoint3D &goal, const GridPoint3D &start, const bool &is_clear_nearby, std::vector<GridPoint3D> &path3d);
        std::vector<GridPoint3D> optimalToStraight(std::vector<GridPoint3D> &path) const;
        bool getShortestOptimalPath(const GridPoint3D &goal, const GridPoint3D &start,
                                    const bool &is_clear_nearby, std::vector<GridPoint3D> &optimal_path3d);

        bool isFree(const GridPoint3D &point);
        bool isOccupied(const GridPoint3D &point);
        bool isUnknown(const GridPoint3D &point);

        bool isNearFree(const GridPoint3D &point, const double &range);
        bool isNearOccupied(const GridPoint3D &point, const double &range);
        bool isNearUnknown(const GridPoint3D &point, const double &range);

    private:
        GridMap3D inflate_map_;
        GridMap3D map_;

        void clearRobotNeighbor(const GridPoint3D &center_point, double clear_radius);
    };

}

#endif // ROBO_PLANNER_WS_MAP_3D_MANAGER_H
