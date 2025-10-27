//
// Created by hjl on 2022/1/11.
//

#ifndef ROBO_PLANNER_WS_MAP_2D_MANAGER_H
#define ROBO_PLANNER_WS_MAP_2D_MANAGER_H

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
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/extract_indices.h>

#include <pcl_ros/impl/transforms.hpp>
#include <pcl/common/common.h>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <unordered_map>

#include "grid_map_2d.h"
#include "plan_env/sdf_map.h"


namespace perception
{
    struct Map2DParams
    {
        std::string frame_id_;
        double init_z_;
        double grid_size_;
        Eigen::Vector2d map_length_;
        Eigen::Vector2i map_grid_num_;
        double inflate_radius_;
    };

    struct Map2DBoundary
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
        vector<pcl::PointXYZ> surface_frontiers_;
        vector<pcl::PointCloud<pcl::PointXYZ>> surface_frontiers_cloud_;
        vector<Eigen::Vector3d> normals_;
        vector<Index2D> indices_;
    };

    struct ObstacleCluster
    {
        // unordered_set<Index2D> cells_;
        vector<Point2D> angle_status_;
        // ObstacleCluster();
    };


    class Map2DManager
    {
    public:
        typedef std::shared_ptr<Map2DManager> Ptr;

        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        ros::Subscriber odom_sub_, update_map2d_sub_;
        ros::Publisher grid_map_2d_pub_, refine_grid_map_2d_pub_, surface_frontiers_pub_, surface_frontier_normals_pub_, occupancy_cloud_pub_, angle_status_pub_;
        ros::Publisher detect_angle_pub_, forward_directory_pub_, circle_pub_, cluster_pub_, angle_status_pub2_, virtual_wall_pub_;
        tf::TransformListener tf_listener_;
        geometry_msgs::Pose current_pose_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr global_frontier_cloud_;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr global_frontier_kdtree_;
        fast_planner::SDFMap::Ptr sdf_map_;

        mutable std::shared_timed_mutex map_2d_mutex_;

        Map2DParams m2dp_;
        bool is_map_updated_;
        Point2D forward_directory_;

        Map2DManager();
        void initMap2DManager(ros::NodeHandle &nh_private, fast_planner::SDFMap::Ptr &sdf_map);
        void getParamsFromRos();
        void setCurrentPose(const geometry_msgs::Pose &current_pose);

        void updateGridMap2D();
        void updateGridStatus();
        void updateSurrandObstacles();
        void obstaclesCluster();
        void findObstaclesClusterToFollow();
        
        void publishVisionMarkers();
        void publishRefineVisionMarkers();
        void publishSurfaceFrntiersMarkers();
        void publishSurfaceFrntierNormalsMarkers();
        void publishObstacleClustersMarkers();
        void publishAngleStatusMarkers();
        void publishVirtualWall();

        bool isCollisionFreeStraight(const Point2D &source, const Point2D &target) const;
        bool getShortestPath(const Point2D &goal, const Point2D &start, std::vector<Point2D> &path2d) const;
        std::vector<Point2D> optimalToStraight(std::vector<Point2D> &path) const;
        bool getShortestOptimalPath(const Point2D &goal, const Point2D &start,
                                    const bool &is_clear_nearby, std::vector<Point2D> &optimal_path2d);

        bool isFree(const Point2D &point);
        bool isOccupied(const Point2D &point);
        bool isUnknown(const Point2D &point);

        bool isNearFree(const Point2D &point, const double &range);
        bool isNearOccupied(const Point2D &point, const double &range);
        bool isNearUnknown(const Point2D &point, const double &range);

        bool isDetect();
        Eigen::Vector3d getAnglePos();

    private:
        GridMap2D inflate_map_;
        GridMap2D map_;
        GridMap2D refine_map_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr surface_frontiers_;
        Map2DBoundary boundary_;
        vector<Point2D> angle_status_;
        vector<ObstacleCluster> obstacle_clusters;
        int start_angle_index_;
        unordered_map<int, int> cell_to_cluster_;
        int angle_index_;
        int cluster_to_follow_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles_cloud_;
        vector<pcl::PointIndices> obstacle_clustered_indices_;
        bool detect_;
        void clearRobotNeighbor(const Point2D &center_point, double clear_radius = 0.0);
    };

}

#endif // ROBO_PLANNER_WS_MAP_2D_MANAGER_H
