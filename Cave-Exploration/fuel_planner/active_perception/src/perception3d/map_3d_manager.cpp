//
// Created by hjl on 2022/1/11.
//

#include "perception3d/map_3d_manager.h"

namespace perception
{
    Map3DManager::Map3DManager()
    {
    }

    void Map3DManager::initMap3DManager(ros::NodeHandle &nh_private, fast_planner::SDFMap::Ptr &sdf_map)
    {
        this->nh_private_ = nh_private;
        this->is_map_updated_ = false;
        this->sdf_map_ = sdf_map;
        getParamsFromRos();
        map_.initialize(m3dp_.grid_size_, m3dp_.map_length_[0], m3dp_.map_length_[1], m3dp_.map_length_[2], Status3D::Unknown_3D);
        m3dp_.map_grid_num_ = map_.getMapGridNum();
        grid_map_3d_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("/topo_planner/gird_map_3d", 5);
    }

    void Map3DManager::getParamsFromRos()
    {
        std::string ns = ros::this_node::getName() + "/GridMap3D";

        m3dp_.frame_id_ = std::string("world");
        if (!ros::param::get(ns + "/frame_id", m3dp_.frame_id_))
        {
            ROS_WARN("No frame_id specified. Looking for %s. Default is 'world'.",
                     (ns + "/frame_id").c_str());
        }

        m3dp_.init_z_ = 1.0;
        if (!ros::param::get(ns + "/init_z", m3dp_.init_z_))
        {
            ROS_WARN("No init_z specified. Looking for %s. Default is '1.0'.",
                     (ns + "/init_z").c_str());
        }

        m3dp_.grid_size_ = 0.2;
        if (!ros::param::get(ns + "/grid_size", m3dp_.grid_size_))
        {
            ROS_WARN("No grid_size specified. Looking for %s. Default is '0.2'.",
                     (ns + "/grid_size").c_str());
        }

        m3dp_.map_length_[0] = 10;
        if (!ros::param::get(ns + "/map_size_x", m3dp_.map_length_[0]))
        {
            ROS_WARN("No map_size_x specified. Looking for %s. Default is '10'.",
                     (ns + "/map_size_x").c_str());
        }

        m3dp_.map_length_[1] = 10;
        if (!ros::param::get(ns + "/map_size_y", m3dp_.map_length_[1]))
        {
            ROS_WARN("No map_size_y specified. Looking for %s. Default is '10'.",
                     (ns + "/map_size_y").c_str());
        }

        m3dp_.map_length_[2] = 10;
        if (!ros::param::get(ns + "/map_size_z", m3dp_.map_length_[2]))
        {
            ROS_WARN("No map_size_z specified. Looking for %s. Default is '10'.",
                     (ns + "/map_size_z").c_str());
        }

        m3dp_.inflate_radius_ = sdf_map_->getInflateRadius();
    }

    void Map3DManager::setCurrentPose(const geometry_msgs::Pose &current_pose)
    {
        current_pose_ = current_pose;
    }

    void Map3DManager::updateGridMap3D()
    {
        // sdf_map_->sdf_mutex_.lock();
        std::unique_lock<std::shared_timed_mutex> lock(map_3d_mutex_);

        map_.resetMapStatus(Status3D::Unknown_3D);

        Eigen::Vector3d center_point(current_pose_.position.x, current_pose_.position.y, current_pose_.position.z);
        map_.setMapCenterAndBoundary(center_point);
        GridPoint3D grid_center3D_;
        for (int i = 0; i < m3dp_.map_grid_num_[0]; ++i)
        {
            for (int j = 0; j < m3dp_.map_grid_num_[1]; ++j)
            {
                for (int k = 0; k < m3dp_.map_grid_num_[2]; ++k)
                {
                    grid_center3D_ = map_.getGridCenter(i, j, k);
                    Eigen::Vector3d grid_center3D(grid_center3D_.x(), grid_center3D_.y(), grid_center3D_.z());
                    if (sdf_map_->isInBox(grid_center3D))
                    {
                        if (sdf_map_->getInflateOccupancy(grid_center3D) == 1)
                        {
                            map_.setOccupied(i, j, k);
                        }
                        else
                        {
                            if (sdf_map_->getOccupancy(grid_center3D) == sdf_map_->OCCUPANCY::FREE)
                                map_.setFree(i, j, k);
                            if (sdf_map_->getOccupancy(grid_center3D) == sdf_map_->OCCUPANCY::UNKNOWN)
                                map_.setUnknown(i, j, k);
                        }
                    }
                    else
                    {
                        map_.setUnknown(i, j, k);
                    }
                }
            }
        }

        inflate_map_ = GridMap3D(map_);
        is_map_updated_ = true;

        // sdf_map_->sdf_mutex_.unlock();
    }

    void Map3DManager::publishVisionMarkers()
    {
        std::shared_lock<std::shared_timed_mutex> lock(map_3d_mutex_);
        grid_map_3d_pub_.publish(inflate_map_.generateMapMarkers(inflate_map_.getMapGrid(), current_pose_));
    }
    void Map3DManager::clearRobotNeighbor(const GridPoint3D &center_point, double clear_radius)
    {
        Index3D center_index, min_index;
        {

            if (inflate_map_.getStatusInMap3D(center_point) == Status3D::Free_3D)
                return;

            center_index = inflate_map_.getIndexInMap3D(center_point);

            double min_distance = 1e10;
            min_index = center_index;
            for (double x = center_point.x() - clear_radius;
                 x < center_point.x() + clear_radius; x += m3dp_.grid_size_)
            {
                for (double y = center_point.y() - clear_radius;
                     y < center_point.y() + clear_radius; y += m3dp_.grid_size_)
                {
                    for (double z = center_point.z() - clear_radius;
                         z < center_point.z() + clear_radius; z += m3dp_.grid_size_)
                    {
                        GridPoint3D point(x, y, z);
                        if (inflate_map_.getStatusInMap3D(point) == Status3D::Free_3D)
                        {
                            if ((center_point - point).norm() < min_distance)
                            {
                                min_distance = (center_point - point).norm();
                                min_index = inflate_map_.getIndexInMap3D(point);
                            }
                        }
                    }
                }
            }
        }

        {
            inflate_map_.setFree(center_index);
            if (min_index != center_index)
            {
                GridPoint3D target = inflate_map_.getGridCenter(center_index);
                GridPoint3D source = inflate_map_.getGridCenter(min_index);
                double end_distance = (target - source).norm();
                Eigen::Vector3d directory = (target - source).normalized();
                double step_length = m3dp_.grid_size_ / 4;
                int step = 0;
                GridPoint3D inner_point = source + directory * step_length * step;
                while ((inner_point - source).norm() < end_distance)
                {
                    inflate_map_.setFree(inflate_map_.getIndexInMap3D(inner_point));
                    step++;
                    inner_point = source + directory * step_length * step;
                }
            }
        }
    }

    bool Map3DManager::isCollisionFreeStraight(const GridPoint3D &source, const GridPoint3D &target) const
    {
        std::shared_lock<std::shared_timed_mutex> lock(map_3d_mutex_);
        return inflate_map_.isCollisionFreeStraight(source, target);
    }

    bool Map3DManager::getShortestPath(const GridPoint3D &goal, const GridPoint3D &start, 
                                       const bool &is_clear_nearby, std::vector<GridPoint3D> &path3d)
    {
        std::shared_lock<std::shared_timed_mutex> lock(map_3d_mutex_);
        
        if (is_clear_nearby)
        {
            clearRobotNeighbor(start, m3dp_.inflate_radius_ + m3dp_.grid_size_);
        }
        
        return inflate_map_.getShortestPath(goal, start, path3d);
    }

    std::vector<GridPoint3D> Map3DManager::optimalToStraight(std::vector<GridPoint3D> &path) const
    {
        std::shared_lock<std::shared_timed_mutex> lock(map_3d_mutex_);
        return inflate_map_.optimalToStraight(path);
    }

    bool Map3DManager::getShortestOptimalPath(const GridPoint3D &goal, const GridPoint3D &start,
                                              const bool &is_clear_nearby, std::vector<GridPoint3D> &optimal_path3d)
    {
        std::unique_lock<std::shared_timed_mutex> lock(map_3d_mutex_);

        if (is_clear_nearby)
        {
            clearRobotNeighbor(start, m3dp_.inflate_radius_ + m3dp_.grid_size_);
        }

        std::vector<GridPoint3D> path3d;
        if (inflate_map_.getShortestPath(goal, start, path3d))
        {
            optimal_path3d = inflate_map_.optimalToStraight(path3d);
            return true;
        }
        else
        {
            optimal_path3d.clear();
            return false;
        }
    }

    bool Map3DManager::isFree(const GridPoint3D &point)
    {
        std::shared_lock<std::shared_timed_mutex> lock(map_3d_mutex_);
        return inflate_map_.getStatusInMap3D(point) == Status3D::Free_3D;
    }

    bool Map3DManager::isOccupied(const GridPoint3D &point)
    {
        std::shared_lock<std::shared_timed_mutex> lock(map_3d_mutex_);
        return inflate_map_.getStatusInMap3D(point) == Status3D::Occupied_3D;
    }

    bool Map3DManager::isUnknown(const GridPoint3D &point)
    {
        std::shared_lock<std::shared_timed_mutex> lock(map_3d_mutex_);
        return inflate_map_.getStatusInMap3D(point) == Status3D::Unknown_3D;
    }

    bool Map3DManager::isNearFree(const GridPoint3D &point, const double &range)
    {
        std::shared_lock<std::shared_timed_mutex> lock(map_3d_mutex_);
        return inflate_map_.isNearFree(point, range);
    }

    bool Map3DManager::isNearOccupied(const GridPoint3D &point, const double &range)
    {
        std::shared_lock<std::shared_timed_mutex> lock(map_3d_mutex_);
        return inflate_map_.isNearOccupy(point, range);
    }

    bool Map3DManager::isNearUnknown(const GridPoint3D &point, const double &range)
    {
        std::shared_lock<std::shared_timed_mutex> lock(map_3d_mutex_);
        return inflate_map_.isNearUnknown(point, range);
    }

}
