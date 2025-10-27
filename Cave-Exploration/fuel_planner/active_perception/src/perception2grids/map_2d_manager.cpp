//
// Created by hjl on 2022/1/11.
//

#include "perception2grids/map_2d_manager.h"

namespace perception
{
    Map2DManager::Map2DManager()
    {
    }

    void Map2DManager::initMap2DManager(ros::NodeHandle &nh_private, fast_planner::SDFMap::Ptr &sdf_map)
    {
        this->nh_private_ = nh_private;
        this->is_map_updated_ = false;
        this->sdf_map_ = sdf_map;
        getParamsFromRos();
        map_.initialize(m2dp_.grid_size_, m2dp_.map_length_[0], m2dp_.map_length_[1], Status2D::Unknown);
        
        inflate_map_.initialize(m2dp_.grid_size_, m2dp_.map_length_[0], m2dp_.map_length_[1], Status2D::Unknown);
        
        m2dp_.map_grid_num_ = map_.getMapGridNum();
        grid_map_2d_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("/topo_planner/gird_map_2d", 5);
        
        surface_frontiers_pub_ = nh_private_.advertise<visualization_msgs::Marker>("/topo_planner/gird_map_2d_surface_frontiers", 5);

        surface_frontier_normals_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("/topo_planner/surface_frontier_normals_2d", 1);

        surface_frontiers_.reset(new pcl::PointCloud<pcl::PointXYZ>);

        boundary_.cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);

        refine_grid_map_2d_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("/topo_planner/refine_gird_map_2d", 5);
    
        occupancy_cloud_pub_ = nh_private_.advertise<sensor_msgs::PointCloud2>("/topo_planner/occupancy_cloud", 1);

        // angle_status_pub_ = nh_private_.advertise<sensor_msgs::PointCloud2>("/topo_planner/angle_status", 1);
    
        detect_angle_pub_ = nh_private_.advertise<sensor_msgs::PointCloud2>("/topo_planner/detect_angle", 1);
    
        circle_pub_ = surface_frontiers_pub_ = nh_private_.advertise<visualization_msgs::Marker>("/topo_planner/circle", 1);
    
        cluster_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("/topo_planner/obstecle_cluster", 5);
    
        angle_status_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("/topo_planner/angle_status2", 1);
    
        obstacles_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);

        global_frontier_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);

        global_frontier_kdtree_.reset(new pcl::search::KdTree<pcl::PointXYZ>);

        virtual_wall_pub_ = nh_private_.advertise<sensor_msgs::PointCloud2>("/topo_planner/virtual_wall", 1);
    }

    void Map2DManager::getParamsFromRos()
    {
        std::string ns = ros::this_node::getName() + "/GridMap2D";

        m2dp_.frame_id_ = std::string("world");
        if (!ros::param::get(ns + "/frame_id", m2dp_.frame_id_))
        {
            ROS_WARN("No frame_id specified. Looking for %s. Default is 'world'.",
                     (ns + "/frame_id").c_str());
        }

        m2dp_.init_z_ = 1.0;
        if (!ros::param::get(ns + "/init_z", m2dp_.init_z_))
        {
            ROS_WARN("No init_z specified. Looking for %s. Default is '1.0'.",
                     (ns + "/init_z").c_str());
        }

        m2dp_.grid_size_ = 0.2;
        if (!ros::param::get(ns + "/grid_size", m2dp_.grid_size_))
        {
            ROS_WARN("No grid_size specified. Looking for %s. Default is '0.2'.",
                     (ns + "/grid_size").c_str());
        }

        m2dp_.map_length_[0] = 10;
        if (!ros::param::get(ns + "/map_size", m2dp_.map_length_[0]))
        {
            ROS_WARN("No m2dp_.map_length_[0] specified. Looking for %s. Default is '10'.",
                     (ns + "/map_size").c_str());
        }

        m2dp_.map_length_[1] = 10;
        if (!ros::param::get(ns + "/map_size", m2dp_.map_length_[1]))
        {
            ROS_WARN("No m2dp_.map_length_[1] specified. Looking for %s. Default is '10'.",
                     (ns + "/map_size").c_str());
        }

        m2dp_.inflate_radius_ = sdf_map_->getInflateRadius();
    }

    void Map2DManager::setCurrentPose(const geometry_msgs::Pose &current_pose)
    {
        std::unique_lock<std::shared_timed_mutex> lock(map_2d_mutex_);
        current_pose_ = current_pose;
    }

    void Map2DManager::updateGridMap2D()
    {
        // sdf_map_->sdf_mutex_.lock();
        // std::unique_lock<std::shared_timed_mutex> lock(map_2d_mutex_);

        updateGridStatus();
        
        obstaclesCluster();

        updateSurrandObstacles();

        findObstaclesClusterToFollow();

        // Point2D current_pos(current_pose_.position.x, current_pose_.position.y);

        // if(forward_directory_[0]==0 && forward_directory_[1]==0)
        //     forward_directory_[0] = 1;

        // double forward_directory_angle_ = std::atan2(forward_directory_.y(), forward_directory_.x()) * 180.0 / M_PI;
        // if (forward_directory_angle_ < 0) 
        //     forward_directory_angle_ += 360.0;

        // double start_angle = forward_directory_angle_ + 90;    
        // if(start_angle > 360.0)
        //     start_angle -= 360.0;

        // start_angle_index_ = int(start_angle/5);

        // angle_index_ = start_angle_index_;
        // int last_angle_index = start_angle_index_ + 1;
        // detect_ = false;
        
        // ROS_WARN("start_angle_index_: %d", start_angle_index_);

        // for(int i = 0; i < angle_status_.size(); i++)
        // {
        //     if(angle_index_ < 0)
        //         angle_index_+=72;
                
        //     if(last_angle_index < 0)
        //         last_angle_index+=72;

        //     if((angle_status_[angle_index_]-current_pos).norm() < 0.01 && (angle_status_[last_angle_index]-current_pos).norm() > 0.01)
        //     {
        //         detect_ = true;
        //         angle_index_++;
        //         break;
        //     }
        //     angle_index_--;
        //     last_angle_index--;      
        // }

        // ROS_WARN("angle_index: %d    detect: %d", angle_index_, int(detect_));

        publishAngleStatusMarkers();
        publishObstacleClustersMarkers();
    }

    void Map2DManager::updateGridStatus()
    {
        // std::unique_lock<std::shared_timed_mutex> lock(map_2d_mutex_);
        Eigen::Vector2d center_point(current_pose_.position.x, current_pose_.position.y);

        map_.resetMapStatus(Status2D::Unknown);
        map_.setMapCenterAndBoundary(center_point);

        inflate_map_.resetMapStatus(Status2D::Unknown);
        inflate_map_.setMapCenterAndBoundary(center_point);

        for (int i = 0; i < m2dp_.map_grid_num_[0]; ++i)
        {
            for (int j = 0; j < m2dp_.map_grid_num_[1]; ++j)
            {
                Point2D grid_center2D = map_.getGridCenter(i, j);
                Eigen::Vector3d grid_center3D(grid_center2D.x(), grid_center2D.y(), m2dp_.init_z_);
                Eigen::Vector3d current_pos(current_pose_.position.x, current_pose_.position.y, m2dp_.init_z_);
                Index2D index(i, j);

                if (sdf_map_->isInBox(grid_center3D))
                {
                    // if (sdf_map_->getInflateOccupancy(grid_center3D) == 1)
                    if (sdf_map_->getOccupancy(grid_center3D) == sdf_map_->OCCUPANCY::OCCUPIED)
                    {
                        map_.setOccupied(i, j);
                        
                        // index = {i+1, j};
                        // if(map_.isInMapRange2D(index))
                        //     map_.setOccupied(index);

                        // index = {i-1, j};
                        // if(map_.isInMapRange2D(index))
                        //     map_.setOccupied(index);
                        
                        // index = {i, j+1};
                        // if(map_.isInMapRange2D(index))
                        //     map_.setOccupied(index);

                        // index = {i, j-1};
                        // if(map_.isInMapRange2D(index))
                        //     map_.setOccupied(index);

                        // 三邻域膨胀
                        // index = {i+1, j};
                        // if(map_.isInMapRange2D(index))
                        //     map_.setOccupied(index);

                        // index = {i+1, j+1};
                        // if(map_.isInMapRange2D(index))
                        //     map_.setOccupied(index);
                        
                        // index = {i, j+1};
                        // if(map_.isInMapRange2D(index))
                        //     map_.setOccupied(index);
                    }
                    else if(map_.getStatusInMap2D(index)!=Status2D::Occupied)
                    {
                        if (sdf_map_->getOccupancy(grid_center3D) == sdf_map_->OCCUPANCY::FREE)
                            map_.setFree(i, j);
                        if (sdf_map_->getOccupancy(grid_center3D) == sdf_map_->OCCUPANCY::UNKNOWN)
                            map_.setUnknown(i, j);
                    }

                    // 将global_frontier作为障碍物
                    if(global_frontier_cloud_->points.size() > 0)
                    {
                        // 搜索的点
                        pcl::PointXYZ search_point;
                        search_point.x = grid_center3D.x(); // 示例值
                        search_point.y = grid_center3D.y(); // 示例值
                        search_point.z = grid_center3D.z(); // 示例值

                        // 搜索半径
                        float radius = 0.6; // 半径为1.0的示例值

                        // 用来保存搜索结果的向量
                        std::vector<int> indices; // 保存点索引的向量
                        std::vector<float> distance; // 保存点距离的向量

                        // 执行半径搜索
                        if (global_frontier_kdtree_->radiusSearch(search_point, radius, indices, distance) > 0)
                        {
                            map_.setOccupied(i, j);
                        }
                    }
                }
                else
                {
                    map_.setUnknown(i, j);
                }
            }
        }

        // inflate_map_ = GridMap2D(map_);

        for (int i = 0; i < m2dp_.map_grid_num_[0]; ++i)
        {
            for (int j = 0; j < m2dp_.map_grid_num_[1]; ++j)
            {
                Point2D grid_center2D = map_.getGridCenter(i, j);
                Eigen::Vector3d grid_center3D(grid_center2D.x(), grid_center2D.y(), m2dp_.init_z_);
                Index2D index(i, j);

                if (sdf_map_->isInBox(grid_center3D))
                {
                    if (sdf_map_->getInflateOccupancy(grid_center3D) == 1)
                    {
                        inflate_map_.setOccupied(i, j);
                    }
                    else
                    {
                        if (sdf_map_->getOccupancy(grid_center3D) == sdf_map_->OCCUPANCY::FREE)
                            inflate_map_.setFree(i, j);
                        if (sdf_map_->getOccupancy(grid_center3D) == sdf_map_->OCCUPANCY::UNKNOWN)
                            inflate_map_.setUnknown(i, j);
                    }
                }
                else
                {
                    inflate_map_.setUnknown(i, j);
                }
            }
        }
        Index2D center_index = map_.getIndexInMap2D(center_point);
        // 将机器人位置设为free
        map_.setFree(center_index);
        inflate_map_.setFree(center_index);

        is_map_updated_ = true; 
    }

    void Map2DManager::updateSurrandObstacles()
    {    
        // 更新每个障碍物集群对应的角度姿状态
        Point2D current_pos(current_pose_.position.x, current_pose_.position.y);
        for(int index = 0; index < 72; index++)
        {
            double angle = index * 5 *M_PI/180.0;   // 与x轴夹角

            Point2D castray_start, castray_end;
            castray_start = current_pos;

            castray_end[0] = current_pos[0] + std::cos(angle) * map_.getMapLength()[0];
            castray_end[1] = current_pos[1] + std::sin(angle) * map_.getMapLength()[1];
            
            Ray2D castray_cells = map_.castRay(castray_start, castray_end);
            
            for(int i = 1; i < castray_cells.size(); i++)
            {
                if(map_.getStatusInMap2D(castray_cells[i])==Status2D::Occupied) // 射线碰到障碍物
                {
                    int cell_id = map_.index2DToGridId(castray_cells[i]);
                    if(cell_to_cluster_.count(cell_id))
                    {
                        int cluster_id = cell_to_cluster_[cell_id];

                        if((obstacle_clusters[cluster_id].angle_status_[index]-current_pos).norm() < 0.1)
                            obstacle_clusters[cluster_id].angle_status_[index] = map_.getGridCenter(castray_cells[i]);

                    }     
                }
            }
        }
    }

    void Map2DManager::obstaclesCluster()
    {
        auto start = ros::Time::now().toSec(); 

        // 障碍物聚类
        obstacles_cloud_->points.clear();
        for (int i = 0; i < m2dp_.map_grid_num_[0]; ++i)
        {
            for (int j = 0; j < m2dp_.map_grid_num_[1]; ++j)
            {   
                Index2D cell_index(i, j);
                if(map_.getStatusInMap2D(cell_index) == Status2D::Occupied)
                {
                    pcl::PointXYZ cell_pos(cell_index.x(), cell_index.y(), m2dp_.init_z_);
                    obstacles_cloud_->points.push_back(cell_pos);
                }
            }
        }

        pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
        kdtree->setInputCloud(obstacles_cloud_);

        obstacle_clustered_indices_.clear();
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclidean_cluster;
        euclidean_cluster.setClusterTolerance(1.5);  // 2cm
        euclidean_cluster.setMinClusterSize(2);
        euclidean_cluster.setMaxClusterSize(25000);
        euclidean_cluster.setSearchMethod(kdtree);
        euclidean_cluster.setInputCloud(obstacles_cloud_);
        euclidean_cluster.extract(obstacle_clustered_indices_);
       
        Point2D current_pos(current_pose_.position.x, current_pose_.position.y);
        obstacle_clusters.clear();
        obstacle_clusters.resize(obstacle_clustered_indices_.size());
        
        for(int i=0; i<obstacle_clusters.size(); i++)
        {
            obstacle_clusters[i].angle_status_.resize(72, current_pos);
        }

        // 将聚类结果存入hash中，方便后续索引
        cell_to_cluster_.clear();
        for(int i=0; i<obstacle_clustered_indices_.size();i++)
        {
            for(auto index: obstacle_clustered_indices_[i].indices)
            {
                pcl::PointXYZ point = obstacles_cloud_->points[index];
                Index2D cell_index(point.x, point.y);
                int ind = map_.index2DToGridId(cell_index);
                cell_to_cluster_[ind] = i;
            }
        }
        auto end = ros::Time::now().toSec(); 
        
    }

    void Map2DManager::findObstaclesClusterToFollow()
    {
        Point2D current_pos(current_pose_.position.x, current_pose_.position.y);

        if(forward_directory_[0]==0 && forward_directory_[1]==0)
            forward_directory_[0] = 1;

        double forward_directory_angle_ = std::atan2(forward_directory_.y(), forward_directory_.x()) * 180.0 / M_PI;
        if (forward_directory_angle_ < 0) 
            forward_directory_angle_ += 360.0;

        double start_angle = forward_directory_angle_ + 120;    
        if(start_angle > 360.0)
            start_angle -= 360.0;

        start_angle_index_ = int(start_angle/5);

        angle_index_ = start_angle_index_;




        int last_angle_index = start_angle_index_ + 1;
        detect_ = false;
        
        // ROS_WARN("start_angle_index_: %d", start_angle_index_);

        for(int i = 0; i < 72; i++)
        {
            if(angle_index_ < 0)
                angle_index_+=72;
                
            if(last_angle_index < 0)
                last_angle_index+=72;


            for(int cluster_num = 0; cluster_num < obstacle_clusters.size(); cluster_num++)
            {
                if((obstacle_clusters[cluster_num].angle_status_[angle_index_]-current_pos).norm() < 0.01 
                    && (obstacle_clusters[cluster_num].angle_status_[last_angle_index]-current_pos).norm() > 0.01)
                {
                    detect_ = true;
                    angle_index_++;
                    cluster_to_follow_ = cluster_num;
                    angle_index_%=72;
                    break;
                }
            }
            if(detect_)
                break;
           
            angle_index_--;
            last_angle_index--;      
        }

        // ROS_WARN("angle_index: %d    detect: %d", angle_index_, int(detect_));

    }

    bool Map2DManager::isDetect()
    {
        return detect_;
    }
        
    Eigen::Vector3d Map2DManager::getAnglePos()
    {
        Eigen::Vector3d angle_pos;
        angle_pos[0] = obstacle_clusters[cluster_to_follow_].angle_status_[angle_index_][0];
        angle_pos[1] = obstacle_clusters[cluster_to_follow_].angle_status_[angle_index_][1];
        angle_pos[2] = m2dp_.init_z_;

        return angle_pos;
    }

    void Map2DManager::publishVirtualWall()
    {
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*global_frontier_cloud_, msg); // 转换
        msg.header.frame_id = "map"; // 设置适当的frame_id
        msg.header.stamp = ros::Time::now(); // 设置当前时间为时间戳
        virtual_wall_pub_.publish(msg); // 发布
    }

    void Map2DManager::publishObstacleClustersMarkers()
    {
        visualization_msgs::MarkerArray clusters_marker_array;
        visualization_msgs::Marker clusters_marker;
        clusters_marker.header.frame_id = "world";
        clusters_marker.header.stamp = ros::Time();
        clusters_marker.ns = "clusters";
        clusters_marker.id = 0;
        clusters_marker.type = visualization_msgs::Marker::CUBE_LIST;
        clusters_marker.action = visualization_msgs::Marker::DELETEALL;

        clusters_marker_array.markers.push_back(clusters_marker);

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(obstacles_cloud_);

        double color[3] = {0.8, 0.0, 0.0};
        for (int i = 0; i < obstacle_clusters.size(); i++) 
        {
            clusters_marker.points.clear();
            clusters_marker.header.stamp = ros::Time();
            clusters_marker.id = i;
            clusters_marker.action = visualization_msgs::Marker::ADD;
            clusters_marker.pose.orientation.w = 1.0;

            clusters_marker.scale.x = 0.6;
            clusters_marker.scale.y = 0.6;
            clusters_marker.scale.z = 0.6;

            clusters_marker.color.a = 1.0; // Don't forget to set the alpha!
            clusters_marker.color.r = color[i]; // Random color
            clusters_marker.color.g = color[(i+1)%3];
            clusters_marker.color.b = color[(i+2)%3];
            
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>); 
            pcl::PointIndices::Ptr indices(new pcl::PointIndices(obstacle_clustered_indices_[i]));
            extract.setIndices(indices);
            extract.filter(*cluster_cloud);

            for (const auto& point : cluster_cloud->points) 
            {
                Index2D cell_index(point.x, point.y);
                Point2D cell_center = map_.getGridCenter(cell_index);
                    // Index2D cell_index(i, j);
                geometry_msgs::Point ros_point;
                ros_point.x = cell_center.x();
                ros_point.y = cell_center.y();
                ros_point.z = point.z;
                clusters_marker.points.push_back(ros_point);
            }
            clusters_marker_array.markers.push_back(clusters_marker);

        }
        cluster_pub_.publish(clusters_marker_array);


        visualization_msgs::MarkerArray angle_marker_array;
        visualization_msgs::Marker marker;

        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time();
        marker.ns = "angle";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::DELETEALL;

        angle_marker_array.markers.push_back(marker);

        Point2D current_pos(current_pose_.position.x, current_pose_.position.y);

        for (int i = 0; i < obstacle_clusters.size(); i++) 
        {
            marker.points.clear();
            marker.header.stamp = ros::Time();
            marker.id = i;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1.0;

            marker.scale.x = 0.6;
            marker.scale.y = 0.6;
            marker.scale.z = 0.01;

            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = color[i];
            marker.color.g = color[(i+1)%3];
            marker.color.b = color[(i+2)%3];

            for(int j = 0; j < obstacle_clusters[i].angle_status_.size(); j++)
            {

                if((obstacle_clusters[i].angle_status_[j]-current_pos).norm()>0.01)
                {
                    geometry_msgs::Point point;
                    point.x = current_pos[0] + std::cos(j*5*M_PI/180.0)*10.0;
                    point.y = current_pos[1] + std::sin(j*5*M_PI/180.0)*10.0;
                    point.z = m2dp_.init_z_;
                    marker.points.push_back(point);
                }  
            } 
            angle_marker_array.markers.push_back(marker);
        }
        angle_status_pub_.publish(angle_marker_array);
    }

    void Map2DManager::publishAngleStatusMarkers()
    {   
        Point2D current_pos(current_pose_.position.x, current_pose_.position.y);

        // 发布搜索起始和结束角度的标记
        pcl::PointCloud<pcl::PointXYZ> detect_angle_cloud;
        pcl::PointXYZ point; 

        point.x = current_pos[0] + std::cos(start_angle_index_*5*M_PI/180.0)*10.0;
        point.y = current_pos[1] + std::sin(start_angle_index_*5*M_PI/180.0)*10.0;
        point.z = m2dp_.init_z_;
        detect_angle_cloud.points.push_back(point);

        if(detect_)
        {
            point.x = current_pos[0] + std::cos(angle_index_*5*M_PI/180.0)*10.0;
            point.y = current_pos[1] + std::sin(angle_index_*5*M_PI/180.0)*10.0;
            point.z = m2dp_.init_z_;
            detect_angle_cloud.points.push_back(point);
        }

        sensor_msgs::PointCloud2 detect_angle_msg;
        pcl::toROSMsg(detect_angle_cloud, detect_angle_msg); // 转换
        detect_angle_msg.header.frame_id = "world"; // 设置适当的参考系
        detect_angle_msg.header.stamp = ros::Time::now(); // 设置时间戳
        detect_angle_pub_.publish(detect_angle_msg); // 发布
        
        // 发布范围标记
        if(detect_)
        {
            visualization_msgs::Marker circle;
            circle.header.frame_id = "world"; // 或其他合适的参考系
            circle.header.stamp = ros::Time::now();
            circle.ns = "circle";
            circle.id = 0;
            circle.type = visualization_msgs::Marker::LINE_STRIP;
            circle.action = visualization_msgs::Marker::ADD;

            circle.pose.orientation.w = 1.0;

            circle.scale.x = 0.2; // 线宽

            circle.color.r = 1.0; // 颜色
            circle.color.g = 0.0;
            circle.color.b = 0.0;
            circle.color.a = 1.0; // 不透明度

            // 定义圆圈的参数
            double radius = 15/2.0;
            int point_num = 100; // 圆圈上点的数量

            for(int i = 0; i <= point_num; ++i) {
                double theta = M_PI * 2.0 * i / point_num;
                // 插值点
                geometry_msgs::Point point;
                point.x = radius * cos(theta) + obstacle_clusters[cluster_to_follow_].angle_status_[angle_index_][0];
                point.y = radius * sin(theta) + obstacle_clusters[cluster_to_follow_].angle_status_[angle_index_][1];
                point.z = m2dp_.init_z_; 

                circle.points.push_back(point);
            }
            circle_pub_.publish(circle);
        }
    }
    
    void Map2DManager::publishSurfaceFrntierNormalsMarkers()
    {
        visualization_msgs::MarkerArray marker_array;
        int id = 0;

        for(int i=0; i<boundary_.surface_frontiers_.size(); i++)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world"; 
            marker.header.stamp = ros::Time::now();
            marker.ns = "normals2d";
            marker.id = id;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1;
        
            geometry_msgs::Point start, end;

            start.x = boundary_.surface_frontiers_[i].x;
            start.y = boundary_.surface_frontiers_[i].y;
            start.z = boundary_.surface_frontiers_[i].z;

            end.x = start.x + boundary_.normals_[i].x()*3;
            end.y = start.y + boundary_.normals_[i].y()*3;
            end.z = start.z + boundary_.normals_[i].z()*3;

            marker.points.push_back(start);
            marker.points.push_back(end);

            marker.scale.x = 1; // Shaft diameter
            marker.scale.y = 1; // Head diameter
            marker.scale.z = 5.0;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;

            marker_array.markers.push_back(marker);

            id++;

            // ROS_WARN("start.x:%f y:%f x:%f  end.x:%f y:%f z:%f", start.x, start.y, start.z, end.x, end.y, end.z);
        }
        surface_frontier_normals_pub_.publish(marker_array);
    }


    void Map2DManager::publishSurfaceFrntiersMarkers()
    {
        visualization_msgs::Marker surface_frontiers_marker;
        
        surface_frontiers_marker.header.frame_id = "world";
        surface_frontiers_marker.header.stamp = ros::Time::now();
       
        surface_frontiers_marker.action = visualization_msgs::Marker::ADD;
        surface_frontiers_marker.pose.orientation.w = 1.0;

        surface_frontiers_marker.id = 0;
        surface_frontiers_marker.type = visualization_msgs::Marker::CUBE_LIST;
        // setting scale
        surface_frontiers_marker.scale.x = map_.getGridSize();
        surface_frontiers_marker.scale.y = map_.getGridSize();
        surface_frontiers_marker.scale.z = 0.01;

        // assigning colors
        surface_frontiers_marker.color.r = 0.0f;
        surface_frontiers_marker.color.g = 0.0f;
        surface_frontiers_marker.color.b = 1.0f;
        surface_frontiers_marker.color.a = 1.0f;

        geometry_msgs::Point point;
      
        for (auto surface_frontier: boundary_.surface_frontiers_)
        {        
            point.x = surface_frontier.x;
            point.y = surface_frontier.y;
            point.z = surface_frontier.z;
            
            surface_frontiers_marker.points.push_back(point);
            // ROS_WARN("point.x: %f   point.x: %f", point.x, point.x);
        }
        surface_frontiers_pub_.publish(surface_frontiers_marker);     
    }

    void Map2DManager::publishVisionMarkers()
    {
        std::shared_lock<std::shared_timed_mutex> lock(map_2d_mutex_);
        grid_map_2d_pub_.publish(inflate_map_.generateMapMarkers(inflate_map_.getMapGrid(), current_pose_));
        // grid_map_2d_pub_.publish(refine_map_.generateMapMarkers(refine_map_.getMapGrid(), current_pose_));
    }

    void Map2DManager::publishRefineVisionMarkers()
    {
        std::shared_lock<std::shared_timed_mutex> lock(map_2d_mutex_);
        // grid_map_2d_pub_.publish(inflate_map_.generateMapMarkers(inflate_map_.getMapGrid(), current_pose_));
        refine_grid_map_2d_pub_.publish(refine_map_.generateMapMarkers(refine_map_.getMapGrid(), current_pose_));
    }

    bool Map2DManager::isCollisionFreeStraight(const Point2D &source, const Point2D &target) const
    {
        std::shared_lock<std::shared_timed_mutex> lock(map_2d_mutex_);

        return inflate_map_.isCollisionFreeStraight(source, target);
    }

    void Map2DManager::clearRobotNeighbor(const Point2D &center_point, double clear_radius)
    {
        // std::unique_lock<std::shared_timed_mutex> lock(map_2d_mutex_);
        Index2D center_index, min_index;
        {
            if(!inflate_map_.isInMapRange2D(center_point))
                return;
            if (inflate_map_.getStatusInMap2D(center_point) == Status2D::Free)
                return;

            center_index = inflate_map_.getIndexInMap2D(center_point);

            double min_distance = 1e10;
            min_index = center_index;

            for (double x = center_point.x() - clear_radius;
                 x < center_point.x() + clear_radius; x += m2dp_.grid_size_)
            {
                for (double y = center_point.y() - clear_radius;
                     y < center_point.y() + clear_radius; y += m2dp_.grid_size_)
                {
                    Point2D point;
                    point.x() = x;
                    point.y() = y;
                    if (inflate_map_.getStatusInMap2D(point) == Status2D::Free)
                    {
                        double distance = (center_point - point).norm();
                        if (distance < min_distance)
                        {
                            min_distance = distance;
                            min_index = inflate_map_.getIndexInMap2D(point);
                        }
                    }
                }
            }
        }

        {
            // std::unique_lock<std::shared_timed_mutex> lock(map_2d_mutex_);
            inflate_map_.setFree(center_index);

            if (min_index != center_index)
            {
                Point2D target = inflate_map_.getGridCenter(center_index);
                Point2D source = inflate_map_.getGridCenter(min_index);
                double end_distance = (target - source).norm();
                Eigen::Vector2d directory = (target - source).normalized();
                double step_length = m2dp_.grid_size_ / 4;
                int step = 0;
                Point2D inner_point = source + directory * step_length * step;
                while ((inner_point - source).norm() < end_distance)
                {
                    inflate_map_.setFree(inflate_map_.getIndexInMap2D(inner_point));
                    step++;
                    inner_point = source + directory * step_length * step;
                }
            }
        }
    }

    bool Map2DManager::getShortestPath(const Point2D &goal, const Point2D &start, std::vector<Point2D> &path2d) const
    {
        std::shared_lock<std::shared_timed_mutex> lock(map_2d_mutex_);

        return inflate_map_.getShortestPath(goal, start, path2d);
    }

    std::vector<Point2D> Map2DManager::optimalToStraight(std::vector<Point2D> &path) const
    {
        std::shared_lock<std::shared_timed_mutex> lock(map_2d_mutex_);

        return inflate_map_.optimalToStraight(path);
    }

    bool Map2DManager::getShortestOptimalPath(const Point2D &goal, const Point2D &start,
                                              const bool &is_clear_nearby, std::vector<Point2D> &optimal_path2d)
    {
        std::unique_lock<std::shared_timed_mutex> lock(map_2d_mutex_);
        // ROS_WARN("lock(map_2d_mutex_);");
        // ROS_WARN("is_clear_nearby:%s", is_clear_nearby);
        if (is_clear_nearby)
        {
            clearRobotNeighbor(start, (m2dp_.inflate_radius_ + m2dp_.grid_size_) * 1.5);
            clearRobotNeighbor(goal, (m2dp_.inflate_radius_ + m2dp_.grid_size_) * 1.5);
        }
        

        std::vector<Point2D> path2d;
        if (inflate_map_.getShortestPath(goal, start, path2d))
        {
            optimal_path2d = inflate_map_.optimalToStraight(path2d);
            return true;
        }
        else
        {
            optimal_path2d.clear();
            return false;
        }
    }

    bool Map2DManager::isFree(const Point2D &point)
    {
        std::shared_lock<std::shared_timed_mutex> lock(map_2d_mutex_);

        return inflate_map_.getStatusInMap2D(point) == Status2D::Free;
    }

    bool Map2DManager::isOccupied(const Point2D &point)
    {
        std::shared_lock<std::shared_timed_mutex> lock(map_2d_mutex_);

        return inflate_map_.getStatusInMap2D(point) == Status2D::Occupied;
    }

    bool Map2DManager::isUnknown(const Point2D &point)
    {
        std::shared_lock<std::shared_timed_mutex> lock(map_2d_mutex_);

        return inflate_map_.getStatusInMap2D(point) == Status2D::Unknown;
    }

    bool Map2DManager::isNearFree(const Point2D &point, const double &range)
    {
        std::shared_lock<std::shared_timed_mutex> lock(map_2d_mutex_);

        return inflate_map_.isNearFree(point, range);
    }

    bool Map2DManager::isNearOccupied(const Point2D &point, const double &range)
    {
        std::shared_lock<std::shared_timed_mutex> lock(map_2d_mutex_);

        return inflate_map_.isNearOccupy(point, range);
    }

    bool Map2DManager::isNearUnknown(const Point2D &point, const double &range)
    {
        std::shared_lock<std::shared_timed_mutex> lock(map_2d_mutex_);

        return inflate_map_.isNearUnknown(point, range);
    }
    
    void histary()
    {
        // refine_map_ = GridMap2D(map_);

        // vector<Index2D> refine_indices;
        // bool refine = false;
        // do{
        //     refine = false;
        //     for (int i = 0; i < m2dp_.map_grid_num_[0]; ++i)
        //     {
        //         for (int j = 0; j < m2dp_.map_grid_num_[1]; ++j)
        //         {   
        //             Index2D index(i, j);

        //             if(refine_map_.getStatusInMap2D(index)!=Status2D::Occupied)
        //                 continue;
                        
        //             Index2D a1 = index;
        //             // Index2D a2(index[0]-1, index[1]);
        //             // Index2D a3(index[0]-1, index[1]+1);
        //             // Index2D a4(index[0], index[1]+1);
        //             // Index2D a5(index[0]+1, index[1]+1);
        //             // Index2D a6(index[0]+1, index[1]);
        //             // Index2D a7(index[0]+1, index[1]-1);
        //             // Index2D a8(index[0], index[1]-1);
        //             // Index2D a9(index[0]-1, index[1]-1);
        //             Index2D a2(index[0], index[1]+1);
        //             Index2D a3(index[0]+1, index[1]+1);
        //             Index2D a4(index[0]+1, index[1]);
        //             Index2D a5(index[0]+1, index[1]-1);
        //             Index2D a6(index[0], index[1]-1);
        //             Index2D a7(index[0]-1, index[1]-1);
        //             Index2D a8(index[0]-1, index[1]);
        //             Index2D a9(index[0]-1, index[1]+1);
                
        //             int a[9];

        //             a[0] = (refine_map_.getStatusInMap2D(a1) == Status2D::Occupied?1:0);
        //             a[1] = (refine_map_.getStatusInMap2D(a2) == Status2D::Occupied?1:0);
        //             a[2] = (refine_map_.getStatusInMap2D(a3) == Status2D::Occupied?1:0);
        //             a[3] = (refine_map_.getStatusInMap2D(a4) == Status2D::Occupied?1:0);
        //             a[4] = (refine_map_.getStatusInMap2D(a5) == Status2D::Occupied?1:0);
        //             a[5] = (refine_map_.getStatusInMap2D(a6) == Status2D::Occupied?1:0);
        //             a[6] = (refine_map_.getStatusInMap2D(a7) == Status2D::Occupied?1:0);
        //             a[7] = (refine_map_.getStatusInMap2D(a8) == Status2D::Occupied?1:0);
        //             a[8] = (refine_map_.getStatusInMap2D(a9) == Status2D::Occupied?1:0);

        //             bool req0 = true;
        //             bool req1 = true;
        //             bool req2 = true;
        //             bool req3 = true;
        //             bool req4 = true;

        //             if((i+j)%2==0)
        //                 req0 = true;
        //             else
        //                 req0 = false;

        //             int req1_sum = 0;
        //             req1_sum = a[1] + a[2] + a[3] + a[4] + a[5] + a[6] + a[7] + a[8];
                    
        //             if (req1_sum >= 2 && req1_sum <= 6) 
        //                 req1 = true;
        //             else 
        //                 req1 = false;

        //             //条件2  01的模式
        //             int req2_sum = 0;
                    
        //             for (int k = 2; k < 9; k++)
        //             {
        //                 if (a[k] == 1 && a[k - 1] == 0)
        //                     req2_sum += 1;
        //             }
                    
        //             if (req2_sum == 1) 
        //                 req2 = true;
        //             else 
        //                 req2 = false;

        //             //条件三
        //             int req3_sum = a[1] * a[3] * a[5];
        //             //std::cout << "req3_sum: " << req3_sum << std::endl;
        //             if (req3_sum == 0) 
        //                 req3 = true;
        //             else 
        //                 req3 = false;

        //             //条件四
        //             int req4_sum = a[3] * a[5] * a[7];
        //             //	std::cout << "req3_sum: " << req3_sum << std::endl;
        
        //             if (req4_sum == 0) 
        //                 req4 = true;
        //             else 
        //                 req4 = false;

        //             if (req1 && req2 && req3 && req4)
        //             {
        //                 refine_indices.push_back(index);
        //                 refine = true;
        //             }
        //         }
        //     }

        //     for(auto index:refine_indices)
        //     {
        //         refine_map_.setUnknown(index);
        //     }

        //     vector<Index2D> refine_indices2;
            
        //     for (int i = 0; i < m2dp_.map_grid_num_[0]; ++i)
        //     {
        //         for (int j = 0; j < m2dp_.map_grid_num_[1]; ++j)
        //         {   
        //             Index2D index(i, j);

        //             if(refine_map_.getStatusInMap2D(index)!=Status2D::Occupied)
        //                 continue;

        //             Index2D a1 = index;
        //             // Index2D a2(index[0]-1, index[1]);
        //             // Index2D a3(index[0]-1, index[1]+1);
        //             // Index2D a4(index[0], index[1]+1);
        //             // Index2D a5(index[0]+1, index[1]+1);
        //             // Index2D a6(index[0]+1, index[1]);
        //             // Index2D a7(index[0]+1, index[1]-1);
        //             // Index2D a8(index[0], index[1]-1);
        //             // Index2D a9(index[0]-1, index[1]-1);
        //             Index2D a2(index[0], index[1]+1);
        //             Index2D a3(index[0]+1, index[1]+1);
        //             Index2D a4(index[0]+1, index[1]);
        //             Index2D a5(index[0]+1, index[1]-1);
        //             Index2D a6(index[0], index[1]-1);
        //             Index2D a7(index[0]-1, index[1]-1);
        //             Index2D a8(index[0]-1, index[1]);
        //             Index2D a9(index[0]-1, index[1]+1);
                    
        //             int a[9];

        //             a[0] = (refine_map_.getStatusInMap2D(a1) == Status2D::Occupied?1:0);
        //             a[1] = (refine_map_.getStatusInMap2D(a2) == Status2D::Occupied?1:0);
        //             a[2] = (refine_map_.getStatusInMap2D(a3) == Status2D::Occupied?1:0);
        //             a[3] = (refine_map_.getStatusInMap2D(a4) == Status2D::Occupied?1:0);
        //             a[4] = (refine_map_.getStatusInMap2D(a5) == Status2D::Occupied?1:0);
        //             a[5] = (refine_map_.getStatusInMap2D(a6) == Status2D::Occupied?1:0);
        //             a[6] = (refine_map_.getStatusInMap2D(a7) == Status2D::Occupied?1:0);
        //             a[7] = (refine_map_.getStatusInMap2D(a8) == Status2D::Occupied?1:0);
        //             a[8] = (refine_map_.getStatusInMap2D(a9) == Status2D::Occupied?1:0);

        //             bool req0 = true;
        //             bool req1 = true;
        //             bool req2 = true;
        //             bool req3 = true;
        //             bool req4 = true;

        //             if((i+j)%2!=0)
        //                 req0 = true;
        //             else
        //                 req0 = false;
        
        //             //条件1 八领域的和
                
        //             int req1_sum = 0;
        //             req1_sum = a[1] + a[2] + a[3] + a[4] + a[5] + a[6] + a[7] + a[8];
        
        //             //std::cout << "req1_sum: " << req1_sum << std::endl;
        //             if (req1_sum >= 2 && req1_sum <= 6)
        //                 req1 = true;
        //             else 
        //                 req1 = false;
        
        //             //条件2  01的模式
        //             int req2_sum = 0;
        //             //	std::cout << "req2_sum: " << req2_sum << std::endl;
        //             for (int k = 2; k < 9; k++)
        //             {
        //                 if (a[k] == 1 && a[k - 1] == 0)
        //                     req2_sum += 1;
        //             }
        //             if (req2_sum == 1) 
        //                 req2 = true;
        //             else 
        //                 req2 = false;
        //             //条件三
        //             int req3_sum = a[1] * a[3] * a[7];
        //             if (req3_sum == 0) 
        //                 req3 = true;
        //             else 
        //                 req3 = false;
        //             //条件四
        //             int req4_sum = a[1] * a[5] * a[7];
        //             //	std::cout << "req3_sum: " << req3_sum << std::endl;
        //             if (req4_sum == 0) 
        //                 req4 = true;
        //             else 
        //                 req4 = false;

        //             if (req1 && req2 && req3 && req4)
        //             {
        //                 refine_indices2.push_back(index);
        //                 refine = true;
        //             }
                        
        //         }
        //     }

        //     for(auto index:refine_indices2)
        //     {
        //         refine_map_.setUnknown(index);
        //     }
        // }while(refine);

        // pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
        // pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ>);
        // kdtree->setInputCloud(sdf_map_->occupancy_cloud_);

        // for(auto surface_frontier:boundary_.surface_frontiers_)
        // {   
        //     // Eigen::Vector3d pos;
        //     // Eigen::Vector3d grad;

        //     // pos[0] = surface_frontier.x;
        //     // pos[1] = surface_frontier.y;
        //     // pos[2] = surface_frontier.z;
        //     // sdf_map_->getDistWithGrad(pos, grad);

        //     // 最小二乘拟合障碍物
        //     std::vector<int> indices;
        //     std::vector<float> distance;
        //     Eigen::Vector4f plane_params;

        //     float curvature;
        //     int index = 0;
            
        //     kdtree->radiusSearch(surface_frontier, 5, indices,  distance);

        //     // // 最小二乘直线拟合
        //     // std::vector<Eigen::Vector2d> occupied_points;
        //     // for(auto index:indices)
        //     // {
        //     //     Eigen::Vector2d occupied_point(boundary_.cloud_->points[index].x,boundary_.cloud_->points[index].y);
        //     //     occupied_points.push_back(occupied_point);
        //     // }
        //     // // 构建设计矩阵A和观测向量b
        //     // Eigen::MatrixXd A(occupied_points.size(), 2);
        //     // Eigen::VectorXd b(occupied_points.size());
        //     // for (size_t i = 0; i < occupied_points.size(); ++i) {
        //     //     A(i, 0) = occupied_points[i].x();
        //     //     A(i, 1) = 1.0;  // 常数项
        //     //     b(i) = occupied_points[i].y();
        //     // }
        //     // // 使用最小二乘法计算拟合参数
        //     // Eigen::Vector2d params = A.colPivHouseholderQr().solve(b);
            
        //     // pcl计算法向量
        //     normal_estimation.computePointNormal(*sdf_map_->occupancy_cloud_, indices, plane_params, curvature);
        //     Eigen::Vector3d cell_normal(plane_params[0], plane_params[1], plane_params[2]);

        //     // Eigen::Vector3d cell_normal(params[0], -1, 0);
        //     // Eigen::Vector3d cell_normal(grad[0], grad[1], 0);
        //     boundary_.normals_.push_back(cell_normal);
        //     ROS_WARN("cell_normal, x: %f  y: %f", cell_normal[0], cell_normal[1]);

        // }



       
        // ROS_WARN("boundary_.normals_.size: %d", boundary_.normals_.size());
        // ROS_WARN("boundary_.surface_frontiers_.size: %d", boundary_.surface_frontiers_.size());
        // // Eigen::Vector3d current_pos(current_pose_.position.x, current_pose_.position.y, current_pose_.position.z);
        // // Eigen::Vector3d cell_normal(plane_params[0], plane_params[1], plane_params[2]);
        
        // // Eigen::Vector3d delta_pos = current_pos - sdf_map_->AddressToPos(cell_iter);
        // // double dot_product = cell_normal.dot(delta_pos);

        // // if (dot_product < 0) 
        // // {
        // //     // 如果点积为负，说明法向量朝向外部，需要反转法向量
        // //     cell_normal = -cell_normal;
        // // }
        // // surface_frontiers[cell_iter] = cell_normal;

        // // pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ>);
        // // kdtree->setInputCloud(surface_frontiers_);
        // // std::vector<pcl::PointIndices> cluster_indices;
        // // pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclidean_cluster;
        // // euclidean_cluster.setClusterTolerance(1.0);  // 2cm，根据实际情况调整
        // // euclidean_cluster.setMinClusterSize(2);     // 最小聚类大小
        // // euclidean_cluster.setMaxClusterSize(25000);   // 最大聚类大小
        // // euclidean_cluster.setSearchMethod(kdtree);
        // // euclidean_cluster.setInputCloud(surface_frontiers_);
        // // euclidean_cluster.extract(cluster_indices);

        // // int j = 0;
        // // for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        // // {
        // //     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        // //     for (const auto& idx : it->indices)
        // //         cloud_cluster->push_back ((*surface_frontiers_)[idx]); //*
        // //     cloud_cluster->width = cloud_cluster->size ();
        // //     cloud_cluster->height = 1;
        // //     cloud_cluster->is_dense = true;

        // //     std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
        // //     j++;
        // // }

    }
}
