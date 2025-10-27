//
// Created by hjl on 2022/1/10.
//

#include "preprocess/viewpoint_manager.h"
#include <map>
#include <unordered_map>

using std::unordered_multimap;

namespace preprocess
{

    ViewpointManager::ViewpointManager(
        const ros::NodeHandle &nh_private,
        const FrontierFinder::Ptr &frontier_finder) : nh_private_(nh_private),
                                                      frontier_finder_(frontier_finder)
    {
        getParamsFromRos();

        repre_points_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("/topo_planner/repre_points_markers", 1);
        frontiers_viewpoints_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
            "/topo_planner/frontiers_viewpoints_markers", 1);
    }

    void ViewpointManager::setMap2DManager(const Map2DManager::Ptr &map_2d_manager)
    {
        this->map_2d_manager_ = map_2d_manager;
    }

    void ViewpointManager::setMap3DManager(const Map3DManager::Ptr &map_3d_manager)
    {
        this->map_3d_manager_ = map_3d_manager;
    }

    void ViewpointManager::setViewPointsBound(const Eigen::Vector3d &max_bound, const Eigen::Vector3d &min_bound)
    {
        this->vmp_.viewpoint_max_bound_ = max_bound;
        this->vmp_.viewpoint_min_bound_ = min_bound;
    }

    void ViewpointManager::getParamsFromRos()
    {
        std::string ns = ros::this_node::getName() + "/ViewpointManager";

        std::string pkg_path = ros::package::getPath("planner");
        std::string txt_path = pkg_path + "/../../files/exploration_data/";

        vmp_.frame_id_ = "world";
        if (!ros::param::get(ns + "/frame_id", vmp_.frame_id_))
        {
            ROS_WARN("No frame_id specified. Looking for %s. Default is 'world'.",
                     (ns + "/frame_id").c_str());
        }

        vmp_.sample_dist_h_ = 1.0;
        if (!ros::param::get(ns + "/sample_dist_horizon", vmp_.sample_dist_h_))
        {
            ROS_WARN("No sample_dist_horizon specified. Looking for %s. Default is '1.0'.",
                     (ns + "/sample_dist_horizon").c_str());
        }

        vmp_.sample_dist_v_ = 1.0;
        if (!ros::param::get(ns + "/sample_dist_vertical", vmp_.sample_dist_v_))
        {
            ROS_WARN("No sample_dist_vertical specified. Looking for %s. Default is '1.0'.",
                     (ns + "/sample_dist_vertical").c_str());
        }

        vmp_.init_z_ = 1.0;
        if (!ros::param::get(ns + "/init_z", vmp_.init_z_))
        {
            ROS_WARN("No init_z specified. Looking for %s. Default is '1.0'.",
                     (ns + "/init_z").c_str());
        }

        vmp_.viewpoint_gain_thre_ = 2.0;
        if (!ros::param::get(ns + "/viewpoint_gain_thre", vmp_.viewpoint_gain_thre_))
        {
            ROS_WARN("No viewpoint_gain_thre specified. Looking for %s. Default is '2.0'.",
                     (ns + "/viewpoint_gain_thre").c_str());
        }

        vmp_.points_dist_thre_h_ = 2.0;
        if (!ros::param::get(ns + "/points_dist_thre_horizon", vmp_.points_dist_thre_h_))
        {
            ROS_WARN("No points_dist_thre_horizon specified. Looking for %s. Default is '2.0'.",
                     (ns + "/points_dist_thre_horizon").c_str());
        }

        vmp_.points_dist_thre_v_ = 2.0;
        if (!ros::param::get(ns + "/points_dist_thre_vertical", vmp_.points_dist_thre_v_))
        {
            ROS_WARN("No points_dist_thre_vertical specified. Looking for %s. Default is '2.0'.",
                     (ns + "/points_dist_thre_vertical").c_str());
        }

        vmp_.bound_margin_h_ = 1.0;
        if (!ros::param::get(ns + "/bound_margin_horizon", vmp_.bound_margin_h_))
        {
            ROS_WARN("No bound_margin_horizon specified. Looking for %s. Default is '1.0'.",
                     (ns + "/bound_margin_horizon").c_str());
        }

        vmp_.bound_margin_v_ = 1.0;
        if (!ros::param::get(ns + "/bound_margin_vertical", vmp_.bound_margin_v_))
        {
            ROS_WARN("No bound_margin_vertical specified. Looking for %s. Default is '1.0'.",
                     (ns + "/bound_margin_vertical").c_str());
        }

        vmp_.points_dist_to_occupied_ = 0.5;
        if (!ros::param::get(ns + "/points_dist_to_occupied", vmp_.points_dist_to_occupied_))
        {
            ROS_WARN("No points_dist_to_occupied specified. Looking for %s. Default is '0.5'.",
                     (ns + "/points_dist_to_occupied").c_str());
        }

        vmp_.points_dist_to_unknown_ = 0.5;
        if (!ros::param::get(ns + "/points_dist_to_unknown", vmp_.points_dist_to_unknown_))
        {
            ROS_WARN("No points_dist_to_unknown specified. Looking for %s. Default is '0.5'.",
                     (ns + "/points_dist_to_unknown").c_str());
        }

        vmp_.points_clear_dist_thre_h_ = 2.0;
        if (!ros::param::get(ns + "/points_clear_dist_thre_horizon", vmp_.points_clear_dist_thre_h_))
        {
            ROS_WARN("No points_clear_dist_thre_horizon specified. Looking for %s. Default is '2.0'.",
                     (ns + "/points_clear_dist_thre_horizon").c_str());
        }

        vmp_.points_clear_dist_thre_v_ = 2.0;
        if (!ros::param::get(ns + "/points_clear_dist_thre_vertical", vmp_.points_clear_dist_thre_v_))
        {
            ROS_WARN("No points_clear_dist_thre_vertical specified. Looking for %s. Default is '2.0'.",
                     (ns + "/points_clear_dist_thre_vertical").c_str());
        }

        vmp_.frontier_viewpoint_dist_thre_ = 4.0;
        if (!ros::param::get(ns + "/frontier_viewpoint_dist_thre", vmp_.frontier_viewpoint_dist_thre_))
        {
            ROS_WARN("No frontier_viewpoint_dist_thre specified. Looking for %s. Default is '4.0'.",
                     (ns + "/frontier_viewpoint_dist_thre").c_str());
        }

        vmp_.lower_fov_ = 15.0;
        if (!ros::param::get(ns + "/lower_fov", vmp_.lower_fov_))
        {
            ROS_WARN("No lower_fov specified. Looking for %s. Default is '15.0'.",
                     (ns + "/lower_fov").c_str());
        }

        vmp_.upper_fov_ = 15.0;
        if (!ros::param::get(ns + "/upper_fov", vmp_.upper_fov_))
        {
            ROS_WARN("No upper_fov specified. Looking for %s. Default is '15.0'.",
                     (ns + "/upper_fov").c_str());
        }
    }

    void ViewpointManager::setCurrentPosition(const utils::Point3D &current_position)
    {
        current_position_ = current_position;
    }

    void ViewpointManager::updateViewpoints(const int &perception_dim)
    {
        if (perception_dim == 2)
        {
            if (!map_2d_manager_->is_map_updated_)
            {
                return;
            }
        }
        else if (perception_dim == 3)
        {
            if (!map_3d_manager_->is_map_updated_)
            {
                return;
            }
        }
        else
        {
            ROS_ERROR("perception_dim error");
            std::cerr << perception_dim << std::endl;
            return;
        }

        frontierAttachInSDFMap(perception_dim);
    }

    void ViewpointManager::clearWorthlessViewPoints(const int &perception_dim)
    {
        Point3DQueue need_to_erase;
        for (const auto &term : viewpoints_attached_frontiers_)
        {
            if (term.first.distanceXY(current_position_) < vmp_.points_clear_dist_thre_h_ &&
                fabs(term.first.z() - current_position_.z()) < vmp_.points_clear_dist_thre_v_)
            {
                need_to_erase.push_back(term.first);
                for (const auto &frontier : term.second)
                {
                    Eigen::Vector3d frontier_eigen(frontier.x(), frontier.y(), frontier.z());
                    frontier_finder_->eraseFrontier(frontier_eigen);
                    frontiers_viewpoints_.erase(frontier);
                }
            }
            if (perception_dim == 3)
            {
                if (term.second.size() * frontier_finder_->sdf_map_->getResolution() * frontier_finder_->sdf_map_->getResolution() < vmp_.viewpoint_gain_thre_)
                    need_to_erase.push_back(term.first);
            }
            else if (perception_dim == 2)
            {
                if (term.second.size() * frontier_finder_->sdf_map_->getResolution() < vmp_.viewpoint_gain_thre_)
                    need_to_erase.push_back(term.first);
            }
        }

        for (const auto &point : need_to_erase)
        {
            viewpoints_attached_frontiers_.erase(point);
        }
    }

    void ViewpointManager::frontierAttachInSDFMap(const int &perception_dim)
    {
        // viewpoints_ikdtree_.reset(new KD_TREE<PointType>(0.3, 0.6, 0.2));
        // points_indices_.clear();
        // points_vector_.clear();

        // if (perception_dim == 2)
        // {
        //     // ros::Time time1 = ros::Time::now();
        //     Point3DQueue sample_points = samplePointsInGridMap2D();
        //     ROS_INFO("start attach frontiers to candidate points..");
        //     // ros::Time time2 = ros::Time::now();
        //     // ROS_ERROR("sample_points: %fs", (time2 - time1).toSec());

        //     // candidate_viewpoints_记录的是历史的viewpoints，第一次是空的
        //     representative_points_.clear();
        //     representative_points_.insert(candidate_viewpoints_.begin(), candidate_viewpoints_.end());
        //     for (const auto &point : sample_points)
        //     {
        //         bool is_suitable = true;
        //         if (!((vmp_.viewpoint_max_bound_.x() - point.x()) >= vmp_.bound_margin_h_ &&
        //               (vmp_.viewpoint_max_bound_.y() - point.y()) >= vmp_.bound_margin_h_ &&
        //               (vmp_.viewpoint_max_bound_.z() - point.z()) >= vmp_.bound_margin_v_ &&
        //               (point.x() - vmp_.viewpoint_min_bound_.x()) >= vmp_.bound_margin_h_ &&
        //               (point.y() - vmp_.viewpoint_min_bound_.y()) >= vmp_.bound_margin_h_ &&
        //               (point.z() - vmp_.viewpoint_min_bound_.z()) >= vmp_.bound_margin_v_))
        //         {
        //             is_suitable = false;
        //         }

        //         // 筛选掉当前的距离历史的太近的point
        //         if (is_suitable)
        //         {
        //             for (const auto &viewpoint : representative_points_)
        //             {
        //                 if (point.distance(viewpoint) < vmp_.points_dist_thre_h_)
        //                 {
        //                     is_suitable = false;
        //                     break;
        //                 }
        //             }
        //         }

        //         if (is_suitable)
        //         {
        //             representative_points_.insert(point);
        //         }
        //     }

        //     // ros::Time time3 = ros::Time::now();
        //     // ROS_ERROR("representative_points2_: %fs", (time3 - time2).toSec());

        //     FrontierMap<Viewpoint> frontiers_viewpoints_term;
        //     std::vector<Eigen::Vector3d> frontier_vector = frontier_finder_->getSingleClusterFrontiers();
        //     for (const auto &frontier_temp : frontier_vector)
        //     {
        //         utils::Point3D frontier(frontier_temp);
        //         // 太远的viewpoints保持原来的连接
        //         if (frontier.distanceXY(current_position_) > frontier_finder_->getMaxRayLength() * 1.2 &&
        //             !frontiers_viewpoints_.empty() &&
        //             frontiers_viewpoints_.count(frontier) != 0)
        //         {
        //             frontiers_viewpoints_term[frontier] = frontiers_viewpoints_[frontier];
        //         }
        //         // 比较近的viewpoints重新链接
        //         else
        //         {
        //             double min_distance = 100000;
        //             Eigen::Vector3d frontier_coord(frontier_temp);
        //             Eigen::Vector3d sensor_point;
        //             double distance;

        //             for (const auto &point : representative_points_)
        //             {
        //                 sensor_point.x() = point.x();
        //                 sensor_point.y() = point.y();
        //                 sensor_point.z() = point.z();
        //                 distance = (frontier_coord - sensor_point).norm();

        //                 if (distance < frontier_finder_->getMaxRayLength() - 0.5 &&
        //                     distance < min_distance &&
        //                     fabs(frontier_coord.z() - sensor_point.z()) / distance < tan(M_PI * 15 / 180) &&
        //                     frontier_finder_->isCollisionFree(frontier_coord, sensor_point))
        //                 {
        //                     min_distance = distance;
        //                     if (min_distance < vmp_.frontier_viewpoint_dist_thre_)
        //                         frontiers_viewpoints_term[frontier] = point;
        //                 }
        //             }
        //         }
        //     }
        //     // ros::Time time4 = ros::Time::now();
        //     // ROS_ERROR("frontiers_viewpoints_term2: %fs", (time4 - time3).toSec());

        //     frontiers_viewpoints_.clear();
        //     frontiers_viewpoints_ = frontiers_viewpoints_term;
        //     ROS_INFO("viewpoints and frontiers nearest search finish.");

        //     viewpoints_attached_frontiers_.clear();
        //     for (const auto &item : frontiers_viewpoints_)
        //     {
        //         viewpoints_attached_frontiers_[item.second].push_back(item.first);
        //     }

        //     // ros::Time time5 = ros::Time::now();
        //     // ROS_ERROR("viewpoints_attached_frontiers2_: %fs", (time5 - time4).toSec());

        //     // 清除vp: 距离当前位置小于采样距离的vp, 少于viewpoint_gain_thre_个fts的vp,
        //     clearWorthlessViewPoints(perception_dim);

        //     // ros::Time time6 = ros::Time::now();
        //     // ROS_ERROR("viewpoints_attached_frontiers2_.erase(point);: %fs", (time6 - time5).toSec());

        //     candidate_viewpoints_.clear();
        //     for (const auto &item : viewpoints_attached_frontiers_)
        //     {
        //         candidate_viewpoints_.insert(item.first);
        //     }
        // }
        // else if (perception_dim == 3)
        // {
        //     ros::Time time1 = ros::Time::now();
        //     Point3DQueue sample_points = samplePointsInGridMap3D();
        //     ROS_INFO("start attach frontiers to candidate points..");

        //     // candidate_viewpoints_记录的是历史的viewpoints，第一次是空的
        //     representative_points_.clear();
        //     representative_points_.insert(candidate_viewpoints_.begin(), candidate_viewpoints_.end());
        //     for (const auto &point : sample_points)
        //     {
        //         bool is_suitable = true;
        //         if (!((vmp_.viewpoint_max_bound_.x() - point.x()) >= vmp_.bound_margin_h_ &&
        //               (vmp_.viewpoint_max_bound_.y() - point.y()) >= vmp_.bound_margin_h_ &&
        //               (vmp_.viewpoint_max_bound_.z() - point.z()) >= vmp_.bound_margin_v_ &&
        //               (point.x() - vmp_.viewpoint_min_bound_.x()) >= vmp_.bound_margin_h_ &&
        //               (point.y() - vmp_.viewpoint_min_bound_.y()) >= vmp_.bound_margin_h_ &&
        //               (point.z() - vmp_.viewpoint_min_bound_.z()) >= vmp_.bound_margin_v_))
        //         {
        //             is_suitable = false;
        //         }

        //         // 筛选掉当前的距离历史的太近的point
        //         if (is_suitable)
        //         {
        //             for (const auto &viewpoint : representative_points_)
        //             {
        //                 if (point.distanceXY(viewpoint) < vmp_.points_dist_thre_h_ &&
        //                     fabs(point.z() - viewpoint.z()) < vmp_.points_dist_thre_v_)
        //                 {
        //                     is_suitable = false;
        //                     break;
        //                 }
        //             }
        //         }
        //         if (is_suitable)
        //         {
        //             representative_points_.insert(point);
        //         }
        //     }
        //     ros::Time time2 = ros::Time::now();
        //     // ROS_WARN("vp-sample_points: %fs", (time2 - time1).toSec());

        //     // 构建representative_points_的kdtree
        //     PointVector point_cloud;
        //     for (const auto &viewpoint : representative_points_)
        //     {
        //         point_cloud.emplace_back(viewpoint.x(), viewpoint.y(), viewpoint.z());
        //         points_vector_.push_back(viewpoint);
        //         points_indices_.insert(std::make_pair(viewpoint, points_vector_.size() - 1));
        //     }
        //     viewpoints_ikdtree_->Build(point_cloud);

        //     ros::Time time3 = ros::Time::now();
        //     // ROS_ERROR("representative_points2_kd_tree: %fs", (time3 - time2).toSec());

        //     double lidar_max_range_v = map_3d_manager_->m3dp_.map_length_[2] / 2;
        //     double lidar_max_range_h = frontier_finder_->getMaxRayLength();
        //     // 清除frontiers_viewpoints_不再是frontiers的部分
        //     std::vector<Eigen::Vector3d> cleared_frontiers = frontier_finder_->getSimpleClearedFrontiers();
        //     std::vector<Eigen::Vector3d> frontier_vector = frontier_finder_->getSimpleLocalFrontiers(lidar_max_range_h * 1.2);
        //     frontier_finder_->resetSimpleClearedFrontiers();
        //     ros::Time time3_1 = ros::Time::now();
        //     // ROS_WARN("getSimpleClearedFrontiers: %fs", (time3_1 - time3).toSec());

        //     for (const auto &ft : cleared_frontiers)
        //     {
        //         if (frontiers_viewpoints_.count(utils::Point3D(ft)) != 0)
        //         {
        //             frontiers_viewpoints_.erase(utils::Point3D(ft));
        //         }
        //     }

        //     // 提取出剩下的frontiers
        //     utils::Point3DSet reserved_frontiers_near;
        //     FrontierMap<Viewpoint> frontiers_viewpoints_term;
        //     auto keySelector = [&reserved_frontiers_near, &frontiers_viewpoints_term,
        //                         lidar_max_range_h, this](const auto &pair)
        //     {
        //         if (current_position_.distanceXY(pair.first) < lidar_max_range_h * 1.2)
        //         {
        //             reserved_frontiers_near.insert(Eigen::Vector3d(pair.first.x(), pair.first.y(), pair.first.z()));
        //         }
        //         else
        //         {
        //             frontiers_viewpoints_term.insert(pair);
        //         }
        //     };
        //     std::for_each(frontiers_viewpoints_.begin(), frontiers_viewpoints_.end(), keySelector);

        //     // representative_points_记录历史的和当前的
        //     reserved_frontiers_near.insert(frontier_vector.begin(), frontier_vector.end());
        //     ros::Time time3_2 = ros::Time::now();
        //     // ROS_WARN("frontiers_viewpoints_.erase(utils::Point3D(ft)): %fs", (time3_2 - time3_1).toSec());

        //     for (const auto &frontier_temp : reserved_frontiers_near)
        //     {
        //         utils::Point3D frontier(frontier_temp);
        //         Eigen::Vector3d frontier_coord_eigen(frontier_temp.x(), frontier_temp.y(), frontier_temp.z());

        //         // 在frontier_viewpoint_dist_thre_范围内搜索近邻点
        //         PointVector point_results;
        //         viewpoints_ikdtree_->Radius_Search(PointType(frontier_temp.x(), frontier_temp.y(), frontier_temp.z()),
        //                                            vmp_.frontier_viewpoint_dist_thre_, point_results);

        //         // 对搜索结果排序
        //         std::multimap<double, utils::Point3D> sorted_results_ikdtree;
        //         for (int i = 0; i < point_results.size(); ++i)
        //         {
        //             utils::Point3D point(point_results[i].x, point_results[i].y, point_results[i].z);
        //             double distance = point.distance(frontier_temp);
        //             sorted_results_ikdtree.insert(std::make_pair(distance, point));
        //         }

        //         // 找最近连接
        //         for (const auto &result : sorted_results_ikdtree)
        //         {
        //             int point3d_index;
        //             if (getPointId(result.second, point3d_index))
        //             {
        //                 utils::Point3D nearest_point_3d = points_vector_[point3d_index];
        //                 Eigen::Vector3d nearest_point_eigen(nearest_point_3d.x(), nearest_point_3d.y(), nearest_point_3d.z());
        //                 Eigen::Vector3d diff_3d(frontier_coord_eigen - nearest_point_eigen);
        //                 diff_3d.z() = 0.0;
        //                 double distance_xy = diff_3d.norm();
        //                 if ((frontier_coord_eigen.z() >= nearest_point_eigen.z()
        //                          ? fabs(frontier_coord_eigen.z() - nearest_point_eigen.z()) / distance_xy < tan(M_PI * vmp_.upper_fov_ / 180)
        //                          : fabs(frontier_coord_eigen.z() - nearest_point_eigen.z()) / distance_xy < tan(M_PI * vmp_.lower_fov_ / 180)) &&
        //                     frontier_finder_->isCollisionFree(frontier_coord_eigen, nearest_point_eigen))
        //                 {
        //                     frontiers_viewpoints_term[frontier] = nearest_point_3d;
        //                     break;
        //                 }
        //             }
        //         }
        //     }
        //     ros::Time time4 = ros::Time::now();
        //     // ROS_WARN("frontiers_viewpoints_term: %fs", (time4 - time3_2).toSec());

        //     frontiers_viewpoints_.clear();
        //     frontiers_viewpoints_ = frontiers_viewpoints_term;
        //     ROS_INFO("viewpoints and frontiers nearest search finish.");

        //     viewpoints_attached_frontiers_.clear();
        //     for (const auto &item : frontiers_viewpoints_)
        //     {
        //         viewpoints_attached_frontiers_[item.second].push_back(item.first);
        //     }

        //     // ros::Time time5 = ros::Time::now();
        //     // ROS_ERROR("viewpoints_attached_frontiers2_: %fs", (time5 - time4).toSec());

        //     clearWorthlessViewPoints(perception_dim);

        //     // ros::Time time6 = ros::Time::now();
        //     // ROS_ERROR("viewpoints_attached_frontiers2_.erase(point);: %fs", (time6 - time4).toSec());

        //     candidate_viewpoints_.clear();
        //     for (const auto &item : viewpoints_attached_frontiers_)
        //     {
        //         candidate_viewpoints_.insert(item.first);
        //     }
        // }
        // else
        // {
        //     ROS_ERROR("viewpoints sample_dim is error!");
        //     std::cerr << perception_dim << std::endl;
        //     return;
        // }
    }

    ViewpointQueue ViewpointManager::samplePointsInGridMap2D()
    {
        double sample_dist = vmp_.sample_dist_h_;

        ViewpointQueue sample_points; // points meeting sampling requirements
        for (double x = -map_2d_manager_->m2dp_.map_length_[0] / 2;
             x <= map_2d_manager_->m2dp_.map_length_[0] / 2; x += sample_dist)
        {
            for (double y = -map_2d_manager_->m2dp_.map_length_[1] / 2;
                 y <= map_2d_manager_->m2dp_.map_length_[1] / 2; y += sample_dist)
            {
                Point2D sample_2d(current_position_.x() + x, current_position_.y() + y);

                if (frontier_finder_->sdf_map_->isInBox(Eigen::Vector3d(sample_2d.x(), sample_2d.y(), vmp_.init_z_)) &&
                    map_2d_manager_->isFree(sample_2d) &&
                    !map_2d_manager_->isNearOccupied(sample_2d, vmp_.points_dist_to_occupied_) &&
                    !map_2d_manager_->isNearUnknown(sample_2d, vmp_.points_dist_to_unknown_) &&
                    // map_2d_manager_->inflate_map_.isCollisionFreeStraight(Point2D(current_position_.x(), current_position_.y()), sample_2d) &&
                    (Point2D(current_position_.x(), current_position_.y()) - sample_2d).norm() <
                        (frontier_finder_->getMaxRayLength() - (frontier_finder_->getFrontierUpperBound() - frontier_finder_->getSensorHeight()) / tan(M_PI * 50 / 180)))
                {

                    Point3D sample_point(sample_2d.x(), sample_2d.y(), vmp_.init_z_);
                    sample_points.push_back(sample_point);
                }
            }
        }
        ROS_INFO("sample viewpoint finish, size is %zu", sample_points.size());
        return sample_points;
    }

    ViewpointQueue ViewpointManager::samplePointsInGridMap3D()
    {
        double sample_dist_h = vmp_.sample_dist_h_;
        double sample_dist_v = vmp_.sample_dist_v_;

        ViewpointQueue sample_points; // points meeting sampling requirements
        for (double x = -map_3d_manager_->m3dp_.map_length_[0] / 2;
             x <= map_3d_manager_->m3dp_.map_length_[0] / 2; x += sample_dist_h)
        {
            for (double y = -map_3d_manager_->m3dp_.map_length_[1] / 2;
                 y <= map_3d_manager_->m3dp_.map_length_[1] / 2; y += sample_dist_h)
            {
                for (double z = -map_3d_manager_->m3dp_.map_length_[2] / 2;
                     z <= map_3d_manager_->m3dp_.map_length_[2] / 2; z += sample_dist_v)
                {
                    GridPoint3D sample_3d(current_position_.x() + x, current_position_.y() + y, current_position_.z() + z);

                    if (frontier_finder_->sdf_map_->isInBox(Eigen::Vector3d(sample_3d.x(), sample_3d.y(), sample_3d.z())) && // 在sdf_map范围内
                        map_3d_manager_->isFree(sample_3d) &&                                                                // 在Map3D是free的
                        !map_3d_manager_->isNearOccupied(sample_3d, vmp_.points_dist_to_occupied_) &&                        // 在Map3D离occupy足够远的
                        !map_3d_manager_->isNearUnknown(sample_3d, vmp_.points_dist_to_unknown_) &&                          // 在Map3D离unknown足够远的
                        (Point2D(current_position_.x(), current_position_.y()) - Point2D(sample_3d.x(), sample_3d.y())).norm() <
                            (frontier_finder_->getMaxRayLength() - (frontier_finder_->getFrontierUpperBound() - frontier_finder_->getSensorHeight()) / tan(M_PI * 40 / 180)) &&
                        fabs(sample_3d.z() - current_position_.z()) / (Point2D(current_position_.x(), current_position_.y()) - Point2D(sample_3d.x(), sample_3d.y())).norm() <
                            tan(M_PI * 50 / 180))
                    // &&map_3d_manager_->inflate_map_.isCollisionFreeStraight(
                    //     GridPoint3D(current_position_.x(), current_position_.y(), current_position_.z()), sample_3d))
                    {
                        sample_points.push_back(sample_3d);
                    }
                }
            }
        }
        ROS_INFO("sample viewpoint finish, size is %zu", sample_points.size());
        return sample_points;
    }

    bool ViewpointManager::getPointId(const utils::Point3D &point, int &point_id)
    {
        if (points_indices_.count(point) != 0)
        {
            point_id = points_indices_[point];
            return true;
        }
        else
            return false;
    }

    visualization_msgs::MarkerArray ViewpointManager::generatePointsMarkers(const Point3DSet &sample_points) const
    {
        visualization_msgs::Marker points;
        points.header.frame_id = vmp_.frame_id_;
        points.header.stamp = ros::Time::now();
        points.ns = "representative_points";
        points.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = 1.0;

        points.id = 0;
        points.type = visualization_msgs::Marker::SPHERE_LIST;

        points.scale.x = 0.3;
        points.scale.y = 0.3;
        points.scale.z = 0.3;

        // assigning colors
        points.color.r = 1.0f;
        points.color.g = 1.0f;
        points.color.b = 1.0f;
        points.color.a = 1.0f;

        for (auto node : sample_points)
        {
            geometry_msgs::Point point;
            point.x = node.x();
            point.y = node.y();
            point.z = node.z();
            points.points.push_back(point);
        }
        visualization_msgs::MarkerArray repre_points_markers;
        repre_points_markers.markers.resize(1);
        repre_points_markers.markers[0] = points;

        return repre_points_markers;
    }

    visualization_msgs::MarkerArray
    ViewpointManager::generateViewPointsWithFrontiersMarkers(
        const ViewpointMap<FrontierQueue> &viewpoints_attached_frontiers) const
    {
        visualization_msgs::Marker points;
        visualization_msgs::Marker edges;
        visualization_msgs::Marker squares;

        edges.header.frame_id = points.header.frame_id = squares.header.frame_id = vmp_.frame_id_;
        edges.header.stamp = points.header.stamp = squares.header.stamp = ros::Time::now();
        edges.ns = "connections";
        points.ns = "viewpoints";
        squares.ns = "frontiers";
        edges.action = points.action = squares.action = visualization_msgs::Marker::ADD;
        edges.pose.orientation.w = points.pose.orientation.w = squares.pose.orientation.w = 1.0;

        points.id = 0;
        points.type = visualization_msgs::Marker::SPHERE_LIST;

        edges.id = 1;
        edges.type = visualization_msgs::Marker::LINE_LIST;

        squares.id = 2;
        squares.type = visualization_msgs::Marker::CUBE_LIST;

        points.scale.x = 0.5;
        points.scale.y = 0.5;
        points.scale.z = 0.5;

        // assigning colors
        points.color.r = 1.0f;
        points.color.g = 0.5f;
        points.color.b = 1.0f;
        points.color.a = 1.0f;

        // setting scale
        edges.scale.x = 0.1;
        edges.scale.y = 0.1;
        edges.scale.z = 0.1;

        edges.color.r = 0.5f;
        edges.color.g = 0.5f;
        edges.color.b = 0.1f;
        edges.color.a = 0.5f;

        squares.scale.x = frontier_finder_->sdf_map_->getResolution();
        squares.scale.y = frontier_finder_->sdf_map_->getResolution();
        squares.scale.z = frontier_finder_->sdf_map_->getResolution();

        // assigning colors
        squares.color.r = 0.5f;
        squares.color.g = 0.0f;
        squares.color.b = 1.0f;
        squares.color.a = 1.0f;

        for (const auto &node : viewpoints_attached_frontiers)
        {
            geometry_msgs::Point point;
            point.x = node.first.x();
            point.y = node.first.y();
            point.z = node.first.z();
            points.points.push_back(point);
        }
        for (const auto &edge : viewpoints_attached_frontiers)
        {
            auto first = edge.first;
            for (const auto &second : edge.second)
            {
                geometry_msgs::Point point;
                point.x = first.x();
                point.y = first.y();
                point.z = first.z();
                edges.points.push_back(point);
                point.x = second.x();
                point.y = second.y();
                point.z = second.z();
                edges.points.push_back(point);
                squares.points.push_back(point);
            }
        }

        for (const auto &node : viewpoints_attached_frontiers)
        {
            geometry_msgs::Point point;
            point.x = node.first.x();
            point.y = node.first.y();
            point.z = node.first.z();
            points.points.push_back(point);
        }
        for (const auto &edge : viewpoints_attached_frontiers)
        {
            auto first = edge.first;
            for (const auto &second : edge.second)
            {
                geometry_msgs::Point point;
                point.x = first.x();
                point.y = first.y();
                point.z = first.z();
                edges.points.push_back(point);
                point.x = second.x();
                point.y = second.y();
                point.z = second.z();
                edges.points.push_back(point);
                squares.points.push_back(point);
            }
        }

        visualization_msgs::MarkerArray frontiers_viewpoints_markers;
        frontiers_viewpoints_markers.markers.resize(3);
        frontiers_viewpoints_markers.markers[0] = points;
        frontiers_viewpoints_markers.markers[1] = edges;
        frontiers_viewpoints_markers.markers[2] = squares;

        return frontiers_viewpoints_markers;
    }

    void ViewpointManager::pubMarkers() const
    {
        // repre_points_pub_.publish(generatePointsMarkers(representative_points_));
        frontiers_viewpoints_pub_.publish(generateViewPointsWithFrontiersMarkers(viewpoints_attached_frontiers_));
    }
}
