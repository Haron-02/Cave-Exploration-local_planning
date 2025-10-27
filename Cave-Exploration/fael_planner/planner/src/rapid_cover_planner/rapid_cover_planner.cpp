// 无人机 “快速覆盖规划器”
// 找 “有价值的观测点”（比如能探索更多未知区域的点）、算每个点的 “价值（增益）”、规划最优路径，最终让机器人高效探索环境
// “读配置参数→初始化（算观测点价值）→选要去的观测点→规划到观测点的路径→可视化路径和观测点”
//
// Created by hjl on 2022/1/13.
//

#include "rapid_cover_planner/rapid_cover_planner.h"
#include "file_utils/file_rw.h"
#include "time_utils/time_utils.h"

namespace rapid_cover_planner
{

    RapidCoverPlanner::RapidCoverPlanner(ros::NodeHandle &nh_private) : nh_private_(nh_private)
    {

        // 1. 初始化“停止移动发布器”：给路径执行器发“停止”指令
        stop_move_pub_ = nh_private_.advertise<std_msgs::Bool>("/topo_planner/stop_move", 1);

        // 2. 从ROS参数服务器读配置（比如初始位置、观测点数量限制）
        setParamsFromRos();

        // 3. 初始化“初始位置”（从参数里读的init_x_/y_/z_）
        init_point_.x() = init_x_;
        init_point_.y() = init_y_;
        init_point_.z() = init_z_;

        // debug
        // 4. 初始化“可视化发布器”（在RViz里显示数据，方便调试）
        viewpoint_with_path_pub_ = nh_private_.advertise<sensor_msgs::PointCloud2>("/topo_planner/viewpoint_with_path_marker", 1);  // 观测点的点云
        viewpoints_gain_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("/topo_planner/viewpoints_gain_marker_", 1);  // 观测点的价值标签

        viewpoint_with_frontiers_pub_ = nh_private_.advertise<visualization_msgs::Marker>("/topo_planner/viewpoint_with_frontiers_marker", 1);  // 观测点到前沿的连线

        path_pub_ = nh_private_.advertise<sensor_msgs::PointCloud2>("/topo_planner/path_marker", 1);    // 规划好的路径点云
    }

    void RapidCoverPlanner::setFrontierFinder(const FrontierFinder::Ptr &frontier_finder)
    {
        this->frontier_finder_ = frontier_finder;
    }

    void RapidCoverPlanner::setMap2DManager(const Map2DManager::Ptr &map_2d_manager)
    {
        map_2d_manager_ = map_2d_manager;
    }

    void RapidCoverPlanner::setMap3DManager(const Map3DManager::Ptr &map_3d_manager)
    {
        map_3d_manager_ = map_3d_manager;
    }

    void RapidCoverPlanner::setViewpointManager(const ViewpointManager::Ptr &viewpoint_manager)
    {
        viewpoint_manager_ = viewpoint_manager;
    }

    void RapidCoverPlanner::setTopoGraph(const IkdTreeTopoGraph::Ptr &road_graph)
    {
        road_graph_ = road_graph;
    }

    void RapidCoverPlanner::setPlannerDim(const int &dim)
    {
        this->planner_dim_ = dim;
    }

    //定 “飞行规矩”：读配置参数
    void RapidCoverPlanner::setParamsFromRos()
    {
        std::string ns = ros::this_node::getName() + "/RapidCoverPlanner";

        init_x_ = 3.0;
        if (!ros::param::get(ns + "/init_x", init_x_))
        {
            ROS_WARN("No init_x specified. Looking for %s. Default is 3.0",
                     (ns + "/init_x").c_str());
        }

        init_y_ = -4.0;
        if (!ros::param::get(ns + "/init_y", init_y_))
        {
            ROS_WARN("No init_y specified. Looking for %s. Default is -4.0",
                     (ns + "/init_y").c_str());
        }

        init_z_ = 2.0;
        if (!ros::param::get(ns + "/init_z", init_z_))
        {
            ROS_WARN("No init_z specified. Looking for %s. Default is 2.0",
                     (ns + "/init_z").c_str());
        }

        //  停止阈值
        stop_x_thre_ = 1.0;
        if (!ros::param::get(ns + "/stop_x_thre", stop_x_thre_))
        {
            ROS_WARN("No stop_x_thre specified. Looking for %s. Default is 1.0",
                     (ns + "/stop_x_thre").c_str());
        }

        stop_y_thre_ = 1.0;
        if (!ros::param::get(ns + "/stop_y_thre", stop_y_thre_))
        {
            ROS_WARN("No stop_y_thre specified. Looking for %s. Default is 1.0",
                     (ns + "/stop_y_thre").c_str());
        }

        stop_z_thre_ = 1.0;
        if (!ros::param::get(ns + "/stop_z_thre", stop_z_thre_))
        {
            ROS_WARN("No stop_z_thre specified. Looking for %s. Default is 1.0",
                     (ns + "/stop_z_thre").c_str());
        }

        max_tour_point_num_ = 10;   // 一次规划最多10个观测点
        if (!ros::param::get(ns + "/max_tour_point_num", max_tour_point_num_))
        {
            ROS_WARN("No max_tour_point_num specified. Looking for %s. Default is 10",
                     (ns + "/max_tour_point_num").c_str());
        }

        viewpoint_ignore_thre_ = 1.0;
        if (!ros::param::get(ns + "/viewpoint_ignore_thre", viewpoint_ignore_thre_))
        {
            ROS_WARN("No viewpoint_ignore_thre specified. Looking for %s. Default is 1.0",
                     (ns + "/viewpoint_ignore_thre").c_str());
        }

        tourpoint_ignore_distance_ = 3.0;
        if (!ros::param::get(ns + "/tourpoint_ignore_distance", tourpoint_ignore_distance_))
        {
            ROS_WARN("No tourpoint_ignore_distance specified. Looking for %s. Default is 3.0",
                     (ns + "/tourpoint_ignore_distance").c_str());
        }

        tourpoint_ignore_thre_ = 2.0;
        if (!ros::param::get(ns + "/tourpoint_ignore_thre", tourpoint_ignore_thre_))
        {
            ROS_WARN("No tourpoint_ignore_thre specified. Looking for %s. Default is 2.0",
                     (ns + "/tourpoint_ignore_thre").c_str());
        }

        local_range_ = 10.0;
        if (!ros::param::get(ns + "/local_range", local_range_))
        {
            ROS_WARN("No local_range specified. Looking for %s. Default is 1.0",
                     (ns + "/local_range").c_str());
        }
        frontier_gain_ = 1.0;
        if (!ros::param::get(ns + "/frontier_gain", frontier_gain_))
        {
            ROS_WARN("No frontier_gain specified. Looking for %s. Default is 1.0",
                     (ns + "/frontier_gain").c_str());
        }

        is_local_planning_ = true;

        is_surface_follow_ = true;

        is_directory_ = true;
        if (!ros::param::get(ns + "/is_directory", is_directory_))
        {
            ROS_WARN("No is_directory specified. Looking for %s. Default is true",
                     (ns + "/is_directory").c_str());
        }

        // 7. 设“成本权重”（方向和距离的重要程度）
        alpha_ = 0.5;   //方向权重0.5（方向差越大，成本越高）
        if (!ros::param::get(ns + "/alpha", alpha_))
        {
            ROS_WARN("No alpha specified. Looking for %s. Default is 0.5",
                     (ns + "/alpha").c_str());
        }

        lambda_ = 1.0;  // 距离权重1.0（距离越远，成本越高）
        if (!ros::param::get(ns + "/lambda", lambda_))
        {
            ROS_WARN("No lambda specified. Looking for %s. Default is 1.0",
                     (ns + "/lambda").c_str());
        }

        // std::ofstream fout;
        std::string pkg_path = ros::package::getPath("planner");
        std::string dir_path = pkg_path + "/../../exploration_data_files/RapidCoverPlanner";
        file_utils::createDirectory(dir_path);

        sum_initilize_time_ = 0;
        initilize_num_ = 0;
        // fout.open(each_tourpoints_initilize_txt_name_, std::ios_base::in | std::ios_base::out | std::ios_base::trunc);
        // fout << "each detection elisped time \n"
        //      << "start time \t"
        //      << "end time \t"
        //      << "elisped time \t"
        //      << "detection_num \t"
        //      << "average time \t"
        //      << std::endl;
        // fout.close();

        // 9. 初始化“时间统计变量”（记规划花了多久）
        sum_solving_time_ = 0;
        solving_num_ = 0;
        each_solving_txt_name_ = dir_path + "/each_rcplan_solving_time.txt";    
        file_utils::writeToFileByTrunc(each_solving_txt_name_, "\t", "start_time(s)", "end_time(s)", "elisped_time(s)",
                                       "iteration_num", "average_time(s)", "points_num");

        sum_two_opt_time = 0.0; // Two-Opt算法总时间（找最短路线的算法）
        // fout.open(two_opt_time_name_, std::ios_base::in | std::ios_base::out | std::ios_base::trunc);
        // fout << "solve_number\t"
        //      << "\t"
        //      << "solve_time(s)\t"
        //      << "mean_time(s)\t"
        //      << "points_num\t" << std::endl;
        // fout.close();

        frontier_gain_time_txt_ = dir_path + "/frontiers_gains_time.txt ";
        file_utils::writeToFileByTrunc(frontier_gain_time_txt_, "\t", "iteration", "duration", "avg_time");
        sum_fontier_gains_time_ = 0.0;  // 总分数计算时间
        fontier_gains_num_ = 0; // 分数计算次数
    }

    bool RapidCoverPlanner::findClosestVirtualObstacle(const Point3D &current_position, Point3D &closest_virtual_obstacle)
    {
        // unordered_set<int> virtual_obstacle_cells = frontier_finder_->getVirtualObstacleCells();
        // bool is_successful = false;
        // double min_distance = DBL_MAX;
        // int closest_cell;
        // for(auto cell:virtual_obstacle_cells)
        // {   
        //     if(!frontier_finder_->isInObstacleCluster(cell))
        //         continue;
        //     int current_point_id = addCurrentPositionToGraph(current_position);
        //     Vector3d cell_pos = frontier_finder_->sdf_map_->AddressToPos(cell);
        //     Point3D target(cell_pos.x(), cell_pos.y(), cell_pos.z());
        //     Path path;
        //     path = getPathInGridMap2D(current_position, target, true);
            
        //     if (path.empty())
        //     {
        //         ROS_WARN("Find path to virtual obstacle in RoadMap");
        //         int target_id = road_graph_->addVertex(target);
        //         int b_id;
        //         utils::Point3D nearest_point;
        //         if (road_graph_->nearestSearch(target, nearest_point, b_id))
        //         {
        //             road_graph_->addTwoWayEdge(target_id, b_id);
        //             ROS_WARN("Added target point and its edges in plan graph.");
        //         }
        //         path = getPathInGraph(current_point_id, target_id);
        //         if (path.empty())
        //         {
        //             // path = getPathFromInflateRegion(current_position, init_point_);
        //         }
        //     }
        //     if (path.empty())
        //     {
        //         ROS_WARN("Path to virtual obstacle is still empty");
        //     }
        //     else
        //     {
        //         is_successful = true;
        //         double distance = 0;
        //         for(int i = 0; i < path.size()- 1; i++)
        //         {
        //             distance += (path[i+1] - path[i]).norm();
        //         }
        //         if(distance < min_distance)
        //         {
        //             min_distance = distance;
        //             closest_virtual_obstacle = target;
        //             closest_cell = cell;
        //         }
        //         ROS_WARN("x:%f  y:%f  z:%f  dis:%f", cell_pos.x(), cell_pos.y(), cell_pos.z(), distance);
        //     }
        // }
        // ROS_WARN("Closest x:%f  y:%f  z:%f  dis:%f", closest_virtual_obstacle.x(), closest_virtual_obstacle.y(), closest_virtual_obstacle.z(), min_distance);
        // frontier_finder_->setFollowingObstacleInd(closest_cell);
        // return is_successful;
    }

    //  算观测点分数 + 滤点 + 更导航地图
    void RapidCoverPlanner::Initialize(const Point3D &current_position)
    {
        ROS_INFO("start initializing ....");
        ros::WallTime start_time = ros::WallTime::now();
        initilize_num_++;

        // 计算Frontier增益
        tour_points_gains_.clear();
        max_gain_ = 0.0;
        auto frontier_time_1 = ros::WallTime::now();
        viewpointsFrontierGain(tour_points_, tour_points_gains_, max_gain_);
        auto frontier_time_2 = ros::WallTime::now();
        sum_fontier_gains_time_ += (frontier_time_2 - frontier_time_1).toSec();
        fontier_gains_num_++;
        file_utils::writeToFileByAdd(frontier_gain_time_txt_, "\t", fontier_gains_num_,
                                     (frontier_time_2 - frontier_time_1).toSec(), sum_fontier_gains_time_ / fontier_gains_num_);

        // 太近的viewpoints只有超过tourpoint_ignore_thre_的才计算在内
        if (tour_points_.size() > 3)
        {
            utils::Point3DSet tour_points;
            std::map<double, Point3DQueue> distances_viewpoints;

            for (const auto &item : tour_points_)   
            {
                auto distance = item.distance(current_position);
                distances_viewpoints[distance].push_back(item);
            }

            for (const auto &item : distances_viewpoints)
            {
                if (item.first < tourpoint_ignore_distance_)
                {
                    for (const auto &point : item.second)
                    {
                        if (viewpoint_manager_->viewpoints_attached_frontiers_[point].size() *
                                frontier_finder_->sdf_map_->getResolution() * frontier_finder_->sdf_map_->getResolution() >
                            tourpoint_ignore_thre_)
                        {
                            tour_points.insert(point);
                        }
                    }
                }
                else
                {
                    for (const auto &point : item.second)
                    {
                        tour_points.insert(point);
                    }
                }
            }
            tour_points_ = tour_points; 
        }

        ROS_INFO("plan graph update...");

        // 将tourpoints放进road_graph_->graph_， 与最邻近点连一条边
        for (auto &viewpoint : tour_points_)
        {
            if (!road_graph_->isPointExisted(viewpoint))
            {
                int a_id = road_graph_->addVertex(viewpoint);
                int b_id;
                utils::Point3D nearest_point;
                if (road_graph_->nearestSearch(viewpoint, nearest_point, b_id))
                {
                    road_graph_->addTwoWayEdge(a_id, b_id);
                    ROS_INFO("added a tour point and its edges in plan graph.");
                }
                // road_graph_->connectPointToGraph2D(viewpoint);
            }
        }
        road_graph_->updateKdtree();

        ros::WallTime end_time = ros::WallTime::now();
        ROS_INFO("fast ray casting gain computed finish,spent %f s", (end_time - start_time).toSec());
        sum_initilize_time_ = sum_initilize_time_ + (end_time - start_time).toSec();
        // std::ofstream fout;
        // fout.open(each_tourpoints_initilize_txt_name_, std::ios_base::in | std::ios_base::out | std::ios_base::app);
        // fout << start_time << "\t" << end_time << "\t" << (end_time - start_time).toSec()
        //      << "\t" << initilize_num_ << "\t" << sum_initilize_time_ / initilize_num_ << "s \t"
        //      << std::endl;
        // fout.close();
    }

    //  核心：给观测点 “打分”
    void RapidCoverPlanner::viewpointsFrontierGain(utils::Point3DSet &viewpoints,
                                                   utils::Point3DMap<double> &tour_points_gains, double &max_gain)
    {
        ROS_INFO("start tour points gain computing...");

        // for (const auto &viewpoint : viewpoints)
        // {
        //     tour_points_gains[viewpoint] = static_cast<double>(viewpoint_manager_->viewpoints_attached_frontiers_[viewpoint].size()) * frontier_gain_;
        //     if (tour_points_gains[viewpoint] > max_gain)
        //     {
        //         max_gain = tour_points_gains[viewpoint];
        //     }
        // }
        std::vector<Eigen::Vector3d> frontiers;
        double sensor_range = frontier_finder_->sdf_map_->getMaxRayLength();    // 无人机传感器最大探测距离（比如5米）
        if (is_local_planning_) // 局部规划→只拿当前位置2倍传感器范围内的边界
            frontiers = frontier_finder_->getSimpleLocalFrontiers(2 * sensor_range);
        else    // 全局规划→拿所有边界
            frontiers = frontier_finder_->getSimpleFrontiers();

        // 2. 逐个给观测点打分
        for (const auto &viewpoint : viewpoints)
        {
            std::vector<Eigen::Vector3d> viewpoint_visual_frontiers; // 每个视点对应能看到的frontiers

            // 遍历每个边界，判断“观测点能不能看到它”（3个条件都满足才算能看到）
            for (const auto &frontier : frontiers)
            {
                Eigen::Vector3d sensor_point(viewpoint.x(), viewpoint.y(), viewpoint.z()); // 可视比较的是sensor点。
                // 条件1：距离<传感器范围的80%（太远看不清）
                // 条件2：在无人机视角内（上视角/下视角以内，不看天上或地下）
                // 条件3：观测点到边界没有遮挡（直线无障碍物）
                if ((frontier - sensor_point).norm() < sensor_range * 0.8 &&
                    (frontier.z() > sensor_point.z()
                         ? fabs(frontier.z() - sensor_point.z()) / (frontier - sensor_point).norm() < tan(M_PI * viewpoint_manager_->vmp_.upper_fov_ / 180)
                         : fabs(frontier.z() - sensor_point.z()) / (frontier - sensor_point).norm() < tan(M_PI * viewpoint_manager_->vmp_.lower_fov_ / 180)) &&
                    frontier_finder_->sdf_map_->isCollisionFreeStraight(sensor_point, frontier))
                {
                    viewpoint_visual_frontiers.push_back(frontier); // 能看到的边界。
                }
            }
            // 打分：分数=能看到的边界数量 × 权重（1.0）
            tour_points_gains[viewpoint] = static_cast<double>(viewpoint_visual_frontiers.size()) * frontier_gain_;

            // Eigen::Vector3d angle_pos = map_2d_manager_->getAnglePos();
            // Eigen::Vector3d sensor_point(viewpoint.x(), viewpoint.y(), viewpoint.z());
            // double distance = (angle_pos-sensor_point).norm();
            // // 提高与障碍物边缘距离近的vp的增益
            // if(distance < sensor_range/2)
            //     tour_points_gains[viewpoint] *= 100;

            // 更新最高分数（知道哪个点最有价值）
            if (tour_points_gains[viewpoint] > max_gain)
            {
                max_gain = tour_points_gains[viewpoint];
            }
        }
    }

    //  多点规划：一次去多个观测点（找最短路线）
    void RapidCoverPlanner::planning(const geometry_msgs::Pose &current_pose, const utils::Point3D &current_directory, bool &is_successed)
    {
        solving_num_++;  // 规划次数+1

        // 1. 记规划开始时间（微秒转秒）
        double start_time_second = static_cast<double>(time_utils::Timer::GetTimeNow("us")) / 1e6;

        // 2. 调用Two-Opt算法找“去多个观测点的最短路线”
        if (two_opt_solve_planning(current_pose, current_directory))
        {
            if (viewpoint_manager_->viewpoints_attached_frontiers_.count(goal_point_) != 0)
            {
                goal_point_frontiers_ = viewpoint_manager_->viewpoints_attached_frontiers_.at(goal_point_);
            }
            else
            {
                goal_point_frontiers_.clear();  // 没关联边界→清空
            }
            ROS_INFO("this iteration planning successed."); // 打印“规划成功”
            is_successed = true;    // 告诉调用方“成功了”
        }
        else
        {
            goal_point_frontiers_.clear();
            ROS_WARN("this iteration planning failed");
            is_successed = false;
        }

        double finish_time_second = static_cast<double>(time_utils::Timer::GetTimeNow("us")) / 1e6;

        double iteration_time = finish_time_second - start_time_second; // 本次耗时
        sum_solving_time_ = sum_solving_time_ + iteration_time; // 累加总耗时
        file_utils::writeToFileByAdd(each_solving_txt_name_, "\t", start_time_second, finish_time_second, iteration_time,
                                     solving_num_, sum_solving_time_ / solving_num_, tour_points_.size());
    }

    //  单点规划：一次去一个最有价值的点
    void RapidCoverPlanner::singlePointPlanning(const geometry_msgs::Pose &current_pose, const utils::Point3D &current_directory, bool &is_successed)
    {
        // 1. 清空旧路径（避免用上次的路径）
        path_to_go_.clear();
        tsp_path_.clear();
        path_segments_.clear();
        is_successed = false;
        
        // vector<Vector3d> viewpoints, nearby_viewpoints;
        // vector<std::vector<int>> viewpoints_attached_frontiers, nearby_viewpoints_attached_frontiers;

        // Vector3d target_viewpoint;
        // std::vector<int> target_viewpoint_attached_frontiers;

        // 2. 拿当前位置（转成3D点和向量格式，方便计算）
        Point3D current_position(current_pose.position.x, current_pose.position.y, current_pose.position.z);
        Vector3d current_pos(current_pose.position.x, current_pose.position.y, current_pose.position.z);
        int current_point_id = addCurrentPositionToGraph(current_position);

        // frontier_finder_->getViewpointsAttachedFrontiers(viewpoints, viewpoints_attached_frontiers);
        
        // int viewpoint_num = viewpoints.size();
        
        // Path viewpoint_paths[viewpoint_num];
        // Path max_gain_path;
        
        // double viewpoint_distance[viewpoint_num];
        // double max_gain = -DBL_MAX;

        // int max_gain_index;
        // Point3D max_gain_viewpoint;

        // viewpoint_with_path_.clear();
        // viewpoints_gain_.clear();

        // int nearby_viewpoint_num = 0;
        // for (int i = 0; i < viewpoint_num; i++)
        // {
        //     Vector3d pos = viewpoints[i];
        //     if((pos-current_pos).norm() < (local_range_+7.5))
        //     {
        //         nearby_viewpoints.push_back(viewpoints[i]);
        //         nearby_viewpoints_attached_frontiers.push_back(viewpoints_attached_frontiers[i]);
        //     }
        // }

        // if(!nearby_viewpoints.empty())
        // {
        //     ROS_WARN("nearby_viewpoint_num:%d", nearby_viewpoints.size());
        //     for (int i = 0; i < nearby_viewpoints.size(); i++)
        //     {
        //         Path path;
        //         Vector3d pos = nearby_viewpoints[i];
        //         Point3D viewpoint(pos.x(), pos.y(), pos.z());
        //         path = getPathInGridMap3D(current_position, viewpoint, true);   // 首先在local_gridmap上找 
        //         if (path.empty())   // 在local_gridmap上找不到，再到roadgraph上找
        //         {
        //             // ROS_WARN("Find viewpoint path in roadmap");
        //             int viewpoint_id;
        //             if (road_graph_->getPointId(viewpoint, viewpoint_id))
        //             {
        //                 path = getPathInGraph(current_point_id, viewpoint_id);
        //             }
        //             else
        //             {
        //                 ROS_ERROR("Viewpoint not found in Map3D!");
        //             }
        //             if (path.empty())   // 在rodamap中搜
        //             {
        //                 ROS_WARN("Find path to viewpoint in RoadMap");
        //                 int b_id;
        //                 int target_id;
        //                 utils::Point3D nearest_point;
        //                 if (road_graph_->nearestSearch(viewpoint, nearest_point, b_id))
        //                 {
        //                     int target_id = road_graph_->addVertex(viewpoint);
        //                     road_graph_->addTwoWayEdge(target_id, b_id);
        //                     // ROS_WARN("Added target point and its edges in plan graph.");
        //                 }
        //                 path = getPathInGraph(current_point_id, target_id);
        //                 if (path.empty())
        //                 {
        //                     ROS_ERROR("Viewpoint not found in Roadmap!");
        //                     // path = getPathFromInflateRegion(current_position, init_point_);
        //                 }
        //             }
        //             // if (!road_graph_->getPointId(init_point_, init_point_id_))
        //             // {
        //             //     int init_point_id_ = road_graph_->addVertex(init_point_);
        //             //     int b_id;
        //             //     utils::Point3D nearest_point;
        //             //     if (road_graph_->nearestSearch(init_point_, nearest_point, b_id))
        //             //     {
        //             //         road_graph_->addTwoWayEdge(init_point_id_, b_id);
        //             //         ROS_INFO("added init_point_id_ and its edges in plan graph.");
        //             //     }
        //             // }
        //         }
        //         // 在local_gridmap上找不到，可能是当前走进了膨胀区，导致连不上graph，
        //         // 先找出离的近的roadmap point，使用A*先找出当前位置到该point的path，后续再在graph上找path
        //         if (path.empty())
        //         {
        //             // viewpoint_distance[i] = DBL_MAX;
        //             // path = getPathFromInflateRegion(current_position, tour_points_term.at(i));
        //         }
        //         else
        //         {   
        //             // ROS_WARN("Find viewpoint path successfully");
        //             double distance = 0;
        //             for(int j=0; j < path.size()-1; j++)
        //             {
        //                 distance += (path[j+1]-path[j]).norm();
        //             }
        //             Point3D diff_vector(viewpoint.x() - current_position.x(),
        //                                 viewpoint.y() - current_position.y(),
        //                                 viewpoint.z() - current_position.z());
        //             double theta = current_directory.angleTo(diff_vector);
        //             // distance = distance * ((1 - alpha_) * (std::abs(std::pow(theta, 2)) / (1 + std::abs(std::pow(theta, 2)))) + alpha_);
        //             distance = distance * ((1 - alpha_) * log2(abs(theta)/M_PI+1) + alpha_);
                    
        //             // double gain = nearby_viewpoints_attached_frontiers[i].size() * exp(-lambda_ * distance);
        //             double gain = -distance;
        //             ROS_WARN("viewpoints_attached_frontiers[i].size():%d", nearby_viewpoints_attached_frontiers[i].size());
        //             if(gain > max_gain)
        //             {
        //                 max_gain = gain;
        //                 max_gain_index = i;
        //                 max_gain_path = path;
        //                 max_gain_viewpoint = viewpoint;
        //                 is_successed = true;

        //                 target_viewpoint = nearby_viewpoints[i];
        //                 target_viewpoint_attached_frontiers = nearby_viewpoints_attached_frontiers[i];
        //             }
        //             // debug
        //             Vector3d viewpoint_pos(viewpoint.x(), viewpoint.y(), viewpoint.z());
        //             viewpoint_with_path_.push_back(viewpoint_pos);
        //             viewpoints_gain_.push_back(gain);
        //         }
                
        //         // ROS_INFO("path into matrix..");
        //         // viewpoint_paths[i] = path;
        //     }
        // }
        // 3. 找“最有价值的观测点”（从找边界的工具里拿）
        Vector3d target_viewpoint;  // 目标观测点
        vector<int> attach_frontiers;   // 这个点关联的边界ID
        Path path;  // 规划的路径
        if(frontier_finder_->getNextViewpoint(target_viewpoint, attach_frontiers))  // 成功拿到目标点
        {
            // 打印当前位置和目标点（调试用，让人知道要飞哪）
            ROS_WARN("Get target viewpoint x:%0.2f  y:%0.2f  z:%0.2f", target_viewpoint.x(), target_viewpoint.y(), target_viewpoint.z());
            ROS_WARN("Current pos          x:%0.2f  y:%0.2f  z:%0.2f", current_pos.x(), current_pos.y(), current_pos.z());
           
            // 告诉“找边界的工具”：“目标点定好了，就去这”
            frontier_finder_->setTargetViewpoint(target_viewpoint, attach_frontiers);

            // 把目标点转成3D点格式
            Point3D viewpoint(target_viewpoint.x(), target_viewpoint.y(), target_viewpoint.z());

            // 4. 用A*算法在3D地图里找“当前位置→目标点”的路径
            frontier_finder_->Astar_->reset();
            if (frontier_finder_->Astar_->search(current_pos, target_viewpoint) != fast_planner::Astar3::REACH_END)
            {
                ROS_ERROR("Find path from cur_pos to target point in grid map failed");
            }
        
             // 拿A*算好的路径
            auto astar_path = frontier_finder_->Astar_->getPath();

            // 把路径转成3D点格式，存到path里
            for(auto &point:astar_path)
            {
                path.push_back(Point3D(point.x(), point.y(), point.z()));
            }
            
            // 5. 如果A*没找到路径，就用“导航地图”找
            if (path.empty())   // 在local_gridmap上找不到，再到roadgraph上找
            {
                ROS_WARN("Target viewpoint not found in Map3D! Finding in RoadMap");
                int viewpoint_id;
                if (road_graph_->getPointId(viewpoint, viewpoint_id))
                {
                    path = getPathInGraph(current_point_id, viewpoint_id);
                }
                if (path.empty())   // 在rodamap中搜
                {
                    int b_id;
                    int target_id;
                    utils::Point3D nearest_point;
                    if (road_graph_->nearestSearch(viewpoint, nearest_point, b_id))
                    {
                        int target_id = road_graph_->addVertex(viewpoint);
                        road_graph_->addTwoWayEdge(target_id, b_id);
                        // ROS_WARN("Added target point and its edges in plan graph.");
                    }
                    path = getPathInGraph(current_point_id, target_id); // 找路径
                    if (path.empty())
                    {
                        ROS_ERROR("Target viewpoint not found in Roadmap!");
                        // path = getPathFromInflateRegion(current_position, init_point_);
                    }
                }
            }
            // 6. 路径有效→标记“规划成功”
            if(!path.empty())
            {
                is_successed = true;
                Point3D start = path.front();
                Point3D end = path.back();

                ROS_WARN("Path start x:%0.2f  y:%0.2f  z:%0.2f", start.x(), start.y(), start.z());
                ROS_WARN("Path end   x:%0.2f  y:%0.2f  z:%0.2f", end.x(), end.y(), end.z());
            }
                
        }
        else
        {
            ROS_ERROR("Get target viewpoint failed!");
        }

        //  规划成功→更新路径，画到RViz里
        if(is_successed)
        {
            // frontier_finder_->setTargetViewpoint(target_viewpoint, target_viewpoint_attached_frontiers);

            ROS_WARN("this iteration planning successed.");
            Path tsp_path;
            
            tsp_path.insert(tsp_path.end(), path.begin(), path.end());
            path_segments_.push_back(path);
            
            tsp_path_ = tsp_path;
            path_to_go_ = path;
            goal_point_ = target_viewpoint;
            ROS_INFO("the goal point is x= %f, y=%f, z=%f", goal_point_.x(), goal_point_.y(), goal_point_.z());

            pubPath(path);
        }
        else
        {
            ROS_WARN("this iteration planning failed");
        }

        pubViewpointWithGain();
        pubViewpointWithFrontiers(target_viewpoint, attach_frontiers);
       
    }

    void RapidCoverPlanner::pubViewpointWithFrontiers(const Vector3d &target_viewpoint, const vector<int> &attach_frontiers)
    {
        visualization_msgs::Marker line_list;
        line_list.header.frame_id = "world";    // 用世界坐标系（避免画歪）
        line_list.header.stamp = ros::Time::now();
        line_list.action = visualization_msgs::Marker::ADD;
        line_list.pose.orientation.w = 1.0;
        line_list.type = visualization_msgs::Marker::LINE_LIST;

        line_list.id = 0;
        // 设置 Marker 的比例（线条宽度）
        line_list.scale.x = 0.1;
        // 设置 Marker 的颜色（绿色）
        line_list.color.r = 0.0;
        line_list.color.g = 1.0;
        line_list.color.b = 0.0;
        line_list.color.a = 1.0;


        geometry_msgs::Point start;
        start.x = target_viewpoint.x();
        start.y = target_viewpoint.y();
        start.z = target_viewpoint.z();

        // 每个边界作为终点，画“观测点→边界”的线
        for(auto cell:attach_frontiers)
        {
            Vector3d pos = frontier_finder_->sdf_map_->AddressToPos(cell);
            geometry_msgs::Point end;
            end.x = pos.x();
            end.y = pos.y();
            end.z = pos.z();
            line_list.points.push_back(start);
            line_list.points.push_back(end);
        }
        viewpoint_with_frontiers_pub_.publish(line_list);
    }

    void RapidCoverPlanner::pubViewpointWithGain()
    {   
        if(!viewpoint_with_path_.empty())
        {
            pcl::PointCloud<pcl::PointXYZ> cloud;
            for (auto viewpoint:viewpoint_with_path_) 
            {   
                pcl::PointXYZ point;
                point.x = viewpoint.x();
                point.y = viewpoint.y();
                point.z = viewpoint.z();

                cloud.points.push_back(point);
            }
            sensor_msgs::PointCloud2 msg;
            pcl::toROSMsg(cloud, msg);  // 从PCL转换为ROS消息
            msg.header.frame_id = "world";  // 设置合适的frame_id
            msg.header.stamp = ros::Time::now();  // 设置时间戳

            viewpoint_with_path_pub_.publish(msg);  // 发布PointCloud2消息

            visualization_msgs::MarkerArray marker_array;
            visualization_msgs::Marker text_marker;
            text_marker.header.frame_id = "world";
            text_marker.header.stamp = ros::Time::now();
        
            for (size_t i = 0; i < viewpoint_with_path_.size(); ++i) 
            {
                text_marker.id = i;
                text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                text_marker.action = visualization_msgs::Marker::ADD;

                text_marker.pose.position.x = viewpoint_with_path_[i].x();
                text_marker.pose.position.y = viewpoint_with_path_[i].y();
                text_marker.pose.position.z = viewpoint_with_path_[i].z() + 1.0; // Slightly above the point
                text_marker.pose.orientation.w = 1.0;

                text_marker.scale.z = 0.7; // Text height

                text_marker.color.r = 1.0;
                text_marker.color.g = 1.0;
                text_marker.color.b = 1.0;
                text_marker.color.a = 1.0;

                std::ostringstream oss;
                oss << std::fixed << std::setprecision(1) << viewpoints_gain_[i];
                text_marker.text = oss.str();

                // text_marker.text = std::to_string(viewpoints_gain_[i]);

                marker_array.markers.push_back(text_marker);
            }

            viewpoints_gain_pub_.publish(marker_array);

        }
    }

    //画 “规划好的路径”
    void RapidCoverPlanner::pubPath(Path &path)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;   // 把路径转成点云（像撒芝麻一样，标出路径的每个点）
        for (auto way_point:path) 
        {   
            pcl::PointXYZ point;
            point.x = way_point.x();
            point.y = way_point.y();
            point.z = way_point.z();

            cloud.points.push_back(point);
        }
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(cloud, msg);  
        msg.header.frame_id = "world";  
        msg.header.stamp = ros::Time::now();  

        path_pub_.publish(msg);  
    }

    void RapidCoverPlanner::findPathToSingleViewpoint(const geometry_msgs::Pose &current_pose, const geometry_msgs::Pose &viewpoint)
    {
        Point3D current_position(current_pose.position.x, current_pose.position.y, current_pose.position.z);
        path_to_go_.clear();
        tsp_path_.clear();
        path_segments_.clear();

        // 拿当前位置和目标点（用户指定的观测点）
        int current_point_id = addCurrentPositionToGraph(current_position);
        Point3D target(viewpoint.position.x, viewpoint.position.y, viewpoint.position.z);
       
        // 按维度找路径（2D用2D地图，3D用3D地图）
        Path path;
        if (planner_dim_ == 2)
        {
            path = getPathInGridMap2D(current_position, target, true);
        }
        else if (planner_dim_ == 3)
        {
            path = getPathInGridMap3D(current_position, target, true);
        }
        else
        {
            ROS_ERROR("planner_dim_ is error!");
            std::cerr << planner_dim_ << std::endl;
            return;
        }

        if (path.empty())
        {
            ROS_WARN("find path in RoadMap");
      
            int target_id = road_graph_->addVertex(target); // 把目标点加进导航地图
            int b_id;
            utils::Point3D nearest_point;
            if (road_graph_->nearestSearch(target, nearest_point, b_id))
            {
                road_graph_->addTwoWayEdge(target_id, b_id);
                ROS_WARN("added target point and its edges in plan graph.");
            }
            
            path = getPathInGraph(current_point_id, target_id);
            if (path.empty())
            {
                // path = getPathFromInflateRegion(current_position, init_point_);
            }
        }
        if (path.empty())
        {
            ROS_ERROR_THROTTLE(1.0, "The path is still empty~~~~~~~~~~");
        }
        tsp_path_.insert(tsp_path_.end(), path.begin(), path.end());
        path_segments_.push_back(path);

        // if (fabs(current_position.x() - init_point_.x()) < stop_x_thre_ &&
        //     fabs(current_position.y() - init_point_.y()) < stop_y_thre_ &&
        //     fabs(current_position.z() - init_point_.z()) < stop_z_thre_)
        // {
        //     std_msgs::Bool stop_msg;
        //     stop_msg.data = true;
        //     stop_move_pub_.publish(stop_msg);
        // }
    }

    // 回 “起飞点”
    void RapidCoverPlanner::findPathBackToOrigin(const geometry_msgs::Pose &current_pose)
    {
        Point3D current_position(current_pose.position.x, current_pose.position.y, current_pose.position.z);
        path_to_go_.clear();
        tsp_path_.clear();
        path_segments_.clear();

        int current_point_id = addCurrentPositionToGraph(current_position);

        Path path;
        if (planner_dim_ == 2)
        {
            path = getPathInGridMap2D(current_position, init_point_, true);
        }
        else if (planner_dim_ == 3)
        {
            path = getPathInGridMap3D(current_position, init_point_, true);
        }
        else
        {
            ROS_ERROR("planner_dim_ is error!");
            std::cerr << planner_dim_ << std::endl;
            return;
        }

        // 地图找不到→用导航地图找
        if (path.empty())
        {
            ROS_INFO("find path in RoadMap");
            if (!road_graph_->getPointId(init_point_, init_point_id_))
            {
                int init_point_id_ = road_graph_->addVertex(init_point_);
                int b_id;
                utils::Point3D nearest_point;
                if (road_graph_->nearestSearch(init_point_, nearest_point, b_id))
                {
                    road_graph_->addTwoWayEdge(init_point_id_, b_id);
                    ROS_INFO("added init_point_id_ and its edges in plan graph.");
                }
            }

            path = getPathInGraph(current_point_id, init_point_id_);
            if (path.empty())
            {
                // path = getPathFromInflateRegion(current_position, init_point_);
            }
        }
        if (path.empty())
        {
            ROS_ERROR_THROTTLE(1.0, "The path is still empty~~~~~~~~~~");
        }
        tsp_path_.insert(tsp_path_.end(), path.begin(), path.end());
        path_segments_.push_back(path);

        // if (fabs(current_position.x() - init_point_.x()) < stop_x_thre_ &&
        //     fabs(current_position.y() - init_point_.y()) < stop_y_thre_ &&
        //     fabs(current_position.z() - init_point_.z()) < stop_z_thre_)
        // {
        //     std_msgs::Bool stop_msg;
        //     stop_msg.data = true;
        //     stop_move_pub_.publish(stop_msg);
        // }
    }

    //  Two-Opt 核心：找多个观测点的最短路线
    bool RapidCoverPlanner::two_opt_solve_planning(const geometry_msgs::Pose &current_pose, const utils::Point3D &current_directory)
    {
        Point3D current_position(current_pose.position.x, current_pose.position.y, current_pose.position.z);

        path_to_go_.clear();
        tsp_path_.clear();
        path_segments_.clear();
        if (tour_points_.empty())
        {
            return false;
        }
        else
        {
            // 清理pre_paths_
            std::vector<Point3D> need_to_erase;
            for (auto &item1 : pre_paths_)
            {
                // 出点为空，整条删去
                if (viewpoint_manager_->viewpoints_attached_frontiers_.count(item1.first) == 0)
                {
                    need_to_erase.push_back(item1.first);
                }
                else
                {
                    // 删去入点为空的部分
                    std::vector<Point3D> to_erase;
                    for (const auto &item2 : item1.second)
                    {
                        if (viewpoint_manager_->viewpoints_attached_frontiers_.count(item2.first) == 0 ||
                            item2.second.empty())
                        {
                            to_erase.push_back(item2.first);
                        }
                    }
                    for (const auto &point : to_erase)
                    {
                        item1.second.erase(point);
                    }
                }
            }
            for (const auto &point : need_to_erase)
            {
                pre_paths_.erase(point);
            }

            // 将当前点接入road_graph
            int current_point_id = addCurrentPositionToGraph(current_position);
            ROS_INFO("current position is x = %f, y= %f, z = %f, id is %d", current_position.x(), current_position.y(),
                     current_position.z(), current_point_id);

            Point3DQueue tour_points_term;
            tour_points_term.push_back(current_position);
            for (const auto &point : tour_points_)
            {
                tour_points_term.push_back(point);
            }
            int tour_points_num = tour_points_term.size();

            ROS_INFO("start get path maxtirx of tour points");

            // 初始化本次的path_matrix
            Path path_matrix[tour_points_num][tour_points_num];
            Path empty_path;
            for (int i = 0; i < tour_points_num; ++i)
            {
                for (int j = 0; j < tour_points_num; ++j)
                {
                    path_matrix[i][j] = empty_path;
                }
            }

            // 找出当前位置到tour_points的path
            for (int i = 1; i < tour_points_num; i++)
            {
                Path path;
                // 首先在local_gridmap上找
                if (planner_dim_ == 2)
                {
                    path = getPathInGridMap2D(current_position, tour_points_term.at(i), true);
                }
                else if (planner_dim_ == 3)
                {
                    path = getPathInGridMap3D(current_position, tour_points_term.at(i), true);
                }
                else
                {
                    ROS_ERROR("planner_dim_ is error!");
                    std::cerr << planner_dim_ << std::endl;
                    return false;
                }

                // 在local_gridmap上找不到，再到roadgraph上找
                if (path.empty())
                {
                    int end_point_id;
                    if (road_graph_->getPointId(tour_points_term.at(i), end_point_id))
                    {
                        path = getPathInGraph(current_point_id, end_point_id);
                    }
                    else
                    {
                        ROS_ERROR("end_point_id is not exist!");
                    }

                    // 在local_gridmap上找不到，可能是当前走进了膨胀区，导致连不上graph，
                    // 先找出离的近的roadmap point，使用A*先找出当前位置到该point的path，后续再在graph上找path
                    if (path.empty())
                    {
                        // path = getPathFromInflateRegion(current_position, tour_points_term.at(i));
                    }
                }
                ROS_INFO("path into matrix..");
                path_matrix[0][i] = path;
                std::reverse(path.begin(), path.end());
                path_matrix[i][0] = path;
            }

            // 找各个tour_points之间的paths
            ROS_INFO("start get other tour points paths..");
            for (int i = 1; i < tour_points_num; ++i)
            {
                for (int j = 1; j < tour_points_num; ++j)
                {
                    if (i == j)
                    {
                        path_matrix[i][j] = empty_path;
                    }
                    else if (path_matrix[i][j].empty())
                    {
                        // 先在gridmap中找path
                        Path path;
                        if (planner_dim_ == 2)
                        {
                            path = getPathInGridMap2D(tour_points_term.at(i), tour_points_term.at(j), false);
                        }
                        else if (planner_dim_ == 3)
                        {
                            path = getPathInGridMap3D(tour_points_term.at(i), tour_points_term.at(j), false);
                        }
                        else
                        {
                            ROS_ERROR("planner_dim_ is error!");
                            std::cerr << planner_dim_ << std::endl;
                            return false;
                        }

                        if (path.size() >= 2)
                        {
                            pre_paths_[tour_points_term.at(i)][tour_points_term.at(j)] = path;
                            auto path_reverse = path;
                            std::reverse(path_reverse.begin(), path_reverse.end());
                            pre_paths_[tour_points_term.at(j)][tour_points_term.at(i)] = path_reverse;
                        }

                        // gridmap找不到再到pre_paths_或roadmap中找
                        if (path.empty())
                        {
                            if (pre_paths_.count(tour_points_term.at(i)) != 0 &&
                                pre_paths_[tour_points_term.at(i)].count(tour_points_term.at(j)) != 0)
                            {
                                path = pre_paths_[tour_points_term.at(i)][tour_points_term.at(j)];
                            }
                            else
                            {
                                int start_point_id;
                                int end_point_id;
                                if (road_graph_->getPointId(tour_points_term.at(i), start_point_id) &&
                                    road_graph_->getPointId(tour_points_term.at(j), end_point_id))
                                {
                                    path = getPathInGraph(start_point_id, end_point_id);
                                    if (path.size() >= 2)
                                    {
                                        pre_paths_[tour_points_term.at(i)][tour_points_term.at(j)] = path;
                                        auto path_reverse = path;
                                        std::reverse(path_reverse.begin(), path_reverse.end());
                                        pre_paths_[tour_points_term.at(j)][tour_points_term.at(i)] = path_reverse;
                                    }
                                }
                                else
                                {
                                    ROS_ERROR("start_point_id or end_point_id can not get from graph!");
                                }
                            }
                        }
                        path_matrix[i][j] = path;
                        std::reverse(path.begin(), path.end());
                        path_matrix[j][i] = path;
                    }
                }
            }
            ROS_INFO("path matrix got, start get cost matrix...");

            // 计算各tourpoints之间的距离损失矩阵
            std::vector<std::vector<double>> cost_matrix = std::vector<std::vector<double>>(
                tour_points_num, std::vector<double>(tour_points_num, 1e10));
            for (int x = 1; x < tour_points_num; ++x)
            {
                for (int y = 1; y < tour_points_num; ++y)
                {
                    if (x == y)
                    {
                        cost_matrix[x][y] = 0.0;
                    }
                    else if (path_matrix[x][y].size() < 2)
                    {
                        cost_matrix[x][y] = 1e10;
                    }
                    else
                    {
                        double path_length = 0.0;
                        for (int k = 0; k < path_matrix[x][y].size() - 1; k++)
                        {
                            path_length = path_length + path_matrix[x][y][k].distance(path_matrix[x][y][k + 1]);
                        }
                        cost_matrix[x][y] = path_length;
                    }
                }
            }

            // 计算当前位置到各tourpoints之间的距离损失矩阵
            for (int y = 1; y < tour_points_num; y++)
            {
                if (path_matrix[0][y].size() < 2)
                {
                    cost_matrix[0][y] = 1e10;
                }
                else
                {
                    // 累计路径长度
                    double path_length = 0.0;
                    for (int k = 0; k < path_matrix[0][y].size() - 1; k++)
                    {
                        path_length = path_length + path_matrix[0][y][k].distance(path_matrix[0][y][k + 1]);
                    }
                    cost_matrix[0][y] = path_length;

                    // 考虑方向性惩罚
                    if (is_directory_)
                    {
                        if (tour_points_term[y].distance(current_position) < frontier_finder_->sdf_map_->getMaxRayLength())
                        {
                            Point3D diff_vector(tour_points_term[y].x() - current_position.x(),
                                                tour_points_term[y].y() - current_position.y(),
                                                tour_points_term[y].z() - current_position.z());
                            double theta = current_directory.angleTo(diff_vector);
                            // cost_matrix[0][y] = cost_matrix[0][y] * ((1 - alpha_) * (log(theta / M_PI + 1) / log(2)) + alpha_);
                            cost_matrix[0][y] = cost_matrix[0][y] * ((1 - alpha_) * (std::abs(std::pow(theta, 2)) / (1 + std::abs(std::pow(theta, 2)))) + alpha_);
                        }
                    }
                }
            }
            for (int x = 0; x < tour_points_num; ++x)
            {
                cost_matrix[x][0] = 0;
            }

            // 开始规划
            ROS_INFO("cost matrix got, start planning..");
            std::vector<std::vector<int>> nodes_indexs;
            std::vector<int> ids;
            for (int i = 0; i < tour_points_num; ++i)
            {
                ids.push_back(i);
            }

            // two-opt solver ..
            ROS_INFO("start 2-opt solver..");
            auto start_time = ros::WallTime::now();

            std::vector<double> gains;
            gains.push_back(0.0);
            for (int i = 1; i < tour_points_num; ++i)
            {
                gains.push_back(tour_points_gains_[tour_points_term[i]]);
            }

            std::vector<int> init_route = ids;
            Two_Opt two_opt_solve(init_route, gains, cost_matrix, lambda_);
            two_opt_solve.solve();
            auto max_unity_way = two_opt_solve.best_route_;

            auto finish_time = ros::WallTime::now();
            double iteration_time = (finish_time - start_time).toSec();
            sum_two_opt_time = sum_two_opt_time + iteration_time;
            // std::ofstream fout;
            // fout.open(two_opt_time_name_, std::ios_base::in | std::ios_base::out | std::ios_base::app);
            // fout << solving_num_ << "\t" << iteration_time << "\t"
            //      << sum_two_opt_time / solving_num_ << "s \t" << max_unity_way.size() - 1 << "\t"
            //      << std::endl;
            // fout.close();

            ROS_INFO(
                "2-opt solver plan is finished, spent %.10f s, finish path node num is %zu, total searched ways size is %d",
                iteration_time, max_unity_way.size(), two_opt_solve.searched_route_num_);
            ROS_INFO("the best route is:");
            for (auto &i : max_unity_way)
            {
                std::cout << i << "\t";
            }
            std::cout << std::endl;
            ROS_INFO("the best unity is %.10f", two_opt_solve.best_unity_);

            if (max_unity_way.size() > 1)
            {
                Path tsp_path;
                for (int i = 0; i < max_unity_way.size() - 1; i++)
                {
                    tsp_path.insert(tsp_path.end(),
                                    path_matrix[max_unity_way[i]][max_unity_way[i + 1]].begin(),
                                    path_matrix[max_unity_way[i]][max_unity_way[i + 1]].end());
                    path_segments_.push_back(path_matrix[max_unity_way[i]][max_unity_way[i + 1]]);
                }
                tsp_path_ = tsp_path;
                path_to_go_ = path_matrix[0][max_unity_way[1]];
                goal_point_ = tour_points_term.at(max_unity_way[1]);
                ROS_INFO("the goal point is x= %f, y=%f, z=%f", goal_point_.x(), goal_point_.y(), goal_point_.z());
                return true;
            }
            else
            {
                tsp_path_.clear();
                return false;
            }
        }
    }

    int RapidCoverPlanner::addCurrentPositionToGraph(const Point3D &current_position)
    {
        ROS_INFO("add current position to the plan map");
        int current_point_id = -1;

        if (!road_graph_->getPointId(current_position, current_point_id))
        {
            double local_range = 1.0;
            Point3DQueue neighbor_vertexs;
            std::vector<int> neighbor_vertex_ids;
            do
            {
                road_graph_->nearRangeSearch(current_position, local_range,
                                             neighbor_vertexs, neighbor_vertex_ids);
                local_range += 1.0;
            } while (neighbor_vertex_ids.empty());

            current_point_id = road_graph_->addVertex(current_position);

            for (const auto &item : neighbor_vertexs)
            {
                if (map_3d_manager_->isCollisionFreeStraight(
                        GridPoint3D(current_position.x(), current_position.y(), current_position.z()),
                        GridPoint3D(item.x(), item.y(), item.z())))
                {
                    int item_id = -1;
                    if (current_point_id != item_id && road_graph_->getPointId(item, item_id))
                    {
                        road_graph_->addTwoWayEdge(current_point_id, item_id);
                    }
                    else
                    {
                        std::cerr << "addCurrentPositionToGraph fail!" << std::endl;
                    }
                }
            }
        }
        return current_point_id;
    }

    Path RapidCoverPlanner::getPathFromInflateRegion(const Point3D &start_point, const Point3D &end_point)
    {
        // 首先先找出当前位置到该point的path
        Path path;
        Point3D break_vertex;
        Point3DQueue neighbor_vertexs;
        std::vector<int> neighbor_vertex_ids;

        double local_range = road_graph_->tgp_.sample_dist_h_;
        do
        {
            // 找出local_range范围内的点

            road_graph_->nearRangeSearch(start_point, local_range, neighbor_vertexs, neighbor_vertex_ids);
            if (!neighbor_vertexs.empty())
            {
                // 使用A*先找出当前位置到该point的path
                for (const auto &vertex : neighbor_vertexs)
                {
                    if (planner_dim_ == 2)
                    {
                        path = getPathInGridMap2D(start_point, vertex, true);
                    }
                    else if (planner_dim_ == 3)
                    {
                        path = getPathInGridMap3D(start_point, vertex, true);
                    }
                    else
                    {
                        ROS_ERROR("planner_dim_ is error: %d", planner_dim_);
                        path.clear();
                        return path;
                    }

                    if (!path.empty())
                    {
                        break_vertex = vertex;
                        break;
                    }
                }
            }
            local_range += 1.0;
        } while (path.empty() && local_range < road_graph_->tgp_.sample_dist_h_ * 1.5);

        // 然后在graph上找path
        int break_point_id, end_point_id;
        if (road_graph_->getPointId(break_vertex, break_point_id) &&
            road_graph_->getPointId(end_point, end_point_id))
        {
            Path path2 = getPathInGraph(break_point_id, end_point_id);
            path.insert(path.end(), path2.begin(), path2.end());
        }
        else
        {
            ROS_ERROR("break_point_id or end_point_id is not exist!");
            path.clear();
        }

        return path;
    }

    Path RapidCoverPlanner::getPathInGraph(const int &start_point_id, const int &end_point_id)
    {
        Path path;
        road_graph_->findShortestPath(start_point_id, end_point_id, path);

        ROS_INFO("the path size is %zu", path.size());

        return path;
    }

    Path RapidCoverPlanner::getPathInGridMap2D(const Point3D &start_point, const Point3D &end_point,
                                               const bool &is_clear_nearby)
    {
        Point2D start_2d(start_point.x(), start_point.y());
        Point2D end_2d(end_point.x(), end_point.y());
        Path path_3d;

        std::vector<Point2D> optimal_path_2d;
        ROS_INFO("start search shortest path..");
        if (map_2d_manager_->getShortestOptimalPath(end_2d, start_2d, is_clear_nearby, optimal_path_2d))
        {
            for (const auto &point : optimal_path_2d)
            {
                path_3d.emplace_back(point.x(), point.y(), init_z_);
            }
        }
        else
        {
            ROS_INFO("start point or end point is out of grid map 2d or occupancy");
            path_3d.clear();
        }

        return path_3d;
    }

    Path RapidCoverPlanner::getPathInGridMap3D(const Point3D &start_point, const Point3D &end_point,
                                               const bool &is_clear_nearby)
    {
        GridPoint3D start_3d(start_point.x(), start_point.y(), start_point.z());
        GridPoint3D end_3d(end_point.x(), end_point.y(), end_point.z());
        std::vector<GridPoint3D> path_3d;
        Path path;

        if (map_3d_manager_->getShortestPath(end_3d, start_3d, is_clear_nearby, path_3d))
        {
            for (const auto &point : path_3d)
            {
                path.emplace_back(point.x(), point.y(), point.z());
            }
        }
        else
        {
            // ROS_WARN("Finding path in Grid3D failed!");
            path.clear();
        }
        return path;
    }
}