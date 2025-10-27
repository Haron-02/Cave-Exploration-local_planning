// “拓扑规划器（TopoPlanner）” 的核心逻辑
// 无人机 / 机器人 “探索任务的总指挥”：接收探索任务、协调底层规划工具
// 判断探索是否完成、生成具体的飞行路径，并把路径交给执行器
//
// Created by hjl on 2022/1/11.
//

#include "topo_planner/topo_planner.h"

namespace topo_planner
{

    TopoPlanner::TopoPlanner(
        ros::NodeHandle &nh_private) : nh_private_(nh_private),
                                       preprocess_inited_(false), exploration_finish_(false), is_failure_timer_start_(false),
                                       planner_action_server_(nh_private_, "/topo_planner/topo_planner",
                                                              boost::bind(&TopoPlanner::plannerCallback, this, _1),
                                                              false)
    {
    }

    void TopoPlanner::init()
    {
        failure_timer_.reset(new time_utils::Timer("failure_timer_"));

        setPlannerDim(this->elements_->perception_dim_);

        planner_ = std::make_shared<rapid_cover_planner::RapidCoverPlanner>(nh_private_);
        planner_->setPlannerDim(this->elements_->perception_dim_);
        planner_->setFrontierFinder(elements_->frontier_finder_);
        if (planner_dim_ == 2)
        {
            planner_->setMap2DManager(elements_->map_2d_manager_);
        }
        else if (planner_dim_ == 3)
        {
            planner_->setMap3DManager(elements_->map_3d_manager_);
        }
        else
        {
            ROS_ERROR("planner_dim_ is error!");
            std::cerr << planner_dim_ << std::endl;
        }

        planner_->setViewpointManager(elements_->viewpoint_manager_);
        planner_->setTopoGraph(elements_->road_map_);

        ros::Duration(0.5).sleep();
        ros::spinOnce();

        explore_finish_sub_ = nh_private_.subscribe<std_msgs::Bool>("/exploration_data/exploration_data_finish", 1,
                                                                    &TopoPlanner::explorationFinishCallback,
                                                                    this);
        goal_scan_pub_ = nh_private_.advertise<std_msgs::Bool>("/topo_planner/goal_scan", 1);

        // goal_scan_timer_ = nh_private_.createTimer(ros::Duration(0.1), &TopoPlanner::goalScanCallback, this, false, false);

        planner_action_server_.start();

        is_surface_follow_ = true;
    }

    void TopoPlanner::setPlannerDim(const int &dim)
    {
        this->planner_dim_ = dim;
    }

    void TopoPlanner::setPreprocessElements(const preprocess::Preprocess::Ptr &elements)
    {
        this->elements_ = elements;
    }

    void TopoPlanner::explorationFinishCallback(const std_msgs::BoolConstPtr &finish)
    {
        exploration_finish_ = finish->data;
    }

    void TopoPlanner::planGlobalPathNew(std::vector<control_planner_interface::Path> &path_segments,
                                     bool &is_success, bool &is_exploration_finished)
    {
        path_segments.clear();
        is_success = false;
        is_exploration_finished = false;

        // // 1.上线程锁
        // elements_->elements_update_mutex_.lock();

        // 2.初始化planner

        geometry_msgs::Pose current_pose(elements_->current_pose_);
        utils::Point3D current_position(elements_->current_position_);
        utils::Point3D forward_directory(elements_->forward_directory_);

        planner_->Initialize(current_position);
        preprocess_inited_ = true;

        ROS_WARN_THROTTLE(0.5, "current_position:(%f, %f, %f).", elements_->current_position_.x(),
                          elements_->current_position_.y(), elements_->current_position_.z());


        // 3.2.探索过程
        
        if(planner_->frontier_finder_->isDetectObstacle())
        {
            ROS_INFO("start planning ...");

            planner_->singlePointPlanning(current_pose, forward_directory, is_success);
            if (is_success)
            {
                is_failure_timer_start_ = false;
                failure_timer_->Stop();
                ROS_INFO("planner way pose generate, result return");
                for (int i = 0; i < planner_->path_segments_.size(); i++)
                {
                    control_planner_interface::Path path_segment;
                    path_segment.path = wayPoseGeneration(planner_->path_segments_[i]);
                    path_segments.push_back(path_segment);
                }
                ROS_INFO("the iteration planning finish");
            }
            else
            {
                if (planner_->tsp_path_.empty())
                {
                    if (!is_failure_timer_start_)
                    {
                        failure_timer_->Start();
                    }
                    is_failure_timer_start_ = true;
                    ROS_WARN("planner get a empty path");
                    path_segments.clear();
                    is_success = false;
                    if (static_cast<double>(failure_timer_->GetTimeNow("ms") - failure_timer_->getStartTime("ms")) / 1e3 > 1.0)
                    {
                        exploration_finish_ = true;
                        is_exploration_finished = true;
                        ROS_ERROR("Can't find a path to another observation point, exploration is over");
                    }
                }
            }
        }
                            
        
        
        // // 5.解线程锁
        // elements_->elements_update_mutex_.unlock();
    }

    void TopoPlanner::planGlobalPath(std::vector<control_planner_interface::Path> &path_segments,
                                     bool &is_success, bool &is_exploration_finished)
    {
        path_segments.clear();
        is_success = false;
        is_exploration_finished = false;

        // // 1.上线程锁
        // elements_->elements_update_mutex_.lock();

        // 2.初始化planner

        geometry_msgs::Pose current_pose(elements_->current_pose_);
        utils::Point3D current_position(elements_->current_position_);
        utils::Point3D forward_directory(elements_->forward_directory_);

        planner_->Initialize(current_position);
        preprocess_inited_ = true;

        ROS_WARN_THROTTLE(0.5, "current_position:(%f, %f, %f).", elements_->current_position_.x(),
                          elements_->current_position_.y(), elements_->current_position_.z());

        // 3.规划开始
        if (exploration_finish_)
        {
            // 3.1.探索完毕返回起点
            is_exploration_finished = true;
            ROS_WARN_THROTTLE(1.0, "~~~~~~~~~~exploration_finish_~~~~~~~~~~~");
            planner_->findPathBackToOrigin(elements_->current_pose_);

            if (planner_->tsp_path_.empty())
            {
                ROS_WARN("planner get a empty path");
                path_segments.clear();
                is_success = false;
            }
            else
            {
                ROS_INFO("planner way pose generate, result return");
                for (int i = 0; i < planner_->path_segments_.size(); i++)
                {
                    // calculate the pose of the path point
                    control_planner_interface::Path path_segment;
                    path_segment.path = wayPoseGeneration(planner_->path_segments_[i]);
                    path_segments.push_back(path_segment);
                }
                is_success = true;
            }
            ROS_INFO("the iteration planning finish");
        }
        else
        {
            // 3.2.探索过程
            if (!preprocess_inited_)
            {
                ROS_WARN("preprocess not finish , waitting..");
                is_success = false;
            }
            // else if (isExplorationFinish())
            // {
            //     ROS_ERROR("tourpoints is empty, planning finish..");
            //     is_success = false;
            //     exploration_finish_ = true;
            //     is_exploration_finished = true;
            // }
            else
            {
                // Vector3d max_gain_viewpoint;
                // if(planner_->frontier_finder_->getLargestGainViewpoint(max_gain_viewpoint))
                // {
                //     geometry_msgs::Pose target_viewpoint;
                //     target_viewpoint.position.x = max_gain_viewpoint.x();
                //     target_viewpoint.position.y = max_gain_viewpoint.y();
                //     target_viewpoint.position.z = max_gain_viewpoint.z();
                //     planner_->findPathToSingleViewpoint(elements_->current_pose_, target_viewpoint);

                //     if (planner_->tsp_path_.empty())
                //     {
                //         ROS_WARN("planner get a empty path");
                //         path_segments.clear();
                //         is_success = false;
                //     }
                //     else
                //     {
                //         ROS_INFO("planner way pose generate, result return");
                //         for (int i = 0; i < planner_->path_segments_.size(); i++)
                //         {
                //             // calculate the pose of the path point
                //             control_planner_interface::Path path_segment;
                //             path_segment.path = wayPoseGeneration(planner_->path_segments_[i]);
                //             path_segments.push_back(path_segment);
                //         }
                //         is_success = true;
                //     }
                //     ROS_INFO("the iteration planning finish");
                // }
                
                ROS_INFO("start planning ...");

                planner_->planning(current_pose, forward_directory, is_success);
                if (is_success)
                {
                    is_failure_timer_start_ = false;
                    failure_timer_->Stop();
                    ROS_INFO("planner way pose generate, result return");
                    for (int i = 0; i < planner_->path_segments_.size(); i++)
                    {
                        control_planner_interface::Path path_segment;
                        path_segment.path = wayPoseGeneration(planner_->path_segments_[i]);
                        path_segments.push_back(path_segment);
                    }
                    ROS_INFO("the iteration planning finish");
                }
                else
                {
                    if (planner_->tsp_path_.empty())
                    {
                        if (!is_failure_timer_start_)
                        {
                            failure_timer_->Start();
                        }
                        is_failure_timer_start_ = true;
                        ROS_WARN("planner get a empty path");
                        path_segments.clear();
                        is_success = false;
                        if (static_cast<double>(failure_timer_->GetTimeNow("ms") - failure_timer_->getStartTime("ms")) / 1e3 > 1.0)
                        {
                            exploration_finish_ = true;
                            is_exploration_finished = true;
                            ROS_ERROR("Can't find a path to another observation point, exploration is over");
                        }
                    }
                }
            }
        }

        // // 5.解线程锁
        // elements_->elements_update_mutex_.unlock();
    }

    void TopoPlanner::plannerCallback(const control_planner_interface::ExplorerPlannerGoalConstPtr &goal)
    {
        // goal_scan_timer_.start();
        bool is_success = false, is_exploration_finished = false;
        std::vector<control_planner_interface::Path> path_segments;

        control_planner_interface::ExplorerPlannerResult result;

        planGlobalPath(path_segments, is_success, is_exploration_finished);
        if (is_success)
        {
            result.paths = path_segments;
            result.is_exploration_finish = is_exploration_finished;
        }
        else
        {
            path_segments.clear();
            result.is_exploration_finish = is_exploration_finished;
        }
        planner_action_server_.setSucceeded(result);
    }

    std::vector<geometry_msgs::Pose> TopoPlanner::wayPoseGeneration(rapid_cover_planner::Path &path)
    {
        std::vector<geometry_msgs::Pose> way_poses;
        utils::Point3D current = elements_->current_position_;

        way_poses.clear();
        if (!path.empty())
        {
            for (int i = 0; i < path.size(); i++)
            {
                tf::Vector3 point(path[i].x(), path[i].y(), path[i].z());

                tf::Vector3 axis(0, 0, 1);
                double angle;
                if (path.size() == 1)
                {
                    angle = std::atan2(path[i].y() - current.y(),
                                       path[i].x() - current.x());
                }
                else if ((i + 1) < path.size())
                {
                    angle = std::atan2((path[i + 1].y() - path[i].y()),
                                       (path[i + 1].x() - path[i].x()));
                }
                else
                {
                    angle = std::atan2((path[i].y() - path[i - 1].y()),
                                       (path[i].x() - path[i - 1].x()));
                }
                tf::Quaternion quaternion(axis, angle);
                tf::Pose poseTF(quaternion, point);
                geometry_msgs::Pose way_pose;
                tf::poseTFToMsg(poseTF, way_pose);

                way_poses.push_back(way_pose);
            }
        }
        return way_poses;
    }

    bool TopoPlanner::isExplorationFinish()
    {
        if (planner_->tour_points_.empty())
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    bool TopoPlanner::isCurrentGoalScanned()
    {
        if (elements_->viewpoint_manager_->candidate_viewpoints_.count(planner_->goal_point_) == 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    void TopoPlanner::goalScanCallback(const ros::TimerEvent &e)
    {
        std_msgs::Bool msg;
        msg.data = isCurrentGoalScanned() ? true : false;
        if (msg.data)
        {
            goal_scan_pub_.publish(msg);
        }
    }
}