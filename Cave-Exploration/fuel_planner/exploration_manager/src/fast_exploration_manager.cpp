#include <thread>
#include <iostream>
#include <fstream>
#include <lkh_tsp_solver/lkh_interface.h>
#include <active_perception/graph_node.h>
#include <active_perception/graph_search.h>
#include <active_perception/perception_utils.h>
#include <plan_env/raycast.h>
#include <plan_env/sdf_map.h>
#include <plan_env/edt_environment.h>
#include <active_perception/frontier_finder.h>
#include <plan_manage/planner_manager.h>

#include <exploration_manager/fast_exploration_manager.h>
#include <exploration_manager/expl_data.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
// #include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>

// #include <control_planner_interface/Path.h>

#include "time_utils/time_utils.h"

using namespace Eigen;

namespace fast_planner
{
  // SECTION interfaces for setup and query

  FastExplorationManager::FastExplorationManager()
  {
  }

  FastExplorationManager::~FastExplorationManager()
  {
    ViewNode::astar_.reset();
    ViewNode::caster_.reset();
    ViewNode::map_.reset();
  }

  void FastExplorationManager::initialize(ros::NodeHandle &nh)
  {
    planner_manager_.reset(new FastPlannerManager);
    planner_manager_->initPlanModules(nh);
    edt_environment_ = planner_manager_->edt_environment_;
    sdf_map_ = edt_environment_->sdf_map_;
    frontier_finder_.reset(new FrontierFinder(sdf_map_, nh));
    
    Eigen::Vector3d bmin, bmax;
    sdf_map_->getBox(bmin, bmax);
    road_map_ = std::make_shared<preprocess::IkdTreeTopoGraph>(sdf_map_, nh);
    road_map_->setSamplePointsBound(bmax, bmin);

    frontier_finder_->setRoadMap(road_map_);
    // view_finder_.reset(new ViewFinder(edt_environment_, nh));
    
    ed_.reset(new ExplorationData);
    ep_.reset(new ExplorationParam);
    fep_.reset(new FAELExplorationParams);
    fed_.reset(new FAELExplorationData);

    is_next_pos_need_modify_ = false;

    next_pos_need_modify_cnt_ = 0;

    loadParams(nh);
    fed_->iteration_num_ = 0;
    fed_->is_exploration_finished_ = false;
    fed_->is_finished_ = false;
    fed_->is_iteration_goal_scaned_ = false;
    fed_->local_plan_start_time_ = std::numeric_limits<double>::max();
    fed_->global_plan_start_time_ = std::numeric_limits<double>::max();
    fed_->is_start_bspline_ = false;
    fed_->new_traj_ = false;

    ViewNode::astar_.reset(new Astar);
    ViewNode::astar_->init(nh, edt_environment_);
    ViewNode::map_ = sdf_map_;

    double resolution_ = sdf_map_->getResolution();
    Eigen::Vector3d origin, size;
    sdf_map_->getRegion(origin, size);
    ViewNode::caster_.reset(new RayCaster);
    ViewNode::caster_->setParams(resolution_, origin);

    planner_manager_->path_finder_->lambda_heu_ = 1.0;
    // planner_manager_->path_finder_->max_search_time_ = 0.05;
    planner_manager_->path_finder_->max_search_time_ = 1.0;
    planner_manager_->path_finder_FAEL_->lambda_heu_ = 1.0;
    // planner_manager_->path_finder_->max_search_time_ = 0.05;
    planner_manager_->path_finder_FAEL_->max_search_time_ = 1.0;

    // Initialize TSP par file
    ofstream par_file(ep_->tsp_dir_ + "/single.par");
    par_file << "PROBLEM_FILE = " << ep_->tsp_dir_ << "/single.tsp\n";
    par_file << "GAIN23 = NO\n";
    par_file << "OUTPUT_TOUR_FILE =" << ep_->tsp_dir_ << "/single.txt\n";
    par_file << "RUNS = 1\n";

    next_pose_pub_ = nh.advertise<nav_msgs::Path>("/next_point", 1);
    // Analysis
    // ofstream fout;
    // fout.open("/home/boboyu/Desktop/RAL_Time/frontier.txt");
    // fout.close();
  }

  void FastExplorationManager::loadParams(ros::NodeHandle &nh)
  {
    if (!nh.param("exploration/use_fuel_planner", ep_->use_fuel_planner_, true))
    {
      ROS_WARN("Can not find param exploration/use_fuel_planner.");
    }

    nh.param("exploration/init_x", ep_->init_x, 0.0);
    nh.param("exploration/init_y", ep_->init_y, 0.0);
    nh.param("exploration/init_z", ep_->init_z, 2.0);
    nh.param("exploration/refine_local", ep_->refine_local_, true);
    nh.param("exploration/refined_num", ep_->refined_num_, -1);
    nh.param("exploration/refined_radius", ep_->refined_radius_, -1.0);
    nh.param("exploration/top_view_num", ep_->top_view_num_, -1);
    nh.param("exploration/max_decay", ep_->max_decay_, -1.0);
    nh.param("exploration/tsp_dir", ep_->tsp_dir_, string("null"));
    nh.param("exploration/relax_time", ep_->relax_time_, 1.0);

    nh.param("exploration/vm", ViewNode::vm_, -1.0);
    nh.param("exploration/am", ViewNode::am_, -1.0);
    nh.param("exploration/yd", ViewNode::yd_, -1.0);
    nh.param("exploration/ydd", ViewNode::ydd_, -1.0);
    nh.param("exploration/w_dir", ViewNode::w_dir_, -1.0);

    if (!nh.param("fael_exploration/init_motion_enable", fep_->init_motion_enable_, false))
    {
      ROS_WARN("No motion initialization setting, set it to False.");
    }

    if (!nh.param("fael_exploration/init_x", fep_->init_x_, 0.0))
    {
      ROS_WARN("No init_x specified. Default is '0.0'.");
    }

    if (!nh.param("fael_exploration/init_y", fep_->init_y_, 0.0))
    {
      ROS_WARN("No init_y specified. Default is '0.0'.");
    }

    if (!nh.param("fael_exploration/init_z", fep_->init_z_, 0.0))
    {
      ROS_WARN("No init_z specified. Default is '0.0'.");
    }

    if (!nh.param("fael_exploration/max_vel", fep_->max_vel_, 1.0))
    {
      ROS_WARN("No max_vel specified. Default is '1.0'.");
    }

    if (!nh.param("fael_exploration/replan_too_long_thre", fep_->replan_too_long_thre_, 1.0))
    {
      ROS_WARN("No replan_too_long_thre specified. Default is '1.0'.");
    }

    if (!nh.param("fael_exploration/local_replan_periodic_thre", fep_->local_replan_periodic_thre_, 2.0))
    {
      ROS_WARN("No local_replan_periodic_thre specified. Default is '2.0'.");
    }

    if (!nh.param("fael_exploration/global_replan_periodic_thre", fep_->global_replan_periodic_thre_, 5.0))
    {
      ROS_WARN("No global_replan_periodic_thre specified. Default is '5.0'.");
    }
  }

  int FastExplorationManager::planExploreMotion(const Vector3d &pos, const Vector3d &vel,
                                                const Vector3d &acc, const Vector3d &yaw)
  {
    ros::Time t1 = ros::Time::now();
    auto t2 = t1;
    Vector3d next_pos;
    double next_yaw;
    ed_->views_.clear();
    ed_->global_tour_.clear();

    std::cout << "start pos: " << pos.transpose() << ", vel: " << vel.transpose()
              << ", acc: " << acc.transpose() << std::endl;

    // 1.Search frontiers and group them into clusters
    frontier_finder_->searchFrontiers();

    double frontier_time = (ros::Time::now() - t1).toSec();
    t1 = ros::Time::now();

    // 2.Find viewpoints (x,y,z,yaw) for all frontier clusters and get visible ones' info
    frontier_finder_->computeFrontiersToVisit();
    frontier_finder_->getFrontiers(ed_->frontiers_);
    frontier_finder_->getFrontierBoxes(ed_->frontier_boxes_);
    frontier_finder_->getDormantFrontiers(ed_->dead_frontiers_);

    if (ed_->frontiers_.empty())
    {
      ROS_WARN("No coverable frontier.");
      return NO_FRONTIER;
    }
    // 每个frontier都只找一个viewpoints
    // Retrieve the first viewpoint that is far enough and has highest coverage
    frontier_finder_->getTopViewpointsInfo(pos, ed_->points_, ed_->yaws_, ed_->averages_);
    for (int i = 0; i < ed_->points_.size(); ++i)
      ed_->views_.push_back(
          ed_->points_[i] + 2.0 * Vector3d(cos(ed_->yaws_[i]), sin(ed_->yaws_[i]), 0));

    double view_time = (ros::Time::now() - t1).toSec();
    ROS_WARN(
        "Frontier: %d, t: %lf, viewpoint: %d, t: %lf", ed_->frontiers_.size(), frontier_time,
        ed_->points_.size(), view_time);

    // 3.Do global and local tour planning and retrieve the next viewpoint

    if (ed_->points_.size() > 1)
    {
      // Find the global tour passing through all viewpoints
      // Create TSP and solve by LKH
      // Optimal tour is returned as indices of frontier
      vector<int> indices;
      findGlobalTour(pos, vel, yaw, indices);

      if (ep_->refine_local_)
      {
        // Do refinement for the next few viewpoints in the global tour
        // -Idx of the first K frontier in optimal tour
        // -所有的local tour_vp都在ep_->refined_radius_范围内 && ed_->refined_ids_.size() >= 2
        t1 = ros::Time::now();

        ed_->refined_ids_.clear();
        ed_->unrefined_points_.clear();
        int knum = min(int(indices.size()), ep_->refined_num_);
        for (int i = 0; i < knum; ++i)
        {
          auto tmp = ed_->points_[indices[i]];     // tmp是第一个被访问的vp
          ed_->unrefined_points_.push_back(tmp);   // ed_->unrefined_points_记录截取的global path的vp
          ed_->refined_ids_.push_back(indices[i]); // 记录截取的global path对应的id，也是访问各个frontier的顺序
          if ((tmp - pos).norm() > ep_->refined_radius_ && ed_->refined_ids_.size() >= 2)
            break;
        }

        // Get top N viewpoints for the next K frontiers
        ed_->n_points_.clear();
        vector<vector<double>> n_yaws;
        frontier_finder_->getViewpointsInfo(
            pos, ed_->refined_ids_, ep_->top_view_num_, ep_->max_decay_, ed_->n_points_, n_yaws);

        ed_->refined_points_.clear();
        ed_->refined_views_.clear();
        vector<double> refined_yaws;
        refineLocalTour(pos, vel, yaw, ed_->n_points_, n_yaws, ed_->refined_points_, refined_yaws);
        next_pos = ed_->refined_points_[0];
        next_yaw = refined_yaws[0];

        // Get marker for view visualization
        for (int i = 0; i < ed_->refined_points_.size(); ++i)
        {
          Vector3d view =
              ed_->refined_points_[i] + 2.0 * Vector3d(cos(refined_yaws[i]), sin(refined_yaws[i]), 0);
          ed_->refined_views_.push_back(view);
        }
        ed_->refined_views1_.clear();
        ed_->refined_views2_.clear();
        for (int i = 0; i < ed_->refined_points_.size(); ++i)
        {
          vector<Vector3d> v1, v2;
          frontier_finder_->percep_utils_->setPose(ed_->refined_points_[i], refined_yaws[i]);
          frontier_finder_->percep_utils_->getFOV(v1, v2);
          ed_->refined_views1_.insert(ed_->refined_views1_.end(), v1.begin(), v1.end());
          ed_->refined_views2_.insert(ed_->refined_views2_.end(), v2.begin(), v2.end());
        }
        double local_time = (ros::Time::now() - t1).toSec();
        ROS_WARN("Local refine time: %lf", local_time);
      }
      else
      {
        // Choose the next viewpoint from global tour
        next_pos = ed_->points_[indices[0]];
        next_yaw = ed_->yaws_[indices[0]];
      }
    }
    else if (ed_->points_.size() == 1)
    {
      // Only 1 destination, no need to find global tour through TSP
      ed_->global_tour_ = {pos, ed_->points_[0]};
      ed_->refined_tour_.clear();
      ed_->refined_views1_.clear();
      ed_->refined_views2_.clear();

      if (ep_->refine_local_)
      {
        // Find the min cost viewpoint for next frontier
        ed_->refined_ids_ = {0};
        ed_->unrefined_points_ = {ed_->points_[0]};
        ed_->n_points_.clear();
        vector<vector<double>> n_yaws;
        frontier_finder_->getViewpointsInfo(
            pos, {0}, ep_->top_view_num_, ep_->max_decay_, ed_->n_points_, n_yaws);

        double min_cost = 100000;
        int min_cost_id = -1;
        vector<Vector3d> tmp_path;
        for (int i = 0; i < ed_->n_points_[0].size(); ++i)
        {
          auto tmp_cost = ViewNode::computeCost(
              pos, ed_->n_points_[0][i], yaw[0], n_yaws[0][i], vel, yaw[1], tmp_path);
          if (tmp_cost < min_cost)
          {
            min_cost = tmp_cost;
            min_cost_id = i;
          }
        }
        next_pos = ed_->n_points_[0][min_cost_id];
        next_yaw = n_yaws[0][min_cost_id];
        ed_->refined_points_ = {next_pos};
        ed_->refined_views_ = {next_pos + 2.0 * Vector3d(cos(next_yaw), sin(next_yaw), 0)};
      }
      else
      {
        next_pos = ed_->points_[0];
        next_yaw = ed_->yaws_[0];
      }
    }
    else
      ROS_ERROR("Empty destination.");

    std::cout << "Next view: " << next_pos.transpose() << ", " << next_yaw << std::endl;

    // 4.Plan trajectory (position and yaw) to the next viewpoint
    t1 = ros::Time::now();

    // Compute time lower bound of yaw and use in trajectory generation
    double diff = fabs(next_yaw - yaw[0]);
    double time_lb = min(diff, 2 * M_PI - diff) / ViewNode::yd_;

    // Generate trajectory of x,y,z------A*算法
    planner_manager_->path_finder_->reset();
    // 首先使用A*在sdf_map中找到最短路径
    if (planner_manager_->path_finder_->search(pos, next_pos) != Astar::REACH_END)
    {
      ROS_ERROR("No path to next viewpoint");
      return FAIL;
    }
    ed_->path_next_goal_ = planner_manager_->path_finder_->getPath();
    shortenPath(ed_->path_next_goal_); // Eigen::vector3d

    const double radius_far = 5.0;
    const double radius_close = 1.5;
    const double len = Astar::pathLength(ed_->path_next_goal_);
    if (len < radius_close)
    {
      // Next viewpoint is very close, no need to search kinodynamic path, just use waypoints-based
      // optimization
      planner_manager_->planExploreTraj(ed_->path_next_goal_, vel, acc, time_lb);
      ed_->next_goal_ = next_pos;
    }
    else if (len > radius_far)
    {
      // Next viewpoint is far away, select intermediate goal on geometric path (this also deal with
      // dead end)
      std::cout << "Far goal." << std::endl;
      double len2 = 0.0;
      vector<Eigen::Vector3d> truncated_path = {ed_->path_next_goal_.front()};
      for (int i = 1; i < ed_->path_next_goal_.size() && len2 < radius_far; ++i)
      {
        auto cur_pt = ed_->path_next_goal_[i];
        len2 += (cur_pt - truncated_path.back()).norm();
        truncated_path.push_back(cur_pt);
      }
      ed_->next_goal_ = truncated_path.back();
      planner_manager_->planExploreTraj(truncated_path, vel, acc, time_lb);
      // if (!planner_manager_->kinodynamicReplan(
      //         pos, vel, acc, ed_->next_goal_, Vector3d(0, 0, 0), time_lb))
      //   return FAIL;
      // ed_->kino_path_ = planner_manager_->kino_path_finder_->getKinoTraj(0.02);
    }
    else
    {
      // Search kino path to exactly next viewpoint and optimize
      std::cout << "Mid goal" << std::endl;
      ed_->next_goal_ = next_pos;

      if (!planner_manager_->kinodynamicReplan(
              pos, vel, acc, ed_->next_goal_, Vector3d(0, 0, 0), time_lb))
      {
        return FAIL;
      }
    }

    if (planner_manager_->local_data_.position_traj_.getTimeSum() < time_lb - 0.1)
      ROS_ERROR("Lower bound not satified!");

    planner_manager_->planYawExplore(yaw, next_yaw, true, ep_->relax_time_);

    double traj_plan_time = (ros::Time::now() - t1).toSec();
    t1 = ros::Time::now();

    double yaw_time = (ros::Time::now() - t1).toSec();
    // ROS_WARN("Traj: %lf, yaw: %lf", traj_plan_time, yaw_time);
    double total = (ros::Time::now() - t2).toSec();
    // ROS_WARN("Total time: %lf", total);
    ROS_ERROR_COND(total > 0.1, "Total time too long!!!");

    return SUCCEED;
  }

  // void FastExplorationManager::planLookAheadPose(const PLAN_STATE &plan_state, geometry_msgs::Pose &target_pose,
  //                                                bool &is_success, bool &is_finished)
  // {
  //   // 1. 初始化一些变量
  //   is_success = false;
  //   is_finished = false;

  //   if (plan_state != PLAN_STATE::GLOBAL && plan_state != PLAN_STATE::LOCAL)
  //     return;

  //   // 2.如果探索结束，判断是否需要停止
  //   static bool expl_finish_flag = false;
  //   if (fed_->is_exploration_finished_)
  //   {
  //     geometry_msgs::Point cur_position = fed_->current_pose_.position;
  //     if (fabs(cur_position.x - fep_->init_x_) < planner_manager_->topo_planner_->planner_->stop_x_thre_ &&
  //         fabs(cur_position.y - fep_->init_y_) < planner_manager_->topo_planner_->planner_->stop_y_thre_ &&
  //         fabs(cur_position.z - fep_->init_z_) < planner_manager_->topo_planner_->planner_->stop_z_thre_)
  //     {
  //       ROS_WARN("exploration_finished and need to stop!~~");
  //       is_success = false;
  //       is_finished = true;
  //       fed_->is_finished_ = true;
  //       return;
  //     }
  //   }

  //   fed_->iteration_num_++;
  //   ROS_INFO("**Planning iteration  %i**", fed_->iteration_num_);

  //   // 3.开始规划
  //   if (plan_state == PLAN_STATE::GLOBAL)
  //   {
  //     bool is_plan_success = false;
  //     bool is_exploration_finished = false;

  //     fed_->global_plan_start_time_ = ros::WallTime::now().toSec();
  //     planner_manager_->topo_planner_->planGlobalPathNew(global_path_segments_, is_plan_success, is_exploration_finished);

  //     if (!is_plan_success)
  //     {
  //       if (is_exploration_finished)
  //         fed_->is_exploration_finished_ = true;

  //       // exploration failed
  //       is_success = false;
  //       is_finished = false;
  //     }
  //   }

  //   // 4.局部目标点决策
  //   ROS_INFO("following the path...");
  //   // got a reasonable path
  //   bool success = false;
  //   double executed_start_time, executed_path_length;
  //   planner_manager_->path_execution_->findLlookAheadPoint(global_path_segments_, success, executed_start_time,
  //                                                          executed_path_length, target_pose);

  //   if (success)
  //   {
  //     fed_->local_plan_start_time_ = executed_start_time;
  //     is_success = true;
  //     is_finished = false;
  //   }
  //   else
  //   {
  //     is_success = false;
  //     is_finished = false;
  //   }
  // }

  int FastExplorationManager::planExploreMotionFromLookAheadPose(const Vector3d &pos, const Vector3d &vel, const Vector3d &acc,
                                                                 const Vector3d &yaw, const Vector3d &look_ahead_goal, const double &look_ahead_yaw)
  {
    static bool is_change_sdf_mode = false;
    if (!is_change_sdf_mode)
    {
      sdf_map_->setChangeCollectionMode(sdf_map_->ONLY_UPDATED);
      is_change_sdf_mode = true;
    }

    ros ::Time t1 = ros::Time::now();
    auto t2 = t1;
    Vector3d next_pos;
    double next_yaw;

    if (is_next_pos_need_modify_)
    {
      // TODO look_ahead_goal_越界约束修改
      ROS_WARN("next_pos_need_modify_~!");
      // next_pos = Eigen::Vector3d(ep_->init_x, ep_->init_y, ep_->init_z);
      next_pos = look_ahead_goal;
      next_yaw = look_ahead_yaw;
    }
    else
    {
      next_pos = look_ahead_goal;
      next_yaw = look_ahead_yaw;
    }

    t1 = ros::Time::now();
    
    // DEBUG:将目标点发布出来，用于测试运动规划算法
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "world";
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "world";
    pose.pose.position.x = next_pos.x();
    pose.pose.position.y = next_pos.y();
    pose.pose.position.z = next_pos.z();

    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    path.poses.push_back(pose);

    next_pose_pub_.publish(path);


    
    // DEBUG

    // Compute time lower bound of yaw and use in trajectory generation
    double diff = fabs(next_yaw - yaw[0]);
    double time_lb = min(diff, 2 * M_PI - diff) / ViewNode::yd_;

    // Generate trajectory of x,y,z------A*算法
    planner_manager_->path_finder_FAEL_->reset();
    // 首先使用A*在sdf_map中找到最短路径
    ROS_WARN("next_pos:(%f, %f, %f)", next_pos.x(), next_pos.y(), next_pos.z());
    if (planner_manager_->path_finder_FAEL_->search(pos, next_pos) != Astar::REACH_END)
    {
      ROS_ERROR("No path to next viewpoint");

      is_next_pos_need_modify_ = true;
      ++next_pos_need_modify_cnt_;
      return FAIL;
    }
    is_next_pos_need_modify_ = false;
    next_pos_need_modify_cnt_ = 0;
    ed_->path_next_goal_FAEL_ = planner_manager_->path_finder_FAEL_->getPath();
    // 将前面A*搜索到的最短路径的点数量尽可能降低
    shortenPath(ed_->path_next_goal_FAEL_); // Eigen::vector3d

    const double radius_far = 50.0;
    const double radius_close = 1.0;
    const double len = Astar::pathLength(ed_->path_next_goal_FAEL_);
    if (len < radius_close)
    {
      ROS_ERROR("Close goal");
      // Next viewpoint is very close, no need to search kinodynamic path, just use waypoints-based
      // optimization
      planner_manager_->planExploreTraj(ed_->path_next_goal_FAEL_, vel, acc, time_lb);
      ed_->next_goal_FAEL_ = next_pos;
    }
    else if (len > radius_far)
    {
      // Next viewpoint is far away, select intermediate goal on geometric path (this also deal with
      // dead end)
      ROS_ERROR("Far goal");
      std::cout << "Far goal." << std::endl;
      double len2 = 0.0;
      vector<Eigen::Vector3d> truncated_path = {ed_->path_next_goal_FAEL_.front()};
      for (int i = 1; i < ed_->path_next_goal_FAEL_.size() && len2 < radius_far; ++i)
      {
        auto cur_pt = ed_->path_next_goal_FAEL_[i];
        len2 += (cur_pt - truncated_path.back()).norm();
        truncated_path.push_back(cur_pt);
      }
      ed_->next_goal_FAEL_ = truncated_path.back();
      planner_manager_->planExploreTraj(truncated_path, vel, acc, time_lb);
      // if (!planner_manager_->kinodynamicReplan(
      //         pos, vel, acc, ed_->next_goal_, Vector3d(0, 0, 0), time_lb))
      //   return FAIL;
      // ed_->kino_path_ = planner_manager_->kino_path_finder_->getKinoTraj(0.02);
    }
    else
    {
      // Search kino path to exactly next viewpoint and optimize
      ROS_ERROR("Mid goal");
      std::cout << "Mid goal" << std::endl;
      ed_->next_goal_FAEL_ = next_pos;

      if (!planner_manager_->kinodynamicReplan(
              pos, vel, acc, ed_->next_goal_FAEL_, Vector3d(0, 0, 0), time_lb))
      {
        return FAIL;
      }
      // ROS_WARN("*************6****************");
    }

    if (planner_manager_->local_data_.position_traj_.getTimeSum() < time_lb - 0.1)
    {
      // ROS_WARN("*************7****************");
      ROS_ERROR("Lower bound not satified!");
      return FAIL;
    }

    planner_manager_->planYawExplore(yaw, next_yaw, true, ep_->relax_time_);

    double traj_plan_time = (ros::Time::now() - t1).toSec();
    t1 = ros::Time::now();

    double yaw_time = (ros::Time::now() - t1).toSec();
    // ROS_WARN("Traj: %lf, yaw: %lf", traj_plan_time, yaw_time);
    double total = (ros::Time::now() - t2).toSec();
    // ROS_WARN("Total time: %lf", total);
    ROS_ERROR_COND(total > 0.1, "Total time too long!!!");

    return SUCCEED;
  }

  void FastExplorationManager::shortenPath(vector<Vector3d> &path)
  {
    if (path.empty())
    {
      ROS_ERROR("Empty path to shorten");
      return;
    }
    // Shorten the tour, only critical intermediate points are reserved.
    // 对于原始path中dist_thresh范围内的第i个point，只有short_tour最后一个point到
    // 原始path的第i+1个point的raycast出现不可通行的地带，这原始path的第i个point才会
    // 被被加入到short_tour中，否则只会加入超出dist_thresh的point
    const double dist_thresh = 3.0;
    vector<Vector3d> short_tour = {path.front()};
    for (int i = 1; i < path.size() - 1; ++i)
    {
      if ((path[i] - short_tour.back()).norm() > dist_thresh)
        short_tour.push_back(path[i]);
      else
      {
        // Add waypoints to shorten path only to avoid collision
        ViewNode::caster_->input(short_tour.back(), path[i + 1]);
        Eigen::Vector3i idx;
        while (ViewNode::caster_->nextId(idx) && ros::ok())
        {
          if (edt_environment_->sdf_map_->getInflateOccupancy(idx) == 1 ||
              edt_environment_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN)
          {
            short_tour.push_back(path[i]);
            break;
          }
        }
      }
    }
    // 加上终点point
    if ((path.back() - short_tour.back()).norm() > 1e-3)
      short_tour.push_back(path.back());

    // Ensure at least three points in the path
    if (short_tour.size() == 2)
      short_tour.insert(short_tour.begin() + 1, 0.5 * (short_tour[0] + short_tour[1]));
    path = short_tour;
  }

  void FastExplorationManager::shortenPath2(vector<Vector3d> &path)
  {
    if (path.empty())
    {
      ROS_ERROR("Empty path to shorten");
      return;
    }
    // Shorten the tour, only critical intermediate points are reserved.
    // 对于原始path中dist_thresh范围内的第i个point，只有short_tour最后一个point到
    // 原始path的第i+1个point的raycast出现不可通行的地带，这原始path的第i个point才会
    // 被被加入到short_tour中，否则只会加入超出dist_thresh的point
    // const double dist_thresh = 3.0;
    vector<Vector3d> short_tour = {path.front()};
    for (int i = 1; i < int(path.size()) - 1; ++i)
    {
      // Add waypoints to shorten path only to avoid collision
      ViewNode::caster_->input(short_tour.back(), path[i + 1]);
      Eigen::Vector3i idx;
      while (ViewNode::caster_->nextId(idx) && ros::ok())
      {
        if (edt_environment_->sdf_map_->getInflateOccupancy(idx) == 1 ||
            edt_environment_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN)
        {
          short_tour.push_back(path[i]);
          break;
        }
      }
    }
    // 加上终点point
    if ((path.back() - short_tour.back()).norm() > 1e-3)
      short_tour.push_back(path.back());


    // 对间距过大点进行插值
    vector<Vector3d> new_path;

    for (size_t i = 0; i < short_tour.size() - 1; ++i) 
    {
      // 当前点和下一个点
      Vector3d cur = short_tour[i];
      Vector3d next = short_tour[i + 1];

      // 计算两点之间的距离
      double distance = (next - cur).norm();

      // 将当前点加入新路径
      new_path.push_back(cur);

      // 如果距离大于阈值，开始插值
      int threshold = 0.6;
      if (distance > threshold) 
      {
        // 计算需要插入的点数
        int num_points = static_cast<int>(distance / threshold);
        // 插值生成中间点
        for (int j = 1; j <= num_points; ++j) 
        {
            double t = static_cast<double>(j) / (num_points + 1); // 线性插值比例
            Vector3d interpolated_point = (1.0 - t) * cur + t * next;
            new_path.push_back(interpolated_point);
        }
      }
    }

    // 将最后一个点加入新路径
    if (!short_tour.empty()) {
        new_path.push_back(short_tour.back());
    }


    // // Ensure at least three points in the path
    // if (short_tour.size() == 2)
    //   short_tour.insert(short_tour.begin() + 1, 0.5 * (short_tour[0] + short_tour[1]));
    path = new_path;
  }

  void FastExplorationManager::findGlobalTour(
      const Vector3d &cur_pos, const Vector3d &cur_vel, const Vector3d cur_yaw,
      vector<int> &indices)
  {
    auto t1 = ros::Time::now();

    // Get cost matrix for current state and clusters
    Eigen::MatrixXd cost_mat;
    frontier_finder_->updateFrontierCostMatrix();
    frontier_finder_->getFullCostMatrix(cur_pos, cur_vel, cur_yaw, cost_mat);
    const int dimension = cost_mat.rows();

    double mat_time = (ros::Time::now() - t1).toSec();
    t1 = ros::Time::now();

    // Write params and cost matrix to problem file
    ofstream prob_file(ep_->tsp_dir_ + "/single.tsp");
    // Problem specification part, follow the format of TSPLIB

    string prob_spec = "NAME : single\nTYPE : ATSP\nDIMENSION : " + to_string(dimension) +
                       "\nEDGE_WEIGHT_TYPE : "
                       "EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\nEDGE_WEIGHT_SECTION\n";

    // string prob_spec = "NAME : single\nTYPE : TSP\nDIMENSION : " + to_string(dimension) +
    //     "\nEDGE_WEIGHT_TYPE : "
    //     "EXPLICIT\nEDGE_WEIGHT_FORMAT : LOWER_ROW\nEDGE_WEIGHT_SECTION\n";

    prob_file << prob_spec;
    // prob_file << "TYPE : TSP\n";
    // prob_file << "EDGE_WEIGHT_FORMAT : LOWER_ROW\n";
    // Problem data part
    const int scale = 100;
    if (false)
    {
      // Use symmetric TSP
      for (int i = 1; i < dimension; ++i)
      {
        for (int j = 0; j < i; ++j)
        {
          int int_cost = cost_mat(i, j) * scale;
          prob_file << int_cost << " ";
        }
        prob_file << "\n";
      }
    }
    else
    {
      // Use Asymmetric TSP
      for (int i = 0; i < dimension; ++i)
      {
        for (int j = 0; j < dimension; ++j)
        {
          int int_cost = cost_mat(i, j) * scale;
          prob_file << int_cost << " ";
        }
        prob_file << "\n";
      }
    }

    prob_file << "EOF";
    prob_file.close();

    // Call LKH TSP solver
    solveTSPLKH((ep_->tsp_dir_ + "/single.par").c_str());

    // Read optimal tour from the tour section of result file
    ifstream res_file(ep_->tsp_dir_ + "/single.txt");
    string res;
    while (getline(res_file, res))
    {
      // Go to tour section
      if (res.compare("TOUR_SECTION") == 0)
        break;
    }

    if (false)
    {
      // Read path for Symmetric TSP formulation
      getline(res_file, res); // Skip current pose
      getline(res_file, res);
      int id = stoi(res);
      bool rev = (id == dimension); // The next node is virutal depot?

      while (id != -1)
      {
        indices.push_back(id - 2);
        getline(res_file, res);
        id = stoi(res);
      }
      if (rev)
        reverse(indices.begin(), indices.end());
      indices.pop_back(); // Remove the depot
    }
    else
    {
      // Read path for ATSP formulation
      while (getline(res_file, res))
      {
        // Read indices of frontiers in optimal tour
        int id = stoi(res);
        if (id == 1) // Ignore the current state
          continue;
        if (id == -1)
          break;
        indices.push_back(id - 2); // Idx of solver-2 == Idx of frontier
      }
    }

    res_file.close();

    // Get the path of optimal tour from path matrix
    frontier_finder_->getPathForTour(cur_pos, indices, ed_->global_tour_);

    double tsp_time = (ros::Time::now() - t1).toSec();
    ROS_WARN("Cost mat: %lf, TSP: %lf", mat_time, tsp_time);
  }

  void FastExplorationManager::refineLocalTour(
      const Vector3d &cur_pos, const Vector3d &cur_vel, const Vector3d &cur_yaw,
      const vector<vector<Vector3d>> &n_points, const vector<vector<double>> &n_yaws,
      vector<Vector3d> &refined_pts, vector<double> &refined_yaws)
  {
    double create_time, search_time, parse_time;
    auto t1 = ros::Time::now();

    // Create graph for viewpoints selection
    GraphSearch<ViewNode> g_search;
    vector<ViewNode::Ptr> last_group, cur_group;

    // Add the current state
    ViewNode::Ptr first(new ViewNode(cur_pos, cur_yaw[0]));
    first->vel_ = cur_vel;
    g_search.addNode(first);
    last_group.push_back(first);
    ViewNode::Ptr final_node;

    // Add viewpoints
    std::cout << "Local tour graph: ";
    for (int i = 0; i < n_points.size(); ++i)
    {
      // Create nodes for viewpoints of one frontier
      for (int j = 0; j < n_points[i].size(); ++j)
      {
        ViewNode::Ptr node(new ViewNode(n_points[i][j], n_yaws[i][j]));
        g_search.addNode(node);
        // Connect a node to nodes in last group
        for (auto nd : last_group)
          g_search.addEdge(nd->id_, node->id_);
        cur_group.push_back(node);

        // Only keep the first viewpoint of the last local frontier
        if (i == n_points.size() - 1)
        {
          final_node = node;
          break;
        }
      }
      // Store nodes for this group for connecting edges
      std::cout << cur_group.size() << ", ";
      last_group = cur_group;
      cur_group.clear();
    }
    std::cout << "" << std::endl;
    create_time = (ros::Time::now() - t1).toSec();
    t1 = ros::Time::now();

    // Search optimal sequence
    vector<ViewNode::Ptr> path;
    g_search.DijkstraSearch(first->id_, final_node->id_, path);

    search_time = (ros::Time::now() - t1).toSec();
    t1 = ros::Time::now();

    // Return searched sequence
    for (int i = 1; i < path.size(); ++i)
    {
      refined_pts.push_back(path[i]->pos_);
      refined_yaws.push_back(path[i]->yaw_);
    }

    // Extract optimal local tour (for visualization)
    ed_->refined_tour_.clear();
    ed_->refined_tour_.push_back(cur_pos);
    ViewNode::astar_->lambda_heu_ = 1.0;
    ViewNode::astar_->setResolution(0.2);
    for (auto pt : refined_pts)
    {
      vector<Vector3d> path;
      if (ViewNode::searchPath(ed_->refined_tour_.back(), pt, path))
        ed_->refined_tour_.insert(ed_->refined_tour_.end(), path.begin(), path.end());
      else
        ed_->refined_tour_.push_back(pt);
    }
    ViewNode::astar_->lambda_heu_ = 10000;

    parse_time = (ros::Time::now() - t1).toSec();
    // ROS_WARN("create: %lf, search: %lf, parse: %lf", create_time, search_time, parse_time);
  }

} // namespace fast_planner
