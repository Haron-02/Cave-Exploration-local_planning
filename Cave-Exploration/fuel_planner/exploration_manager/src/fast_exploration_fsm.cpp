
#include <plan_manage/planner_manager.h>
#include <exploration_manager/fast_exploration_manager.h>
#include <traj_utils/planning_visualization.h>

#include <exploration_manager/fast_exploration_fsm.h>
#include <exploration_manager/expl_data.h>
#include <plan_env/edt_environment.h>
#include <plan_env/sdf_map.h>
#include "file_utils/file_rw.h"

#include "visualization_tools/IterationTime.h"

// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/PointStamped.h>

using Eigen::Vector4d;

namespace fast_planner
{
  void FastExplorationFSM::init(ros::NodeHandle &nh)
  {
    fp_.reset(new FSMParam);
    fd_.reset(new FSMData);

    /*  Fsm param  */
    nh.param("fsm/thresh_replan1", fp_->replan_thresh1_, -1.0);
    nh.param("fsm/thresh_replan2", fp_->replan_thresh2_, -1.0);
    nh.param("fsm/thresh_replan3", fp_->replan_thresh3_, -1.0);
    nh.param("fsm/replan_time", fp_->replan_time_, -1.0);
    nh.param("fsm/use_init_motion", fp_->use_init_motion_, false);
    nh.param("fsm/show_road_map", fp_->show_road_map_, true);
    

    /* Initialize main modules */
    expl_manager_.reset(new FastExplorationManager);
    expl_manager_->initialize(nh);
    visualization_.reset(new PlanningVisualization(nh));

    planner_manager_ = expl_manager_->planner_manager_;
    state_ = EXPL_STATE::INIT;
    plan_state_ = PLAN_STATE::GLOBAL;
    fd_->have_odom_ = false;
    fd_->state_str_ = {"INIT", "WAIT_TRIGGER", "PLAN_TRAJ", "PUB_TRAJ", "EXEC_TRAJ", "FINISH"};
    fd_->static_state_ = true;
    fd_->trigger_ = false;
    fd_->flight_length_ = 0;
    fd_->iter_num_ = 0;
    fd_->total_exploration_time_= 0;
    is_get_look_ahead_goal_ = false;
    planning_num_ = 0;
    sum_planning_time_ = 0;

    /* Ros sub, pub and timer */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &FastExplorationFSM::explorationFSMCallback, this);
    // safety_timer_ = nh.createTimer(ros::Duration(0.05), &FastExplorationFSM::safetyCallback, this);

    traj_pub_timer_ = nh.createTimer(ros::Duration(0.05), &FastExplorationFSM::trajPubCallback, this);
    traj_pub_ = nh.advertise<visualization_msgs::Marker>("/planning/traj", 10);

    next_pos_pub_ = nh.advertise<nav_msgs::Path>("/next_point", 1);

    trigger_sub_ =
        nh.subscribe("/waypoint_generator/waypoints", 1, &FastExplorationFSM::triggerCallback, this);
    odom_sub_ = nh.subscribe("/odom_world", 1, &FastExplorationFSM::odometryCallback, this);
    explore_finish_sub_ = nh.subscribe<std_msgs::Bool>("/exploration_data/exploration_data_finish", 1,
                                                       &FastExplorationFSM::explorationFinishCallback,
                                                       this);

    traj_sub_ = nh.subscribe("/traj_waypoints", 1, &FastExplorationFSM::trajCallback, this);

    end_time_sub_ = nh.subscribe("/end_time", 1, &FastExplorationFSM::endTimeCallback, this);

    forced_back_to_origin_sub_ = nh.subscribe("/move_base_simple/goal", 1, &FastExplorationFSM::forcedBackToOriginCallback, this);


    start_explore_pub_ = nh.advertise<std_msgs::Float64>("/exploration_data/explorer_start", 1);

    finish_explore_pub_ = nh.advertise<std_msgs::Float64>("/exploration_data/explorer_finish", 1);

    iteration_time_pub_ = nh.advertise<visualization_tools::IterationTime>("/planning/iteration_time", 1);

    next_tour_point_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/planning/next_tour_point", 1);
   
    next_point_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/planning/next_point_pub_", 1);

    box_pub_ = nh.advertise<visualization_msgs::Marker>("/planning/box_marker", 1);

    replan_pub_ = nh.advertise<std_msgs::Empty>("/planning/replan", 10);
    new_pub_ = nh.advertise<std_msgs::Empty>("/planning/new", 10);
    bspline_pub_ = nh.advertise<bspline::Bspline>("/planning/bspline", 10);

    topo_planner_trigger_client_ = nh.serviceClient<fael_exploration::Trigger>("/topo_planner/trigger");
    bspline_client_ = nh.serviceClient<bspline::BsplineProcess>("/planning/bspline_service");
  
    initStatistic(nh);
  }

  void FastExplorationFSM::initStatistic(ros::NodeHandle &nh)
  {
    exp_time_pub_ = nh.advertise<std_msgs::Float64>("/statistics/exp_time", 1);
    update_graph_time_pub_ = nh.advertise<std_msgs::Float64>("/statistics/update_graph_time", 1);
    detect_obstacle_time_pub_ = nh.advertise<std_msgs::Float64>("/statistics/detect_obstacle_time", 1);
    detect_global_frontier_time_pub_ = nh.advertise<std_msgs::Float64>("/statistics/detect_global_frontier_time", 1);
    cluster_frontier_time_pub_ = nh.advertise<std_msgs::Float64>("/statistics/cluster_frontier_time", 1);
    sample_viewpoint_time_pub_ = nh.advertise<std_msgs::Float64>("/statistics/sample_viewpoint_time", 1);
    cluster_viewpoint_time_pub_ = nh.advertise<std_msgs::Float64>("/statistics/cluster_viewpoint_time", 1);
    global_planning_time_pub_ = nh.advertise<std_msgs::Float64>("/statistics/global_planning_time", 1);
    viewpoint_refine_time_pub_ = nh.advertise<std_msgs::Float64>("/statistics/viewpoint_refine_time", 1);

    visualize_time_pub_ = nh.advertise<std_msgs::Float64>("/statistics/visualize_time", 1);

    flight_length_pub_ = nh.advertise<std_msgs::Float64>("/statistics/flight_length", 1);

    sd_.reset(new StatisticalData);
    std::string start_time, scene;
    nh.param("analysis/start_time", start_time, string("null"));
    nh.param("analysis/scene", scene, string("null"));

    std::time_t time = static_cast<std::time_t>(std::stoll(start_time));
    std::tm* local_time = std::localtime(&time);
    std::ostringstream oss;
    oss << std::put_time(local_time, "%Y_%m_%d-%H_%M_%S");
    start_time =  oss.str();

    string pkg_path = ros::package::getPath("exploration_manager");
    string dir_path = pkg_path + "/../../exploration_data_files/" + scene + "/" + start_time;
    file_utils::createDirectory(dir_path);

    start_time_txt_ = dir_path + "/start_time.txt";
    file_utils::writeToFileByTrunc(start_time_txt_, string("\t"), "start_time");
    
    module_time_txt_ = dir_path + "/module_time.txt";
    file_utils::writeToFileByTrunc(module_time_txt_, string("\t"),
      "iter", "duration", "exp", "update_graph", "detect_obstacle", "detect_global_frontier", "cluster_frontier", 
      "sample_viewpoint", "cluster_viewpoint", "global_planning", "viewpoint_refine", "visualization");

    module_avg_max_time_txt_ = dir_path + "/module_time_avg_max.txt";
    file_utils::writeToFileByTrunc(module_avg_max_time_txt_, string("\t"),
      "exp", "update_graph", "detect_obstacle", "detect_global_frontier", "cluster_frontier", 
      "sample_viewpoint", "cluster_viewpoint", "global_planning", "viewpoint_refine", "update_graph", "visualization");

    motion_data_txt_name_ = dir_path + "/motion_data.txt";
    file_utils::writeToFileByTrunc(motion_data_txt_name_, string("\t"),
      "iter", "length", "duration", "vel_avg", "pos.x", "pos.z", "vel", "vel.x", "vel.y", "vel.z", "cur_time");

    visualization_time_txt_name_ = dir_path + "/visualization.txt";
    file_utils::writeToFileByTrunc(visualization_time_txt_name_, std::string("\t"),
                                   "visulize_time");
  }

  void FastExplorationFSM::explorationFSMCallback(const ros::TimerEvent &e)
  {
    static double last_update_time;
    // 输出FSM当前状态
    ROS_INFO_STREAM_THROTTLE(1.0, "[FSM]: state: " << fd_->state_str_[int(state_)]);
    // 根据不同状态做不同的事情
    switch (state_)
    {
    case INIT:
    {
      // Wait for odometry ready
      if (!fd_->have_odom_)
      {
        ROS_WARN_THROTTLE(1.0, "no odom.");
        return;
      }

      // 等待subscriber连接
      if (!(start_explore_pub_.getNumSubscribers() > 0))
      {
        return;
      }
      // publish exploration status
      expl_manager_->fed_->start_explore_time_ = static_cast<double>(time_utils::Timer::GetTimeNow("us")) / 1e6;
      std_msgs::Float64 is_explore_start;
      is_explore_start.data = expl_manager_->fed_->start_explore_time_;
      start_explore_pub_.publish(is_explore_start);

      // Go to wait trigger when odom is ok
      transitState(WAIT_TRIGGER, "FSM");

      break;
    }

    case WAIT_TRIGGER:
    {
      double cur_time = ros::Time::now().toSec();
      if(cur_time - last_update_time > 0.19)
      {
        last_update_time = cur_time;
        surfaceFrontierExploration();
        pubBox();
      }
      // Do nothing but wait for trigger
      ROS_WARN_THROTTLE(1.0, "wait for trigger.");
      break;
    }

    case FINISH:
    {
      ROS_INFO_THROTTLE(1.0, "finish exploration.");
      break;
    }

    case PLAN_TRAJ: // 轨迹规划状态
    {
      ROS_WARN("PLAN_TRAJ");
      time_utils::Timer expl_planning_timer("expl_planning_timer");
      expl_planning_timer.Start();
      // ROS_WARN("explorationPlanner");
      Eigen::Vector3d next_pos;
      if(explorationPlanner(next_pos))
      {
        transitState(EXEC_TRAJ, "FSM");
      }
      else
      {
        // expl_manager_->frontier_finder_->coverSurfaceFrontier(expl_manager_->next_tour_point_frontiers_);
        transitState(PLAN_TRAJ, "FSM");
      }
        
      

      expl_manager_->fed_->local_plan_start_time_ = ros::WallTime::now().toSec();

      expl_planning_timer.Stop(false, "us");
      double iteration_time = static_cast<double>(expl_planning_timer.GetDuration("us")) / 1e6;
      double start_time = static_cast<double>(expl_planning_timer.getStartTime("us")) / 1e6;
      double end_time = static_cast<double>(expl_planning_timer.getStopTime("us")) / 1e6;
      sum_planning_time_ += iteration_time;
      planning_num_++;
      file_utils::writeToFileByAdd(planning_txt_name_, "\t", start_time, end_time,
                                   iteration_time, planning_num_, sum_planning_time_ / planning_num_);

      visualization_tools::IterationTime iter_time;
      iter_time.current_time = end_time;
      iter_time.iterationTime = iteration_time;
      iteration_time_pub_.publish(iter_time);

 
      
      
      break;
    }

    case WAIT_PLANNING:
    {
      ROS_INFO_THROTTLE(1.0, "wait motion planner.");
      transitState(WAIT_PLANNING, "FSM"); // 发布轨迹之后就是切换到执行轨迹状态
      break;
    }

    case EXEC_TRAJ:
    {
      double cur_time = ros::WallTime::now().toSec() - expl_manager_->fed_->local_plan_start_time_;
      double time_to_end = expl_manager_->fed_->bspline_end_time_ - ros::WallTime::now().toSec();
      // Replan if traj is almost fully executed
      if (time_to_end < fp_->replan_thresh1_ && expl_manager_->fed_->new_traj_)
      {
        expl_manager_->fed_->new_traj_ = false;
        ROS_WARN("Replan: traj fully executed=================================");
        transitState(EXPL_STATE::PLAN_TRAJ, "FSM");
        if(target_type_ == VIEWPOINT)
        {
          // expl_manager_->frontier_finder_->coverSurfaceFrontier();
          plan_state_ = PLAN_STATE::GLOBAL;
        }
        else
          plan_state_ = PLAN_STATE::LOCAL;
        return;
      }

      
      if(cur_time > fp_->replan_thresh2_)
      {
        if (expl_manager_->frontier_finder_->isSurfaceFrontierCovered2())
        {
          ROS_ERROR("Replan: goal scan call");   
          transitState(EXPL_STATE::PLAN_TRAJ, "FSM");
          plan_state_ = PLAN_STATE::GLOBAL;
          return;
        }

        // if (cur_time > fp_->replan_thresh3_) // 1.5s
        // {
        //   ROS_ERROR("Replan: local periodic call");
        //   transitState(EXPL_STATE::PLAN_TRAJ, "FSM");
        //   plan_state_ = PLAN_STATE::LOCAL;    
        //   return;
        // }

        // close to goal
        if ((fd_->odom_pos_ - expl_manager_->next_tour_point_).norm() < 1.0) 
        {
          ROS_ERROR("Replan: close to goal");
          expl_manager_->frontier_finder_->coverSurfaceFrontier(expl_manager_->next_tour_point_frontiers_);
          transitState(EXPL_STATE::PLAN_TRAJ, "FSM");
          plan_state_ = PLAN_STATE::GLOBAL;    
          return;
        }
        // if (isPeriodicGlobalReplan()) // 2.5s
        // {
        //   ROS_ERROR("FAEL_Replan: Global periodic call~~");
        //   transitState(EXPL_STATE::PLAN_TRAJ, "FSM");
        //   plan_state_ = PLAN_STATE::GLOBAL;   // 探索规划
        //   return;
        // }





        // if (expl_manager_->frontier_finder_->global_planning_update_ && duration > 1.0)
        // {
        //   ROS_ERROR("FAEL_Replan: global_planning_update~~");   
        //   transitState(EXPL_STATE::PLAN_TRAJ, "FSM");
        //   plan_state_ = PLAN_STATE::GLOBAL;
        //   return;
        // }


      }
      break;
    }
    }
  }

  int FastExplorationFSM::explorationPlanner(Eigen::Vector3d &next_pos)
  {
    int res;
    auto &ft = expl_manager_->frontier_finder_;
    auto &rm = expl_manager_->road_map_;
    auto &sdf = expl_manager_->frontier_finder_->sdf_map_;
    if(plan_state_== GLOBAL)
    {
      surfaceFrontierExploration();
      // ROS_WARN("getTourPoints");

      ft->getTourPoint(expl_manager_->next_tour_point_, expl_manager_->next_tour_point_frontiers_);

      // ROS_WARN("setTargetViewpoint");
      ft->setTargetViewpoint(expl_manager_->next_tour_point_, expl_manager_->next_tour_point_frontiers_);
      // ROS_WARN("Next tour point x:%0.2f  y:%0.2f  z:%0.2f", expl_manager_->next_tour_point_.x(), expl_manager_->next_tour_point_.y(), expl_manager_->next_tour_point_.z());
      // ROS_WARN("pubNextTourPoint");
      pubNextTourPoint();
      expl_manager_->fed_->global_plan_start_time_ = ros::Time::now().toSec();
    }
    ROS_WARN("find path to tour point");
    
    vector<Vector3d> path;
    ft->Astar_->reset();

    if (ft->Astar_->search(fd_->odom_pos_, expl_manager_->next_tour_point_) != fast_planner::Astar3::REACH_END)
    {
      ROS_ERROR("[explorationPlanner]:Find path from cur_pos to target point in grid map failed");
      target_type_ = VIEWPOINT;
      return 0;
    } 
    else
    {
      path = ft->Astar_->getPath();
      expl_manager_->shortenPath2(path);

      next_pos = *(path.begin()+1);
      double distance = 0;

      for(int i = 1; i < path.size(); i++)
      {
        distance+=(path[i]-path[i-1]).norm();
        
        next_pos = path[i];
        if(distance > 5.0)
          break;
      }
      pubNextPoint(next_pos);
      ROS_WARN("Publish next point  x:%0.2f  y:%0.2f  z:%0.2f", next_pos.x(), next_pos.y(), next_pos.z());
      target_type_ = VIEWPOINT;
      return 1;
    }
        
    
    

    // if (path.empty())   // 在local_gridmap上找不到，再到roadgraph上找
    // {   
    //   ROS_WARN("Target viewpoint not found in Map3D! Finding in RoadMap");

    //   int current_point_id = rm->addPointToGraph(fd_->odom_pos_);

    //   int viewpoint_id;
    //   if (rm->getPointId(expl_manager_->next_tour_point_, viewpoint_id))
    //   {
    //     rm->findShortestPath(current_point_id, viewpoint_id, path);
    //     ROS_WARN("Target viewpoint found in roadmap 1.");
    //   }
        
  
    //   if (path.empty())   // 在rodamap中搜
    //   {
    //     int b_id;
    //     int target_id;
    //     utils::Point3D nearest_point;
    //     if (rm->nearestSearch(expl_manager_->next_tour_point_, nearest_point, b_id))
    //     {
    //         int target_id = rm->addVertex(expl_manager_->next_tour_point_);
    //         rm->addTwoWayEdge(target_id, b_id);
    //         ROS_WARN("Added target point and its edges in plan graph.");
    //     }
    //     rm->findShortestPath(current_point_id, viewpoint_id, path);
    //     if (path.empty())
    //         ROS_ERROR("Target viewpoint not found in Roadmap!");
    //     else
    //       ROS_WARN("Target viewpoint found in roadmap 2.");

    //   }
    // }
    // // ROS_WARN("Start findLookAheadPoint");
    // if(!path.empty())
    // {
    //   expl_manager_->shortenPath2(path);

    //   next_pos = *(path.begin()+1);
    //   double distance = 0;

    //   for(int i = 1; i < path.size(); i++)
    //   {
    //     distance+=(path[i]-path[i-1]).norm();
        
    //     next_pos = path[i];
    //     if(distance > 8.0)
    //       break;
    //   }
    //   pubNextPoint(next_pos);
    //   ROS_WARN("Publish next point  x:%0.2f  y:%0.2f  z:%0.2f", next_pos.x(), next_pos.y(), next_pos.z());
    //   target_type_ = VIEWPOINT;
    //   // if((next_pos-expl_manager_->next_tour_point_).norm() < 0.5)
    //   //   target_type_ = VIEWPOINT;
    //   // else
    //   //   target_type_ = WAYPOINT;
    // }

      // pubNextPoint(expl_manager_->next_tour_point_);
      // ROS_WARN("Publish next point  x:%0.2f  y:%0.2f  z:%0.2f", expl_manager_->next_tour_point_.x(), expl_manager_->next_tour_point_.y(), expl_manager_->next_tour_point_.z());
      // // if((next_pos-expl_manager_->next_tour_point_).norm() < 0.5)
      //   target_type_ = VIEWPOINT;

    return 1;      
  }

  void FastExplorationFSM::FSMCallback(const ros::TimerEvent &e)
  {
    // 输出FSM当前状态
    ROS_INFO_STREAM_THROTTLE(1.0, "[FSM]: state: " << fd_->state_str_[int(state_)]);

    // 根据不同状态做不同的事情
    switch (state_)
    {
    case INIT:
    {
      // Wait for odometry ready
      if (!fd_->have_odom_)
      {
        ROS_WARN_THROTTLE(1.0, "no odom.");
        return;
      }

      if (!(start_explore_pub_.getNumSubscribers() > 0))
      {
        return;
      }

      // publish exploration status
      expl_manager_->fed_->start_explore_time_ = static_cast<double>(time_utils::Timer::GetTimeNow("us")) / 1e6;
      std_msgs::Float64 is_explore_start;
      is_explore_start.data = expl_manager_->fed_->start_explore_time_;
      start_explore_pub_.publish(is_explore_start);

      // Go to wait trigger when odom is ok
      transitState(WAIT_TRIGGER, "FSM");

      break;
    }

    case WAIT_TRIGGER:
    {
      // Do nothing but wait for trigger
      ROS_WARN_THROTTLE(1.0, "wait for trigger.");
      break;
    }

    case FINISH:
    {
      ROS_INFO_THROTTLE(1.0, "finish exploration.");
      break;
    }

    case PLAN_TRAJ: // 轨迹规划状态
    {
      // 如果探索结束，则发送结束的时间
      static bool explore_finish_flag = false;
      // static double last_time = 0;

      // double now_time = ros::WallTime::now().toSec();

      // ROS_WARN("plan period:%f", now_time-last_time);
      // last_time = now_time;

      if (expl_manager_->fed_->is_exploration_finished_ && !explore_finish_flag)
      {
        expl_manager_->fed_->finish_explore_time_ = static_cast<double>(time_utils::Timer::GetTimeNow("us")) / 1e6;

        std_msgs::Float64 finish;
        finish.data = expl_manager_->fed_->finish_explore_time_;
        finish_explore_pub_.publish(finish);

        ROS_WARN("** the exploration process has finished, total_time is %f s **",
                 (expl_manager_->fed_->finish_explore_time_ - expl_manager_->fed_->start_explore_time_));

        explore_finish_flag = true;
      }

      // 从起始静止状态开始规划
      if (fd_->static_state_)
      {
        // 指定一些初始的位姿
        // Plan from static state (hover)
        fd_->start_pt_ = fd_->odom_pos_;
        fd_->start_vel_ = fd_->odom_vel_;
        fd_->start_acc_.setZero();

        fd_->start_yaw_(0) = fd_->odom_yaw_;
        fd_->start_yaw_(1) = fd_->start_yaw_(2) = 0.0;
      }
      else
      {
        // Replan from non-static state, starting from 'replan_time' seconds later！！！！！
        LocalTrajData *info = &planner_manager_->local_data_;
        double t_r = (ros::Time::now() - info->start_time_).toSec() + fp_->replan_time_;

        // 从先前的B样条中获取初始位姿，下一次起点的状态是当前轨迹运行在当前时刻继续replan_time_之后的状态
        // 直接让飞机在replan_time_之后切换到下一条轨迹？
        fd_->start_pt_ = info->position_traj_.evaluateDeBoorT(t_r);
        fd_->start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_r);
        fd_->start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_r);
        fd_->start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_r)[0];
        fd_->start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_r)[0];
        fd_->start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_r)[0];
      }

      // Inform traj_server the replanning
      replan_pub_.publish(std_msgs::Empty()); // 不传递任何信息，仅仅起到一个触发作用

      time_utils::Timer expl_planning_timer("expl_planning_timer");
      expl_planning_timer.Start();

      int res = callExplorationPlanner(); // ***规划部分***

      expl_planning_timer.Stop(false, "us");
      double iteration_time = static_cast<double>(expl_planning_timer.GetDuration("us")) / 1e6;
      double start_time = static_cast<double>(expl_planning_timer.getStartTime("us")) / 1e6;
      double end_time = static_cast<double>(expl_planning_timer.getStopTime("us")) / 1e6;
      sum_planning_time_ += iteration_time;
      planning_num_++;
      file_utils::writeToFileByAdd(planning_txt_name_, "\t", start_time, end_time,
                                   iteration_time, planning_num_, sum_planning_time_ / planning_num_);

      visualization_tools::IterationTime iter_time;
      iter_time.current_time = end_time;
      iter_time.iterationTime = iteration_time;
      iteration_time_pub_.publish(iter_time);

      // ROS_ERROR("time for callExplorationPlanner: %fs", iteration_time);
      if (res == SUCCEED) // 规划成功跳到发布轨迹状态
      {
        transitState(PUB_TRAJ, "FSM");
      }
      else if (res == NO_FRONTIER) // 已经没有frontier了，规划完成
      {
        ROS_WARN("NO_FRONTIER");
        transitState(FINISH, "FSM");
        fd_->static_state_ = true;
        clearVisMarker();
      }
      else if (res == FAIL) // 规划失败，继续轨迹规划状态继续规划
      {
        // Still in PLAN_TRAJ state, keep replanning
        ROS_WARN("plan fail");
        fd_->static_state_ = true;
      }
      break;
    }

    case PUB_TRAJ:
    {
      double dt = (ros::Time::now() - fd_->newest_traj_.start_time).toSec();
      if (dt > 0)
      {
        expl_manager_->fed_->is_start_bspline_ = true;
        expl_manager_->fed_->bspline_start_time_ = fd_->newest_traj_.start_time;
        expl_manager_->fed_->bspline_duration_time_ = fd_->newest_traj_.duration_time;

        bspline_client_.waitForExistence();
        bspline::BsplineProcess bsp_process;
        bsp_process.request.bspline = fd_->newest_traj_;
        if (!bspline_client_.call(bsp_process))
        {
          return;
        }

        fd_->static_state_ = false;
        transitState(EXEC_TRAJ, "FSM"); // 发布轨迹之后就是切换到执行轨迹状态

        thread vis_thread(&FastExplorationFSM::visualize, this);
        vis_thread.detach(); // 使用分离子线程后台执行可视化
      }
      break;
    }

    case EXEC_TRAJ:
    {
      LocalTrajData *info = &planner_manager_->local_data_;
      double t_cur = (ros::Time::now() - info->start_time_).toSec();

      if (expl_manager_->ep_->use_fuel_planner_)
      {
        // Replan if traj is almost fully executed
        double time_to_end = info->duration_ - t_cur;
        if (time_to_end < fp_->replan_thresh1_)
        {
          transitState(PLAN_TRAJ, "FSM");
          ROS_WARN("Replan: traj fully executed=================================");
          return;
        }

        // Replan if next frontier to be visited is covered
        if (t_cur > fp_->replan_thresh2_ && expl_manager_->frontier_finder_->isSurfaceFrontierCovered())
        {
          transitState(PLAN_TRAJ, "FSM");
          ROS_WARN("Replan: cluster covered=====================================");
          return;
        }
        // Replan after some time
        if (t_cur > fp_->replan_thresh3_ && !classic_)
        {
          transitState(PLAN_TRAJ, "FSM");
          ROS_INFO("Replan: periodic call=======================================");
        }
      }
      else
      {
        LocalTrajData *info = &planner_manager_->local_data_;
        double t_cur = (ros::Time::now() - info->start_time_).toSec();

        // if((ros::Time::now().toSec() - expl_manager_->fed_->bspline_start_time_.toSec()) > 1)
        if(1)
        {
          // if (isWaitTooLong())  // traj is almost fully executed
          // {
          //   ROS_ERROR("FAEL_Replan: isWaitTooLong~~");
          //   transitState(EXPL_STATE::PLAN_TRAJ, "FSM");
          //   plan_state_ = PLAN_STATE::GLOBAL;
          //   return;
          // }

          if (isPeriodicGlobalReplan())
          {
            ROS_ERROR("FAEL_Replan: Global periodic call~~");
            transitState(EXPL_STATE::PLAN_TRAJ, "FSM");
            plan_state_ = PLAN_STATE::GLOBAL;   // 探索规划
            return;
          }

          if (expl_manager_->frontier_finder_->isSurfaceFrontierCovered())
          {
            ROS_ERROR("FAEL_Replan: goal scan call~~");   
            transitState(EXPL_STATE::PLAN_TRAJ, "FSM");
            plan_state_ = PLAN_STATE::GLOBAL;
            return;
          }

          if (isPeriodicLocalReplan())
          {
            ROS_ERROR("FAEL_Replan: Local periodic call~~");
            transitState(EXPL_STATE::PLAN_TRAJ, "FSM");
            plan_state_ = PLAN_STATE::LOCAL;    
            return;
          }
          // Replan if traj is almost fully executed
          double time_to_end = info->duration_ - t_cur;
          if (time_to_end < fp_->replan_thresh1_)
          {
            ROS_ERROR("FAEL_Replan: traj fully executed~~");
            transitState(EXPL_STATE::PLAN_TRAJ, "FSM");
            plan_state_ = PLAN_STATE::LOCAL;    
            return;
          }
        }
      }
      break;
    }
    }
  }

  int FastExplorationFSM::callExplorationPlanner()
  {
    ros::Time time_r = ros::Time::now() + ros::Duration(fp_->replan_time_);
    int res;

    Eigen::Vector3d next_pos;
    double next_yaw = 0.0;

    if (expl_manager_->ep_->use_fuel_planner_)
    {
      planner_manager_->lockSDFMap();
      res = expl_manager_->planExploreMotion(fd_->start_pt_, fd_->start_vel_, fd_->start_acc_,
                                             fd_->start_yaw_);
      planner_manager_->unLockSDFMap();
    }
    else
    {
      auto &ft = expl_manager_->frontier_finder_;
      auto &rm = expl_manager_->road_map_;
      auto &sdf = expl_manager_->frontier_finder_->sdf_map_;
      if(plan_state_== GLOBAL)
      {
        ft->getTourPoints(expl_manager_->tour_points_, expl_manager_->tour_points_frontiers_);

        if(!expl_manager_->tour_points_.empty())
        {
          expl_manager_->next_tour_point_= expl_manager_->tour_points_[0];
          expl_manager_->next_tour_point_frontiers_ = expl_manager_->tour_points_frontiers_[0];

          expl_manager_->tour_points_.erase(expl_manager_->tour_points_.begin());
          expl_manager_->next_tour_point_frontiers_.erase(expl_manager_->next_tour_point_frontiers_.begin());
        }

        ft->setTargetViewpoint(expl_manager_->next_tour_point_, expl_manager_->next_tour_point_frontiers_);
        // ROS_WARN("tour_points_ size:%d", expl_manager_->tour_points_.size());
        // ROS_WARN("Next tour point x:%0.2f  y:%0.2f  z:%0.2f", expl_manager_->next_tour_point_.x(), expl_manager_->next_tour_point_.y(), expl_manager_->next_tour_point_.z());
        pubNextTourPoint();
      }

      vector<Vector3d> path;
      ft->Astar_->reset();

      if (ft->Astar_->search(fd_->odom_pos_, expl_manager_->next_tour_point_) != fast_planner::Astar3::REACH_END)
          // ROS_ERROR("Find path from cur_pos to target point in grid map failed");
      
      path = ft->Astar_->getPath();

      if (path.empty())   // 在local_gridmap上找不到，再到roadgraph上找
      {   
        // ROS_WARN("Target viewpoint not found in Map3D! Finding in RoadMap");

        int current_point_id = rm->addPointToGraph(fd_->odom_pos_);

        int viewpoint_id;
        if (rm->getPointId(expl_manager_->next_tour_point_, viewpoint_id))
          rm->findShortestPath(current_point_id, viewpoint_id, path);
    
        if (path.empty())   // 在rodamap中搜
        {
          int b_id;
          int target_id;
          utils::Point3D nearest_point;
          if (rm->nearestSearch(expl_manager_->next_tour_point_, nearest_point, b_id))
          {
              int target_id = rm->addVertex(expl_manager_->next_tour_point_);
              rm->addTwoWayEdge(target_id, b_id);
              // ROS_WARN("Added target point and its edges in plan graph.");
          }
          rm->findShortestPath(current_point_id, viewpoint_id, path);
          // if (path.empty())
              // ROS_ERROR("Target viewpoint not found in Roadmap!");

        }
      }
      // ROS_WARN("Start findLookAheadPoint");
      if(!path.empty())
      {

        expl_manager_->shortenPath2(path);
        next_pos = path.back();
        for(int i = 0; i < (path.size()-1); i++)
        {
          if((path[i+1]-fd_->odom_pos_).norm() > 15.0)
          {
            next_pos = path[i];
            break;
          }
        }
      }      
    }

    planner_manager_->lockSDFMap();
    // 规划路径
    res = expl_manager_->planExploreMotionFromLookAheadPose(fd_->start_pt_, fd_->start_vel_, fd_->start_acc_,
                                                            fd_->start_yaw_, next_pos, next_yaw);
    planner_manager_->unLockSDFMap();
      

    // int res = expl_manager_->classicFrontier(fd_->start_pt_, fd_->start_yaw_[0]);
    // classic_ = true;

    // int res = expl_manager_->rapidFrontier(fd_->start_pt_, fd_->start_vel_, fd_->start_yaw_[0],
    // classic_);

    if (res == SUCCEED)
    {
      auto info = &planner_manager_->local_data_;
      info->start_time_ = (ros::Time::now() - time_r).toSec() > 0 ? ros::Time::now() : time_r; // 真正的开始时间

      bspline::Bspline bspline;
      bspline.order = planner_manager_->pp_.bspline_degree_;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;
      bspline.duration_time = info->duration_;
      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint(); // 提取位置B样条轨迹的控制点
      for (int i = 0; i < pos_pts.rows(); ++i)
      {
        geometry_msgs::Point pt;
        pt.x = pos_pts(i, 0);
        pt.y = pos_pts(i, 1);
        pt.z = pos_pts(i, 2);
        bspline.pos_pts.push_back(pt);
      }
      Eigen::VectorXd knots = info->position_traj_.getKnot(); // 提取位置B样条轨迹的节点
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }
      Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint(); // 提取yaw角B样条轨迹的控制点
      for (int i = 0; i < yaw_pts.rows(); ++i)
      {
        double yaw = yaw_pts(i, 0);
        bspline.yaw_pts.push_back(yaw);
      }
      bspline.yaw_dt = info->yaw_traj_.getKnotSpan(); // 提取yaw角的B样条轨迹的节点间隔

      fd_->newest_traj_ = bspline;
    }
    return res;
  }

  void FastExplorationFSM::pubNextTourPoint()
  {
    sensor_msgs::PointCloud2 msg;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    pcl::PointXYZ point;
    point.x = expl_manager_->next_tour_point_.x();
    point.y = expl_manager_->next_tour_point_.y();
    point.z = expl_manager_->next_tour_point_.z();
    cloud.points.push_back(point);
    // 将 PCL 点云数据转换为 ROS 点云消息
    pcl::toROSMsg(cloud, msg);

    // 设置消息头信息
    msg.header.frame_id = "world";  // frame_id 是 RViz 中显示点云时的参考坐标系
    msg.header.stamp = ros::Time::now();
    next_tour_point_pub_.publish(msg);
  }

  void FastExplorationFSM::pubNextPoint(Vector3d point)
  {
    sensor_msgs::PointCloud2 msg;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    pcl::PointXYZ pcl_point;
    pcl_point.x = point.x();
    pcl_point.y = point.y();
    pcl_point.z = point.z();
    cloud.points.push_back(pcl_point);
    // 将 PCL 点云数据转换为 ROS 点云消息
    pcl::toROSMsg(cloud, msg);

    // 设置消息头信息
    msg.header.frame_id = "world";  // frame_id 是 RViz 中显示点云时的参考坐标系
    msg.header.stamp = ros::Time::now();
    next_point_pub_.publish(msg);

    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "world";
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "world";
    pose.pose.position.x = point.x();
    pose.pose.position.y = point.y();
    pose.pose.position.z = point.z();

    pose.pose.orientation.x = pose.pose.orientation.y = pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    path.poses.push_back(pose);

    next_pos_pub_.publish(path);

    point_id_++;
  }

  void FastExplorationFSM::pubBox()
  {
    auto &sdf = expl_manager_->frontier_finder_->sdf_map_;
    Vector3d min, max;
    sdf->getBox(min, max);
    // 创建Marker消息
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world"; 
    marker.header.stamp = ros::Time::now();
    
    marker.type = visualization_msgs::Marker::LINE_LIST; 
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.scale.x = 1.0; 
    marker.color.r = 1.0;  
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0; 

    marker.pose.orientation.x = marker.pose.orientation.y = marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    geometry_msgs::Point p1, p2, p3, p4, p5, p6, p7, p8;
    
    p1.x = min.x(); p1.y = min.y(); p1.z = min.z();
    p2.x = max.x(); p2.y = min.y(); p2.z = min.z();
    p3.x = max.x(); p3.y = max.y(); p3.z = min.z();
    p4.x = min.x(); p4.y = max.y(); p4.z = min.z();
    
    p5.x = min.x(); p5.y = min.y(); p5.z = max.z();
    p6.x = max.x(); p6.y = min.y(); p6.z = max.z();
    p7.x = max.x(); p7.y = max.y(); p7.z = max.z();
    p8.x = min.x(); p8.y = max.y(); p8.z = max.z();

    marker.points.push_back(p1); marker.points.push_back(p2);
    marker.points.push_back(p2); marker.points.push_back(p3);
    marker.points.push_back(p3); marker.points.push_back(p4);
    marker.points.push_back(p4); marker.points.push_back(p1);

    marker.points.push_back(p5); marker.points.push_back(p6);
    marker.points.push_back(p6); marker.points.push_back(p7);
    marker.points.push_back(p7); marker.points.push_back(p8);
    marker.points.push_back(p8); marker.points.push_back(p5);

    marker.points.push_back(p1); marker.points.push_back(p5);
    marker.points.push_back(p2); marker.points.push_back(p6);
    marker.points.push_back(p3); marker.points.push_back(p7);
    marker.points.push_back(p4); marker.points.push_back(p8);

    box_pub_.publish(marker);
  }

  void FastExplorationFSM::visualize()
  {
    auto info = &planner_manager_->local_data_;
    auto plan_data = &planner_manager_->plan_data_;
    auto ed_ptr = expl_manager_->ed_;

    // Draw updated box
    // Vector3d bmin, bmax;
    // planner_manager_->edt_environment_->sdf_map_->getUpdatedBox(bmin, bmax);
    // visualization_->drawBox((bmin + bmax) / 2.0, bmax - bmin, Vector4d(0, 1, 0, 0.3), "updated_box", 0,
    // 4);

    // Draw frontier
    static int last_ftr_num = 0;
    for (int i = 0; i < ed_ptr->frontiers_.size(); ++i)
    {
      visualization_->drawCubes(ed_ptr->frontiers_[i], 0.1,
                                visualization_->getColor(double(i) / ed_ptr->frontiers_.size(), 0.8),
                                "frontier", i, 4);
      // visualization_->drawBox(ed_ptr->frontier_boxes_[i].first, ed_ptr->frontier_boxes_[i].second,
      //                         Vector4d(0.5, 0, 1, 0.3), "frontier_boxes", i, 4);
    }
    for (int i = ed_ptr->frontiers_.size(); i < last_ftr_num; ++i)
    {
      visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i, 4);
      // visualization_->drawBox(Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector4d(1, 0, 0, 0.3),
      // "frontier_boxes", i, 4);
    }
    last_ftr_num = ed_ptr->frontiers_.size();
    // for (int i = 0; i < ed_ptr->dead_frontiers_.size(); ++i)
    //   visualization_->drawCubes(ed_ptr->dead_frontiers_[i], 0.1, Vector4d(0, 0, 0, 0.5), "dead_frontier",
    //                             i, 4);
    // for (int i = ed_ptr->dead_frontiers_.size(); i < 5; ++i)
    //   visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 0.5), "dead_frontier", i, 4);

    // Draw global top viewpoints info
    visualization_->drawSpheres(ed_ptr->points_, 0.5, Vector4d(0.9, 0, 0, 1), "points", 0, 6);
    visualization_->drawLines(ed_ptr->global_tour_, 0.07, Vector4d(0, 0.5, 0, 1), "global_tour", 0, 6);
    visualization_->drawLines(ed_ptr->points_, ed_ptr->views_, 0.05, Vector4d(0, 1, 0.5, 1), "view", 0, 6);
    visualization_->drawLines(ed_ptr->points_, ed_ptr->averages_, 0.03, Vector4d(1, 0, 0, 1),
                              "point-average", 0, 6);

    // Draw local refined viewpoints info
    // visualization_->drawSpheres(ed_ptr->refined_points_, 0.2, Vector4d(0, 0, 1, 1), "refined_pts", 0, 6);
    // visualization_->drawLines(ed_ptr->refined_points_, ed_ptr->refined_views_, 0.05,
    //                           Vector4d(0.5, 0, 1, 1), "refined_view", 0, 6);
    // visualization_->drawLines(ed_ptr->refined_tour_, 0.07, Vector4d(0, 0, 1, 1), "refined_tour", 0, 6);
    // visualization_->drawLines(ed_ptr->refined_views1_, ed_ptr->refined_views2_, 0.04, Vector4d(0, 0, 0,
    // 1),
    //                           "refined_view", 0, 6);
    // visualization_->drawLines(ed_ptr->refined_points_, ed_ptr->unrefined_points_, 0.05, Vector4d(1, 1,
    // 0, 1),
    //                           "refine_pair", 0, 6);
    // for (int i = 0; i < ed_ptr->n_points_.size(); ++i)
    //   visualization_->drawSpheres(ed_ptr->n_points_[i], 0.1,
    //                               visualization_->getColor(double(ed_ptr->refined_ids_[i]) /
    //                               ed_ptr->frontiers_.size()),
    //                               "n_points", i, 6);
    // for (int i = ed_ptr->n_points_.size(); i < 15; ++i)
    //   visualization_->drawSpheres({}, 0.1, Vector4d(0, 0, 0, 1), "n_points", i, 6);

    // Draw trajectory
    // visualization_->drawSpheres({ ed_ptr->next_goal_ }, 0.3, Vector4d(0, 1, 1, 1), "next_goal", 0, 6);
    visualization_->drawBspline(info->position_traj_, 0.1, Vector4d(1.0, 0.0, 0.0, 1), false, 0.15,
                                Vector4d(1, 1, 0, 1));
    // visualization_->drawSpheres(plan_data->kino_path_, 0.1, Vector4d(1, 0, 1, 1), "kino_path", 0, 0);
    // visualization_->drawLines(ed_ptr->path_next_goal_, 0.05, Vector4d(0, 1, 1, 1), "next_goal", 1, 6);
  }

  void FastExplorationFSM::clearVisMarker()
  {
    // visualization_->drawSpheres({}, 0.2, Vector4d(0, 0.5, 0, 1), "points", 0, 6);
    // visualization_->drawLines({}, 0.07, Vector4d(0, 0.5, 0, 1), "global_tour", 0, 6);
    // visualization_->drawSpheres({}, 0.2, Vector4d(0, 0, 1, 1), "refined_pts", 0, 6);
    // visualization_->drawLines({}, {}, 0.05, Vector4d(0.5, 0, 1, 1), "refined_view", 0, 6);
    // visualization_->drawLines({}, 0.07, Vector4d(0, 0, 1, 1), "refined_tour", 0, 6);
    // visualization_->drawSpheres({}, 0.1, Vector4d(0, 0, 1, 1), "B-Spline", 0, 0);

    // visualization_->drawLines({}, {}, 0.03, Vector4d(1, 0, 0, 1), "current_pose", 0, 6);
  }

  void FastExplorationFSM::surfaceFrontierExploration()
  {
    static int delay = 0;
    if (++delay < 5) // 不执行5次，直接return
      return;

    // time_utils::Timer total_timer("total_time");
    total_timer.Start();
    // time_utils::Timer exploration_timer("exploration_time");
    exploration_timer.Start();

    auto ft = expl_manager_->frontier_finder_;
    auto ed = expl_manager_->ed_;
    auto rm = expl_manager_->road_map_;

    ft->setCurrentPose(fd_->odom_pos_);
    ft->updateForwardDirectory(fd_->odom_vel_);
    rm->setCurrentPosition(fd_->odom_pos_);

    // time_utils::Timer update_graph_timer("update_graph_time");
    update_graph_timer.Start();
    ROS_WARN("----------updateTopoGraphByMapAndViewpoints----------");
    rm->updateTopoGraphByMapAndViewpoints();
    update_graph_timer.Stop();

    // time_utils::Timer detect_obstacle_timer("detect_obstacle_time");
    detect_obstacle_timer.Start();
    ROS_WARN("----------updateObstacleClusters----------");
    ft->updateObstacleCloud();
    detect_obstacle_timer.Stop();
    
    // time_utils::Timer detect_global_frontier_timer("detect_global_frontier_time");
    detect_global_frontier_timer.Start();
    ROS_WARN("----------updateGlobalFrontier----------");
    ft->updateGlobalFrontier(); // too slow
    detect_global_frontier_timer.Stop();

    // time_utils::Timer cluster_frontier_timer("cluster_frontier_time");
    cluster_frontier_timer.Start();
    ROS_WARN("----------updateSurfaceFrontierCluster----------");
    ft->updateSurfaceFrontierCluster(); // too slow
    cluster_frontier_timer.Stop();

    // time_utils::Timer sample_viewpoint_timer("sample_viewpoint_time");
    sample_viewpoint_timer.Start();
    ROS_WARN("----------sampleSurfaceFrontierViewpoints2----------");
    ft->sampleSurfaceFrontierViewpoints2();
    sample_viewpoint_timer.Stop();

    // time_utils::Timer cluster_viewpoint_timer("cluster_viewpoint_time");
    cluster_viewpoint_timer.Start();
    ROS_WARN("----------updateViewpointCluster----------");
    ft->updateViewpointCluster();
    cluster_viewpoint_timer.Stop();

    // time_utils::Timer global_planning_timer("global_planning_time");
    global_planning_timer.Start();
    ROS_WARN("----------global_planning----------");
    ft->global_planning5();
    global_planning_timer.Stop();

    // time_utils::Timer viewpoint_refine_timer("viewpoint_refine_time");
    viewpoint_refine_timer.Start();
    ROS_WARN("----------viewpointRefine----------");
    ft->viewpointRefine();
    viewpoint_refine_timer.Stop();

    exploration_timer.Stop();

    // time_utils::Timer visualize_timer("visualize_time");
    visualize_timer.Start();   
    explorationVisualize();
    visualize_timer.Stop();

    processStatistics();
  }
  void FastExplorationFSM::processStatistics()
  {
    if(fd_->trigger_)
    {
      sd_->exp_time[0] = static_cast<double>(exploration_timer.GetDuration("us")) / 1e3;
      sd_->detect_obstacle_time[0] = static_cast<double>(detect_obstacle_timer.GetDuration("us")) / 1e3;
      sd_->detect_global_frontier_time[0] = static_cast<double>(detect_global_frontier_timer.GetDuration("us")) / 1e3;
      sd_->cluster_frontier_time[0] = static_cast<double>(cluster_frontier_timer.GetDuration("us")) / 1e3;
      sd_->sample_viewpoint_time[0] = static_cast<double>(sample_viewpoint_timer.GetDuration("us")) / 1e3;
      sd_->cluster_viewpoint_time[0] = static_cast<double>(cluster_viewpoint_timer.GetDuration("us")) / 1e3;
      sd_-> global_planning_time[0] = static_cast<double>(global_planning_timer.GetDuration("us")) / 1e3;
      sd_->viewpoint_refine_time[0] = static_cast<double>(viewpoint_refine_timer.GetDuration("us")) / 1e3;

      sd_->update_graph_time[0] = static_cast<double>(update_graph_timer.GetDuration("us")) / 1e3;
      sd_->visualize_time[0] = static_cast<double>(visualize_timer.GetDuration("us")) / 1e3;
      fd_->iter_num_++;
     
      sd_->exp_time[1] += sd_->exp_time[0];
      sd_->detect_obstacle_time[1] += sd_->detect_obstacle_time[0];
      sd_->detect_global_frontier_time[1] += sd_->detect_global_frontier_time[0];
      sd_->cluster_frontier_time[1] += sd_->cluster_frontier_time[0];
      sd_->sample_viewpoint_time[1] += sd_->sample_viewpoint_time[0];
      sd_->cluster_viewpoint_time[1] += sd_->cluster_viewpoint_time[0];
      sd_->global_planning_time[1] += sd_->global_planning_time[0];
      sd_->viewpoint_refine_time[1] += sd_->viewpoint_refine_time[0];
      sd_->update_graph_time[1] += sd_->update_graph_time[0];
      sd_->visualize_time[1] += sd_->visualize_time[0];

      sd_->exp_time[2] = sd_->exp_time[1]/fd_->iter_num_;
      sd_->detect_obstacle_time[2] = sd_->detect_obstacle_time[1]/fd_->iter_num_;
      sd_->detect_global_frontier_time[2] = sd_->detect_global_frontier_time[1]/fd_->iter_num_;
      sd_->cluster_frontier_time[2] = sd_->cluster_frontier_time[1]/fd_->iter_num_;
      sd_->sample_viewpoint_time[2] = sd_->sample_viewpoint_time[1]/fd_->iter_num_;
      sd_->cluster_viewpoint_time[2] = sd_->cluster_viewpoint_time[1]/fd_->iter_num_;
      sd_->global_planning_time[2] = sd_->global_planning_time[1]/fd_->iter_num_;
      sd_->viewpoint_refine_time[2] = sd_->viewpoint_refine_time[1]/fd_->iter_num_;
      sd_->update_graph_time[2] = sd_->update_graph_time[1]/fd_->iter_num_;
      sd_->visualize_time[2] = sd_->visualize_time[1]/fd_->iter_num_;

      sd_->exp_time[3] = max(sd_->exp_time[0], sd_->exp_time[3]);
      sd_->detect_obstacle_time[3] = max(sd_->detect_obstacle_time[0], sd_->detect_obstacle_time[3]);
      sd_->detect_global_frontier_time[3] = max(sd_->detect_global_frontier_time[0], sd_->detect_global_frontier_time[3]);
      sd_->cluster_frontier_time[3] = max(sd_->cluster_frontier_time[0], sd_->cluster_frontier_time[3]);
      sd_->sample_viewpoint_time[3] = max(sd_->sample_viewpoint_time[0], sd_->sample_viewpoint_time[3]);
      sd_->cluster_viewpoint_time[3] = max(sd_->cluster_viewpoint_time[0], sd_->cluster_viewpoint_time[3]);
      sd_->global_planning_time[3] = max(sd_->global_planning_time[0], sd_->global_planning_time[3]);
      sd_->viewpoint_refine_time[3] = max(sd_->viewpoint_refine_time[0], sd_->viewpoint_refine_time[3]);
      sd_->update_graph_time[3] = max(sd_->update_graph_time[0], sd_->update_graph_time[3]);
      sd_->visualize_time[3] = max(sd_->visualize_time[0], sd_->visualize_time[3]);

      ros::Time cur_time = ros::Time::now();
      file_utils::writeToFileByAdd(module_time_txt_, "\t", 
                                  fd_->iter_num_,
                                  (cur_time - fd_->start_time_).toSec(),
                                  sd_->exp_time[0],
                                  sd_->update_graph_time[0],
                                  sd_->detect_obstacle_time[0],
                                  sd_->detect_global_frontier_time[0],
                                  sd_->cluster_frontier_time[0],
                                  sd_->sample_viewpoint_time[0],
                                  sd_->cluster_viewpoint_time[0],
                                  sd_->global_planning_time[0],
                                  sd_->viewpoint_refine_time[0],
                                  sd_->visualize_time[0],
                                  cur_time); 
      file_utils::writeToFileByAdd(module_avg_max_time_txt_, "\t", 
                                  fd_->iter_num_,
                                  "avg",
                                  sd_->exp_time[2],
                                  sd_->update_graph_time[2],
                                  sd_->detect_obstacle_time[2],
                                  sd_->detect_global_frontier_time[2],
                                  sd_->cluster_frontier_time[2],
                                  sd_->sample_viewpoint_time[2],
                                  sd_->cluster_viewpoint_time[2],
                                  sd_->global_planning_time[2],
                                  sd_->viewpoint_refine_time[2],
                                  sd_->visualize_time[2],
                                  cur_time); 
      file_utils::writeToFileByAdd(module_avg_max_time_txt_, "\t",
                                  fd_->iter_num_,
                                  "max",
                                  sd_->exp_time[3],
                                  sd_->update_graph_time[3],
                                  sd_->detect_obstacle_time[3],
                                  sd_->detect_global_frontier_time[3],
                                  sd_->cluster_frontier_time[3],
                                  sd_->sample_viewpoint_time[3],
                                  sd_->cluster_viewpoint_time[3],
                                  sd_->global_planning_time[3],
                                  sd_->viewpoint_refine_time[3],
                                  sd_->visualize_time[3],
                                  cur_time); 

        std_msgs::Float64 msg;
        msg.data = sd_->exp_time[0], exp_time_pub_.publish(msg);  

        msg.data = sd_->update_graph_time[0], update_graph_time_pub_.publish(msg);  

        msg.data = sd_->detect_obstacle_time[0], detect_obstacle_time_pub_.publish(msg);  

        msg.data = sd_->detect_global_frontier_time[0], detect_global_frontier_time_pub_.publish(msg);  

        msg.data = sd_->cluster_frontier_time[0], cluster_frontier_time_pub_.publish(msg);  
        msg.data = sd_->sample_viewpoint_time[0], sample_viewpoint_time_pub_.publish(msg);  
        msg.data = sd_->cluster_viewpoint_time[0], cluster_viewpoint_time_pub_.publish(msg);  
        msg.data = sd_->global_planning_time[0], global_planning_time_pub_.publish(msg);  
        msg.data = sd_->viewpoint_refine_time[0], viewpoint_refine_time_pub_.publish(msg);  
        msg.data = sd_->visualize_time[0], visualize_time_pub_.publish(msg);  
    }
  }

  void FastExplorationFSM::explorationVisualize()
  {
    auto ft = expl_manager_->frontier_finder_;
    auto ed = expl_manager_->ed_;
    auto rm = expl_manager_->road_map_;

    time_utils::Timer visualize_timer("visualize_timer");
    visualize_timer.Start();

    // ROS_WARN("----------pubFrontiersMarkers----------");
    time_utils::Timer pubFrontiers_timer("pubFrontiers_timer");
    pubFrontiers_timer.Start();
    ft->pubFrontiersMarkers();
    pubFrontiers_timer.Stop();

    // ROS_WARN("----------pubObstacleClustersMarkers----------");
    time_utils::Timer pubObstacleClusters_timer("pubObstacleClusters_timer");
    pubObstacleClusters_timer.Start();
    ft->pubObstacleClustersMarkers();
    pubObstacleClusters_timer.Stop();

    // ROS_WARN("----------pubViewPoints----------");
    time_utils::Timer pubViewPoints_timer("pubViewPoints_timer");
    pubViewPoints_timer.Start();
    ft->pubViewPoints();
    pubViewPoints_timer.Stop();

    // ROS_WARN("----------pubSurfaceFrontierClustersMarkers----------");
    time_utils::Timer pubSurfaceFrontier_timer("pubSurfaceFrontier_timer");
    pubSurfaceFrontier_timer.Start();
    ft->pubSurfaceFrontierClustersMarkers();
    pubSurfaceFrontier_timer.Stop();

    // ROS_WARN("----------pubOccupancyCloud----------");
    time_utils::Timer pubOccupancyCloud_timer("pubOccupancyCloud_timer");
    pubOccupancyCloud_timer.Start();
    ft->pubOccupancyCloud();
    pubOccupancyCloud_timer.Stop();

    // ROS_WARN("----------pubSurfaceFrontierNormalsMarkers----------");
    time_utils::Timer pubSurfaceFrontierNormals_timer("pubSurfaceFrontierNormals_timer");
    pubSurfaceFrontierNormals_timer.Start();
    ft->pubSurfaceFrontierNormalsMarkers();
    pubSurfaceFrontierNormals_timer.Stop();

    // ROS_WARN("----------pubGlobalPathMarkers----------");
    time_utils::Timer pubGlobalPath_timer("pubGlobalPath_timer");
    pubGlobalPath_timer.Start();
    ft->pubGlobalPathMarkers();
    pubGlobalPath_timer.Stop();

    // ROS_WARN("----------pubViewpointWithFrontiers----------");
    time_utils::Timer pubViewpointWithFrontiers_timer("pubViewpointWithFrontiers_timer");
    pubViewpointWithFrontiers_timer.Start();
    ft->pubViewpointWithFrontiers();
    pubViewpointWithFrontiers_timer.Stop();

    // ROS_WARN("----------pubEndVels----------");
    time_utils::Timer pubEndVels_timer("pubEndVels_timer");
    pubEndVels_timer.Start();
    ft->pubEndVels();
    pubEndVels_timer.Stop();

    // ROS_WARN("----------pubCurrentDirection----------");
    time_utils::Timer pubCurrentDirection_timer("pubCurrentDirection_timer");
    pubCurrentDirection_timer.Start();
    ft->pubCurrentDirection();
    pubCurrentDirection_timer.Stop();


    // ROS_WARN("----------pubViewpointClusters2----------");
    time_utils::Timer pubViewpointCluster_timer("pubViewpointCluster_timer");
    pubViewpointCluster_timer.Start();
    ft->pubViewpointClusters2();
    pubViewpointCluster_timer.Stop();

    ft->pubRoadMapPath();

    // ROS_WARN("----------pubGraphMarkers----------");
    time_utils::Timer pubGraph_timer("pubGraph_timer");
    pubGraph_timer.Start();
    if(fp_->show_road_map_)
      rm->pubGraphMarkers();
    pubGraph_timer.Stop();

    visualize_timer.Stop();
    double visualize_time = static_cast<double>(visualize_timer.GetDuration("us")) / 1e3;
    double pubFrontiers_time = static_cast<double>(pubFrontiers_timer.GetDuration("us")) / 1e3;
    double pubObstacleClusters_time = static_cast<double>(pubObstacleClusters_timer.GetDuration("us")) / 1e3;
    double pubViewPoints_time = static_cast<double>(pubViewPoints_timer.GetDuration("us")) / 1e3;
    double pubSurfaceFrontier_time = static_cast<double>(pubSurfaceFrontier_timer.GetDuration("us")) / 1e3;
    double pubOccupancyCloud_time = static_cast<double>(pubOccupancyCloud_timer.GetDuration("us")) / 1e3;
    double pubSurfaceFrontierNormals_time = static_cast<double>(pubSurfaceFrontierNormals_timer.GetDuration("us")) / 1e3;
    double pubGlobalPath_time = static_cast<double>(pubGlobalPath_timer.GetDuration("us")) / 1e3;
    double pubViewpointWithFrontiers_time = static_cast<double>(pubViewpointWithFrontiers_timer.GetDuration("us")) / 1e3;
    double pubEndVels_time = static_cast<double>(pubEndVels_timer.GetDuration("us")) / 1e3;
    double pubCurrentDirection_time = static_cast<double>(pubCurrentDirection_timer.GetDuration("us")) / 1e3;
    double pubViewpointCluster_time = static_cast<double>(pubViewpointCluster_timer.GetDuration("us")) / 1e3;
    double pubGraph_time = static_cast<double>(pubGraph_timer.GetDuration("us")) / 1e3;
    file_utils::writeToFileByAdd(visualization_time_txt_name_, "\t", 
                                 visualize_time, 
                                 pubFrontiers_time,
                                 pubObstacleClusters_time,
                                 pubViewPoints_time,
                                 pubSurfaceFrontier_time,
                                 pubOccupancyCloud_time,
                                 pubSurfaceFrontierNormals_time,
                                 pubGlobalPath_time,
                                 pubViewpointWithFrontiers_time,
                                 pubEndVels_time,
                                 pubCurrentDirection_time,
                                 pubViewpointCluster_time,
                                 pubGraph_time);
  }

  void FastExplorationFSM::triggerCallback(const nav_msgs::PathConstPtr &msg)
  {
    if (state_ != WAIT_TRIGGER)
      return;
    fd_->trigger_ = true;
    expl_manager_->frontier_finder_->trigger_ = true;
    cout << "Triggered!" << endl;
    transitState(PLAN_TRAJ, "triggerCallback");
    plan_state_ = PLAN_STATE::GLOBAL;
    fd_->start_time_ = ros::Time::now();
    file_utils::writeToFileByAdd(start_time_txt_, "\t", fd_->start_time_);
  }

  void FastExplorationFSM::safetyCallback(const ros::TimerEvent &e)
  {
    if (state_ == EXPL_STATE::EXEC_TRAJ)
    {
      // Check safety and trigger replan if necessary
      double dist;
      bool safe = planner_manager_->checkTrajCollision(dist);
      if (!safe)
      {
        ROS_WARN("Replan: collision detected==================================");
        transitState(PLAN_TRAJ, "safetyCallback");
        plan_state_ = PLAN_STATE::GLOBAL;
      }
    }
  }

  void FastExplorationFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    static ros::Time last_time = ros::Time::now();

    expl_manager_->fed_->current_pose_ = msg->pose.pose;
    // 接收里程计位置信息
    fd_->odom_pos_(0) = msg->pose.pose.position.x;
    fd_->odom_pos_(1) = msg->pose.pose.position.y;
    fd_->odom_pos_(2) = msg->pose.pose.position.z;

    // 接收里程计线速度信息
    fd_->odom_vel_(0) = msg->twist.twist.linear.x;
    fd_->odom_vel_(1) = msg->twist.twist.linear.y;
    fd_->odom_vel_(2) = msg->twist.twist.linear.z;

    // 接收里程计姿态信息
    fd_->odom_orient_.w() = msg->pose.pose.orientation.w;
    fd_->odom_orient_.x() = msg->pose.pose.orientation.x;
    fd_->odom_orient_.y() = msg->pose.pose.orientation.y;
    fd_->odom_orient_.z() = msg->pose.pose.orientation.z;

    // 从四元数转换到旋转矩阵-->为了求出yaw角，这里使用了ZYX表示方法？是的Eigen是ZYX表示(RPY角)
    Eigen::Vector3d rot_x = fd_->odom_orient_.toRotationMatrix().block<3, 1>(0, 0);
    fd_->odom_yaw_ = atan2(rot_x(1), rot_x(0)); // ZYX表示法？

    fd_->have_odom_ = true;

    if(fd_->trigger_)
    {
      ros::Time cur_time = ros::Time::now();
      if((cur_time - last_time).toSec() > 0.05)
      {
        sd_->flight_length_ += (fd_->odom_pos_-fd_->last_odom_pos_).norm();
        double duration = (cur_time - fd_->start_time_).toSec();
        fd_->last_odom_pos_ = fd_->odom_pos_;
        last_time = cur_time;

        file_utils::writeToFileByAdd(motion_data_txt_name_, "\t", 
                                      fd_->iter_num_,
                                      sd_->flight_length_,
                                      duration,
                                      sd_->flight_length_/duration,
                                      fd_->odom_pos_(0),
                                      fd_->odom_pos_(1),
                                      fd_->odom_pos_(2),
                                      fd_->odom_vel_.norm(),
                                      fd_->odom_vel_(0),
                                      fd_->odom_vel_(1),
                                      fd_->odom_vel_(2),
                                      cur_time); 
        std_msgs::Float64 msg;
        msg.data = sd_->flight_length_;  
        flight_length_pub_.publish(msg);
      }
    }
    else
    {
      fd_->last_odom_pos_ = fd_->odom_pos_;
    }
  }

  void FastExplorationFSM::trajCallback(const nav_msgs::PathConstPtr &msg)
  {
    // ROS_WARN("Get traj waypoints");
    expl_manager_->frontier_finder_->cur_traj_ = *msg;
  }

  void FastExplorationFSM::endTimeCallback(const std_msgs::Float64ConstPtr &msg)
  {
    // ROS_WARN("Get end time");
    expl_manager_->fed_->bspline_end_time_ = msg->data;
    expl_manager_->fed_->new_traj_ = true;
  }

  void FastExplorationFSM::transitState(EXPL_STATE new_state, string pos_call)
  {
    int pre_s = int(state_);
    state_ = new_state;
    cout << "[" + pos_call + "]: from " + fd_->state_str_[pre_s] + " to " + fd_->state_str_[int(new_state)]
         << endl;
  }

  void FastExplorationFSM::explorationFinishCallback(const std_msgs::BoolConstPtr &finish)
  {
    exploration_finish_ = finish->data;
    if (!expl_manager_->fed_->is_exploration_finished_ && finish->data)
    {
      expl_manager_->fed_->is_exploration_finished_ = true;
      if (state_ == EXPL_STATE::PLAN_TRAJ ||
          state_ == EXPL_STATE::EXEC_TRAJ)
      {
        transitState(EXPL_STATE::PLAN_TRAJ, "explorationFinishCallback");
        plan_state_ = PLAN_STATE::GLOBAL;
      }
    }
  }

  void FastExplorationFSM::forcedBackToOriginCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    if ((state_ == EXPL_STATE::PLAN_TRAJ ||
         state_ == EXPL_STATE::EXEC_TRAJ) &&
        (fabs(msg->pose.position.x - expl_manager_->fep_->init_x_) < 1.0 &&
         fabs(msg->pose.position.y - expl_manager_->fep_->init_y_) < 1.0))
    {
      ROS_ERROR("-----------------forcedBackToOriginCallback!----------------");
      transitState(EXPL_STATE::PLAN_TRAJ, "forcedBackToOriginCallback");
      plan_state_ = PLAN_STATE::GLOBAL;
      expl_manager_->fed_->is_exploration_finished_ = true;
    }
  }

  bool FastExplorationFSM::isWaitTooLong()
  {
    if (expl_manager_->fed_->is_start_bspline_)
    {
      return expl_manager_->fed_->bspline_duration_time_ - (ros::Time::now() - expl_manager_->fed_->bspline_start_time_).toSec() <
             expl_manager_->fep_->replan_too_long_thre_;
    }
    else
    {
      return false;
    }
  }

  bool FastExplorationFSM::isPeriodicLocalReplan()
  {
    if ((ros::WallTime::now().toSec() - expl_manager_->fed_->local_plan_start_time_) > expl_manager_->fep_->local_replan_periodic_thre_)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  bool FastExplorationFSM::isPeriodicGlobalReplan()
  {
    if ((ros::WallTime::now().toSec() - expl_manager_->fed_->global_plan_start_time_) > expl_manager_->fep_->global_replan_periodic_thre_)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  void FastExplorationFSM::trajPubCallback(const ros::TimerEvent &e)
  {
    static int id = 1; 
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.type = visualization_msgs::Marker::SPHERE_LIST;
    mk.action = visualization_msgs::Marker::ADD;
    mk.id = id;
    id++;
    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;

    mk.color.r = 1;
    mk.color.g = 0;
    mk.color.b = 0;
    mk.color.a = 1;

    mk.scale.x = 0.5;
    mk.scale.y = 0.5;
    mk.scale.z = 0.5;

    geometry_msgs::Point pt;
    pt.x = fd_->odom_pos_(0);
    pt.y = fd_->odom_pos_(1);
    pt.z = fd_->odom_pos_(2);
    mk.points.push_back(pt);
    
    // ROS_WARN("publish");
    traj_pub_.publish(mk);
  }
} // namespace fast_planner
