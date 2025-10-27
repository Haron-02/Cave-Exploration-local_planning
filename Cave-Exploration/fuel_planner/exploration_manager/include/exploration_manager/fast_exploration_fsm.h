#ifndef _FAST_EXPLORATION_FSM_H_
#define _FAST_EXPLORATION_FSM_H_

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <bspline/BsplineProcess.h>
#include "exploration_manager/fast_exploration_manager.h"
#include <sensor_msgs/PointCloud2.h>

#include <algorithm>
#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <thread>

#include <fael_exploration/Trigger.h>
#include "time_utils/time_utils.h"

using Eigen::Vector3d;
using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::vector;

namespace fast_planner
{
  class FastPlannerManager;
  class FastExplorationManager;
  class PlanningVisualization;
  struct FSMParam;
  struct FSMData;
  struct StatisticalData;

  enum EXPL_STATE
  {
    INIT,
    WAIT_TRIGGER,
    PLAN_TRAJ,
    PUB_TRAJ,
    EXEC_TRAJ,
    FINISH,
    PUB_TARGET,
    WAIT_PLANNING
  };

  class FastExplorationFSM
  {
  private:
    /* planning utils */
    shared_ptr<FastPlannerManager> planner_manager_;  // 轨迹规划
    shared_ptr<FastExplorationManager> expl_manager_; // 探索规划
    shared_ptr<PlanningVisualization> visualization_;

    shared_ptr<FSMParam> fp_;
    shared_ptr<FSMData> fd_;
    shared_ptr<StatisticalData> sd_;
    EXPL_STATE state_;
    PLAN_STATE plan_state_;
    TARGET_TYPE target_type_;

    bool classic_;
    bool exploration_finish_;
    bool is_get_look_ahead_goal_;
    Eigen::Vector3d look_ahead_goal_;
    double look_ahead_yaw_;
    

    std::string planning_txt_name_, exp_module_txt_name_, motion_data_txt_name_, visualization_time_txt_name_;
    std::string module_time_txt_, module_avg_max_time_txt_, start_time_txt_;
    int planning_num_;
    double sum_planning_time_;
    int point_id_ = 0;

    // 
    int round_;
    time_utils::Timer total_timer;
    time_utils::Timer exploration_timer;
    time_utils::Timer update_graph_timer;
    time_utils::Timer detect_obstacle_timer;
    time_utils::Timer detect_global_frontier_timer;
    time_utils::Timer cluster_frontier_timer;
    time_utils::Timer sample_viewpoint_timer;
    time_utils::Timer cluster_viewpoint_timer;
    time_utils::Timer global_planning_timer;
    time_utils::Timer viewpoint_refine_timer;
    time_utils::Timer visualize_timer;

    ros::Publisher exp_time_pub_, detect_obstacle_time_pub_, detect_global_frontier_time_pub_;
    ros::Publisher cluster_frontier_time_pub_, sample_viewpoint_time_pub_, cluster_viewpoint_time_pub_;
    ros::Publisher global_planning_time_pub_, viewpoint_refine_time_pub_, visualize_time_pub_;
    ros::Publisher update_graph_time_pub_, flight_length_pub_;
    double trigger_time_, finish_time_;

    /* ROS utils */
    ros::NodeHandle node_;
    ros::Timer exec_timer_, safety_timer_, vis_timer_, frontier_timer_, traj_pub_timer_;
    ros::Timer surface_frontier_timer_;
    ros::Subscriber trigger_sub_, odom_sub_, traj_sub_, end_time_sub_, goal_scan_sub_, forced_back_to_origin_sub_;
    ros::Publisher replan_pub_, new_pub_, bspline_pub_, start_explore_pub_, finish_explore_pub_, iteration_time_pub_, traj_pub_;
    ros::Publisher next_pos_pub_, box_pub_;
    ros::Subscriber explore_finish_sub_, stop_move_sub_, look_ahead_goal_sub_;
    ros::ServiceClient topo_planner_trigger_client_;
    ros::ServiceClient bspline_client_;

    // DEBUG
    ros::Publisher next_tour_point_pub_, next_point_pub_;
    // DEBUG

    /* helper functions */
    int callExplorationPlanner();
    void surfaceFrontierExploration();
    void transitState(EXPL_STATE new_state, string pos_call);
    
    /* Init funtions*/
    void initStatistic(ros::NodeHandle &nh);

    /* ROS functions */
    void trajPubCallback(const ros::TimerEvent &e);

    void explorationFSMCallback(const ros::TimerEvent &e);
    int explorationPlanner(Eigen::Vector3d &next_pos);

    void FSMCallback(const ros::TimerEvent &e);
    void safetyCallback(const ros::TimerEvent &e);
    void frontierCallback(const ros::TimerEvent &e);

    void triggerCallback(const nav_msgs::PathConstPtr &msg);
    void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
    void trajCallback(const nav_msgs::PathConstPtr &msg);

    void endTimeCallback(const std_msgs::Float64ConstPtr &msg);

    void explorationFinishCallback(const std_msgs::BoolConstPtr &finish);
    void stopMoveCallback(const std_msgs::BoolConstPtr &stop_move_msg);
    void lookAheadGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &look_ahead_goal_msg);
    void forcedBackToOriginCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void goalScanCallback(const std_msgs::BoolConstPtr &goal_scan_msg);

    // int getNextPosAndYaw(const bool &use_init_motion, const PLAN_STATE &plan_state, const Eigen::Vector3d &yaw,
    //                      Eigen::Vector3d &next_pos, double &next_yaw);
    // int planTargetPointAndYaw(const PLAN_STATE &plan_state, Eigen::Vector3d &target_pos, double &target_yaw);
    bool isWaitTooLong();
    bool isPeriodicLocalReplan();
    bool isPeriodicGlobalReplan();
    void visualize();
    void explorationVisualize();
    void clearVisMarker();


    /* Statistics*/
    void processStatistics();

    void pubNextTourPoint();
    void pubNextPoint(Vector3d point);
    void pubBox();

  public:
    FastExplorationFSM(/* args */)
    {
    }
    ~FastExplorationFSM()
    {
    }

    void init(ros::NodeHandle &nh);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace fast_planner

#endif