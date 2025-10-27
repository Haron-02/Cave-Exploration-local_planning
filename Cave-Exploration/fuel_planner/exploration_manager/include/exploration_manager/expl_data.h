#ifndef _EXPL_DATA_H_
#define _EXPL_DATA_H_

#include <Eigen/Eigen>
#include <vector>
#include <bspline/Bspline.h>

using Eigen::Vector3d;
using std::vector;

namespace fast_planner
{
  struct FSMData
  {
    // FSM data
    bool trigger_, have_odom_, static_state_, pub_box_;
    vector<string> state_str_;

    Eigen::Vector3d odom_pos_, odom_vel_; // odometry state
    Eigen::Quaterniond odom_orient_;
    double odom_yaw_;

    Eigen::Vector3d start_pt_, start_vel_, start_acc_, start_yaw_; // start state
    vector<Eigen::Vector3d> start_poss;
    bspline::Bspline newest_traj_;

    // 数据统计
    ros::Time start_time_;
    double flight_length_;
    int iter_num_;

    Eigen::Vector3d last_odom_pos_;
    double total_exploration_time_;
  };

  struct FSMParam
  {
    double replan_thresh1_;
    double replan_thresh2_;
    double replan_thresh3_;
    double replan_time_;   // second
    bool use_init_motion_; // second
    bool show_road_map_;
  };

  struct StatisticalData
  {
    double exp_time[4] = {0}; // cur sum avg max 
    double detect_obstacle_time[4] = {0};
    double detect_global_frontier_time[4] = {0};
    double cluster_frontier_time[4] = {0};
    double sample_viewpoint_time[4] = {0};
    double cluster_viewpoint_time[4] = {0};
    double global_planning_time[4] = {0};
    double viewpoint_refine_time[4] = {0};

    double update_graph_time[4] = {0};
    double visualize_time[4] = {0};

    double flight_length_ = 0;
  };

  struct ExplorationData
  {
    vector<vector<Vector3d>> frontiers_;
    vector<vector<Vector3d>> dead_frontiers_;
    vector<pair<Vector3d, Vector3d>> frontier_boxes_;
    vector<Vector3d> points_;
    vector<Vector3d> averages_;
    vector<Vector3d> views_;
    vector<double> yaws_;
    vector<Vector3d> global_tour_;

    vector<int> refined_ids_;
    vector<vector<Vector3d>> n_points_;
    vector<Vector3d> unrefined_points_;
    vector<Vector3d> refined_points_;
    vector<Vector3d> refined_views_; // points + dir(yaw)
    vector<Vector3d> refined_views1_, refined_views2_;
    vector<Vector3d> refined_tour_;

    Vector3d next_goal_;
    Vector3d next_goal_FAEL_;
    vector<Vector3d> path_next_goal_;
    vector<Vector3d> path_next_goal_FAEL_;

    // viewpoint planning
    // vector<Vector4d> views_;
    vector<Vector3d> views_vis1_, views_vis2_;
    vector<Vector3d> centers_, scales_;
  };

  struct ExplorationParam
  {
    // params
    bool use_fuel_planner_;
    bool refine_local_;
    int refined_num_;
    double refined_radius_;
    int top_view_num_;
    double max_decay_;
    string tsp_dir_; // resource dir of tsp solver
    double relax_time_;
    double init_x;
    double init_y;
    double init_z;
  };

  struct FAELExplorationParams
  {
    bool init_motion_enable_;
    double init_x_;
    double init_y_;
    double init_z_;
    double max_vel_;
    double replan_too_long_thre_;
    double local_replan_periodic_thre_;
    double global_replan_periodic_thre_;
  };

  struct FAELExplorationData
  {
    int iteration_num_;
    double start_explore_time_;
    double finish_explore_time_;
    bool is_iteration_goal_scaned_;
    bool is_exploration_finished_;
    bool is_finished_;

    double local_plan_start_time_;
    double global_plan_start_time_;
    bool is_start_bspline_;
    ros::Time bspline_start_time_;
    double bspline_duration_time_;
    double bspline_end_time_;
    bool new_traj_;
    geometry_msgs::Pose current_pose_;
  };

} // namespace fast_planner

#endif