#ifndef _EXPLORATION_MANAGER_H_
#define _EXPLORATION_MANAGER_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <memory>
#include <vector>

#include <geometry_msgs/Pose.h>
// #include <control_planner_interface/Path.h>

using Eigen::Vector3d;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;
namespace preprocess
{
  class IkdTreeTopoGraph;
}

namespace fast_planner
{
  class EDTEnvironment;
  class SDFMap;
  class FastPlannerManager;
  class FrontierFinder;
  struct ExplorationParam;
  struct ExplorationData;
  struct FAELExplorationData;
  struct FAELExplorationParams;

  enum EXPL_RESULT
  {
    NO_FRONTIER,
    FAIL,
    SUCCEED
  };

  enum PLAN_STATE
  {
    IDLE,
    GLOBAL,
    LOCAL
  };

  enum TARGET_TYPE
  {
    VIEWPOINT,
    WAYPOINT
  };

  class FastExplorationManager
  {
  public:
    FastExplorationManager();
    ~FastExplorationManager();

    void initialize(ros::NodeHandle &nh);

    void loadParams(ros::NodeHandle &nh);

    int planExploreMotion(const Vector3d &pos, const Vector3d &vel, const Vector3d &acc,
                          const Vector3d &yaw);

    // void planLookAheadPose(const PLAN_STATE &plan_state, geometry_msgs::Pose &target_pose,
    //                        bool &is_success, bool &is_finished);

    int planExploreMotionFromLookAheadPose(const Vector3d &pos, const Vector3d &vel, const Vector3d &acc,
                                           const Vector3d &yaw, const Vector3d &look_ahead_goal_, const double &look_ahead_yaw_);

    void shortenPath2(vector<Vector3d> &path);

    // Benchmark method, classic frontier and rapid frontier
    int classicFrontier(const Vector3d &pos, const double &yaw);
    int rapidFrontier(const Vector3d &pos, const Vector3d &vel, const double &yaw, bool &classic);

    shared_ptr<ExplorationData> ed_;
    shared_ptr<ExplorationParam> ep_;
    shared_ptr<FAELExplorationData> fed_;
    shared_ptr<FAELExplorationParams> fep_;
    shared_ptr<FastPlannerManager> planner_manager_;
    shared_ptr<FrontierFinder> frontier_finder_;
    shared_ptr<preprocess::IkdTreeTopoGraph> road_map_;
    // unique_ptr<ViewFinder> view_finder_;
    ros::Publisher next_pose_pub_;

    vector<Vector3d> tour_points_;
    vector<vector<int>> tour_points_frontiers_;

    Vector3d next_tour_point_;
    vector<int> next_tour_point_frontiers_;

  private:
    shared_ptr<EDTEnvironment> edt_environment_;
    shared_ptr<SDFMap> sdf_map_;
    
    // road_map_

    // std::vector<control_planner_interface::Path> global_path_segments_;

    bool is_next_pos_need_modify_;
    int next_pos_need_modify_cnt_;

    // Find optimal tour for coarse viewpoints of all frontiers
    void findGlobalTour(const Vector3d &cur_pos, const Vector3d &cur_vel, const Vector3d cur_yaw,
                        vector<int> &indices);

    // Refine local tour for next few frontiers, using more diverse viewpoints
    void refineLocalTour(const Vector3d &cur_pos, const Vector3d &cur_vel, const Vector3d &cur_yaw,
                         const vector<vector<Vector3d>> &n_points, const vector<vector<double>> &n_yaws,
                         vector<Vector3d> &refined_pts, vector<double> &refined_yaws);

    void shortenPath(vector<Vector3d> &path);

  public:
    typedef shared_ptr<FastExplorationManager> Ptr;
  };

} // namespace fast_planner

#endif