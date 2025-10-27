#ifndef TOPO_PLANNER_WS_EXPLORER_H
#define TOPO_PLANNER_WS_EXPLORER_H

#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <string>

#include <control_planner_interface/PlannerMsgs.h>
#include <control_planner_interface/control_planner_interface.h>
#include <control_planner_interface/pci_vehicle.h>
#include "bspline/Bspline.h"
#include <fael_exploration/Trigger.h>

namespace explorer
{

    class Explorer_FSM
    {
    public:
        enum struct FAEL_EXPL_STATE
        {
            INIT,
            WAIT_TRIGGER,
            PLAN_TRAJ,
            EXEC_TRAJ,
            FINISH
        };

        enum struct RunModeType
        {
            kSim = 0, // Run in simulation.
            kReal = 1 // Run with real robot.
        };

        enum struct ExecutionPathType
        {
            kLocalPath = 0,
            kHomingPath = 1,
            kGlobalPath = 2,
            kNarrowEnvPath = 3, // Narrow env.
            kManualPath = 4     // Manually set path.
        };

        struct FAELExplorationParams
        {
            bool init_motion_enable_;
            double init_x_;
            double init_y_;
            double init_z_;
            double max_vel_;
            double replan_too_long_thre_;
            double replan_periodic_thre_;
        };

        struct FAELExplorationData
        {
            FAEL_EXPL_STATE state_;
            ros::V_string state_string_vec_;

            int iteration_num_;
            double start_explore_time_;
            bool iteration_goal_is_scaned_;
            bool need_to_next_iteration_;
            bool exploration_finished_;
            bool need_to_stop_move_;

            double follow_start_time_;
            bool is_start_bspline;
            ros::Time bspline_start_time_;
            double bspline_duration_time_;

            geometry_msgs::Pose current_pose_;
        };

        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        ros::Publisher start_explore_pub_;
        ros::Publisher finish_explore_pub_;

        ros::Subscriber planner_msgs_sub_;
        ros::Subscriber explore_finish_sub_;
        ros::Subscriber trigger_sub_;
        ros::Subscriber stop_move_sub_;
        ros::Subscriber bspline_sub_;
        ros::Subscriber forced_back_to_origin_sub_;
        ros::Subscriber goal_scan_sub_;

        ros::ServiceServer trigger_server_;
        ros::Timer fael_explore_timer_;

        std::shared_ptr<interface::ControlPlannerInterface> interface_;
        std::shared_ptr<FAELExplorationParams> fael_expl_params;
        std::shared_ptr<FAELExplorationData> fael_expl_data;

        Explorer_FSM(ros::NodeHandle &nh_private, std::shared_ptr<interface::ControlPlannerInterface> &interface);

        bool loadParams();

        bool init();

        bool initMotion();

        void planTargetPoint(geometry_msgs::Pose &target_pose, bool &is_success, bool &is_finish);

        bool callForPlanner(const int &iteration_num,
                            std::vector<control_planner_interface::Path> &path_segments,
                            bool &is_exploration_finish);

        void followThePath(const std::vector<control_planner_interface::Path> &path_segments);

        void followThePath(const std::vector<control_planner_interface::Path> &path_segments,
                           bool &success, double &start_time, double &path_length, geometry_msgs::Pose &goal_pose);

        bool isWaitTooLong();

        bool isPeriodicReplan();

        void explorationFinishCallback(const std_msgs::BoolConstPtr &finish);

        void stopMoveCallback(const std_msgs::BoolConstPtr &stop_move_msg);

        void FSMCallback(const ros::TimerEvent &e);

        void transitState(FAEL_EXPL_STATE new_state, std::string pos_call);

        void triggerCallback(const nav_msgs::PathConstPtr &msg);

        bool triggerCallback(fael_exploration::Trigger::Request &req, fael_exploration::Trigger::Response &resp);

        void forcedBackToOriginCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

        void goalScanCallback(const std_msgs::BoolConstPtr &goal_scan_msg);

        void odomCallback(const nav_msgs::OdometryConstPtr &odom_msg);

        void bsplineCallback(const bspline::BsplineConstPtr &msg);
    };
}

#endif // TOPO_PLANNER_WS_EXPLORER_H
