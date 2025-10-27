// 无人机探索任务的有限状态机（FSM）实现（Explorer_FSM类）
// 核心作用是通过 “状态切换” 管理无人机的整个探索流程 —— 从初始化、等待触发，到规划轨迹、执行轨迹，再到探索完成，相当于无人机探索任务的 “大脑决策中枢”
#include "explorer/explorer_fsm.h"
#include <iostream>
#include <limits>


namespace explorer
{

    Explorer_FSM::Explorer_FSM(
        ros::NodeHandle &nh_private,
        std::shared_ptr<interface::ControlPlannerInterface> &interface) : nh_private_(nh_private),
                                                                          interface_(interface) // 节点句柄+控制-规划接口（手脚）
    {
        // 1. 初始化参数对象（存配置，如初始位置、速度阈值）和数据对象（存运行状态，如当前状态、迭代次数）
        fael_expl_params.reset(new FAELExplorationParams);
        fael_expl_data.reset(new FAELExplorationData);

        // 2. 创建ROS发布者：发“探索开始/结束时间”（给其他模块看进度）
        trigger_server_ = nh_private_.advertiseService("/topo_planner/trigger", &Explorer_FSM::triggerCallback, this);// 服务触发器

        start_explore_pub_ = nh_private_.advertise<std_msgs::Float64>("/exploration_data/explorer_start", 1);   // 开始时间

        finish_explore_pub_ = nh_private_.advertise<std_msgs::Float64>("/exploration_data/explorer_finish", 1); // 结束时间

        // 3. 创建ROS订阅者：收外部指令（停止、回原点、目标扫描完成等）
        explore_finish_sub_ = nh_private_.subscribe<std_msgs::Bool>("/exploration_data/exploration_data_finish", 1,
                                                                    &Explorer_FSM::explorationFinishCallback,
                                                                    this);  // 探索完成订阅

        stop_move_sub_ = nh_private_.subscribe<std_msgs::Bool>("/topo_planner/stop_move", 1,
                                                               &Explorer_FSM::stopMoveCallback,
                                                               this);   // 停止移动订阅

        bspline_sub_ = nh_private_.subscribe("/planning/bspline", 10, &Explorer_FSM::bsplineCallback, this);// B样条轨迹订阅，用于判断超时

        forced_back_to_origin_sub_ = nh_private_.subscribe("/move_base_simple/goal", 1, &Explorer_FSM::forcedBackToOriginCallback, this);// 强制回原点

        goal_scan_sub_ = nh_private_.subscribe("/topo_planner/goal_scan", 1, &Explorer_FSM::goalScanCallback, this);// 目标扫描完成

        // 4. 状态机“心跳”：每0.05秒触发一次FSMCallback（驱动状态切换）
        fael_explore_timer_ = nh_private_.createTimer(ros::Duration(0.05), &Explorer_FSM::FSMCallback, this);

        // 5. 加载配置参数+初始化数据对象（默认状态）
        loadParams();
        fael_expl_data->state_ = FAEL_EXPL_STATE::INIT; // 初始状态：INIT（初始化）
        fael_expl_data->state_string_vec_ = {"INIT", "WAIT_TRIGGER", "PLAN_TRAJ", "EXPL_FINISH", "FINISH"};
        fael_expl_data->iteration_num_ = 0; // 探索迭代次数（第1次、第2次...）
        fael_expl_data->is_start_bspline = false;   // 是否开始B样条轨迹
        fael_expl_data->exploration_finished_ = false; // 探索是否完成（默认未完成）
        fael_expl_data->iteration_goal_is_scaned_ = false;// 目标是否扫描完成
        fael_expl_data->need_to_next_iteration_ = true;// 是否需要下一轮规划（默认需要）
        fael_expl_data->need_to_stop_move_ = false;// 是否需要停止（默认不停止）
        fael_expl_data->follow_start_time_ = std::numeric_limits<double>::max();// 执行路径的开始时间（默认极大值，代表未开始）
    }

    //有限状态机的核心回调函数，根据当前状态执行相应操作并决定是否切换状态
    void Explorer_FSM::FSMCallback(const ros::TimerEvent &e)
    {
        switch (fael_expl_data->state_)
        {
        case FAEL_EXPL_STATE::INIT:
        {
            // Wait for the system is ready.
            if (!init())
            {
                ROS_WARN_THROTTLE(1.0, "init unsuccessful");
                return;
            }
            ROS_WARN("init motion has finished...");

            // Go to wait trigger when is ok
            transitState(FAEL_EXPL_STATE::WAIT_TRIGGER, "FSM");

            break;
        }

        // 2. 等待触发：等外部指令（如手动触发、其他模块触发）
        case FAEL_EXPL_STATE::WAIT_TRIGGER:
        {
            ROS_WARN_THROTTLE(1.0, "FAEL_Planner is waiting for trigger."); 
            break;
        }

        // 3. 规划轨迹：让规划器生成下一段探索路径
        case FAEL_EXPL_STATE::PLAN_TRAJ:
        {
            geometry_msgs::Pose target_pose;    // 目标位姿
            bool is_success, is_finish;
            planTargetPoint(target_pose, is_success, is_finish);    // 核心规划函数
            if (is_success) // 规划成功→切换到“执行轨迹”
            {
                transitState(FAEL_EXPL_STATE::EXEC_TRAJ, "FSM");
            }
            else
            {
                if (is_finish)  // 规划失败且探索完成→切换到“完成”
                {
                    transitState(FAEL_EXPL_STATE::FINISH, "FSM");
                }
            }
            break;
        }

        // 4. 执行轨迹：飞规划好的路径，同时检查异常
        case FAEL_EXPL_STATE::EXEC_TRAJ:
        {
            // 异常1：等待太久（比如路径卡住）→重规划
            if (isWaitTooLong())
            {
                ROS_ERROR("FAEL_Replan: isWaitTooLong~~");
                fael_expl_data->need_to_next_iteration_ = true;
                transitState(FAEL_EXPL_STATE::PLAN_TRAJ, "FSM");
                return;
            }

            // 异常2：到了周期性重规划时间（比如每2秒重规划一次，保证路径新鲜）
            if (isPeriodicReplan())
            {
                ROS_ERROR("FAEL_Replan: periodic call~~");
                fael_expl_data->need_to_next_iteration_ = true;
                transitState(FAEL_EXPL_STATE::PLAN_TRAJ, "FSM");
                return;
            }

            break;
        }

        // 5. 完成状态：探索结束，只打印提示
        case FAEL_EXPL_STATE::FINISH:
        {
            ROS_INFO_THROTTLE(1.0, "FAEL finish."); // 1秒内只打印一次“探索完成”
            break;
        }

        default:
            break;
        }
    }

    //规划目标点：调用规划器生成路径，并处理结果    
    void Explorer_FSM::planTargetPoint(geometry_msgs::Pose &target_pose, bool &is_success, bool &is_finish)
    {
        static bool finish_flag = false;    // 静态变量：确保“探索完成”只处理一次

        // 第一步：如果探索已完成→处理结束逻辑
        if (fael_expl_data->exploration_finished_)
        {
            if (!finish_flag)
            {
                // 计算探索总时间（当前时间 - 开始时间）
                double finish_explore_time = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                                                                     std::chrono::high_resolution_clock::now().time_since_epoch())
                                                                     .count()) /
                                             1000000;

                std_msgs::Float64 finish;
                finish.data = finish_explore_time;
                finish_explore_pub_.publish(finish);

                ROS_WARN("** the exploration process has finished, total_time is %f s **",
                         (finish_explore_time - fael_expl_data->start_explore_time_));

                finish_flag = true; // 标记为“已处理”，避免重复执行
            }


            if (fael_expl_data->need_to_stop_move_)
            {
                ROS_WARN("exploration_finished and need to stop!");
                is_success = false;
                is_finish = true;
                return;
            }
        }

        // 第二步：如果不需要下一轮规划→规划失败
        if (!fael_expl_data->need_to_next_iteration_)
        {
            ROS_WARN_THROTTLE(1.0, "fael_expl_data->need_to_next_iteration_ is false!!!");
            is_success = false;
            is_finish = false;
            return;
        }
        else
        {
            // Planning And Execute
            fael_expl_data->iteration_num_++;// 需要下一轮→迭代次数+1
        }

        // 第三步：调用规划器获取路径
        ROS_INFO("**Planning iteration  %i**", fael_expl_data->iteration_num_);
        bool is_exploration_finish;
        std::vector<control_planner_interface::Path> path_segments;
        if (callForPlanner(fael_expl_data->iteration_num_, path_segments, is_exploration_finish))
        {
            if (path_segments.empty())  // 路径为空→规划失败（需要下一轮）
            {
                ROS_WARN("this iteration get an empty path ,need next planning iteration");
                fael_expl_data->need_to_next_iteration_ = true;
                // 如果规划器说“探索完成”，更新本地状态
                if (!fael_expl_data->exploration_finished_ && is_exploration_finish)
                {
                    fael_expl_data->exploration_finished_ = true;
                }
                is_success = false;
                is_finish = false;
            }
            else    // 路径有效→执行路径
            {
                ROS_INFO("following the path...");

                bool success = false;
                double start_time = 0.0, path_length = 0.0;
                // got a reasonable path
                followThePath(path_segments, success, start_time, path_length, target_pose);

                if (success)  // 执行路径成功→规划成功
                {
                    fael_expl_data->need_to_next_iteration_ = false;
                    is_success = true;
                    is_finish = false;
                }
                else // 执行失败→规划失败
                {
                    is_success = false;
                    is_finish = false;
                }
            }
        }
        else    // 调用规划器超时/失败→规划失败
        {
            // exploration failed
            ROS_WARN("the iteration is out time or aborted");
            fael_expl_data->need_to_next_iteration_ = true;
            is_success = false;
            is_finish = false;
        }
    }

    //  管理状态切换
    void Explorer_FSM::transitState(FAEL_EXPL_STATE new_state, std::string pos_call)
    {
        int pre_s = int(fael_expl_data->state_);
        fael_expl_data->state_ = new_state;
        // 打印“谁触发的切换+从哪个状态到哪个状态”
        std::cout << "[" + pos_call + "]: from " + fael_expl_data->state_string_vec_[pre_s] + 
                                        " to " + fael_expl_data->state_string_vec_[int(new_state)]
                  << std::endl;
    }

    // 从 ROS 参数服务器读配置（如初始位置、速度阈值），缺省值兜底
    bool Explorer_FSM::loadParams()
    {

        const std::string &ns = ros::this_node::getName();

        // 是否启用初始运动（默认不启用）
        if (!nh_private_.param("fael_exploration/init_motion_enable", fael_expl_params->init_motion_enable_, false))
        {
            ROS_WARN("No motion initialization setting, set it to False.");
        }

        // 初始位置偏移（x/y/z，默认0）
        if (!nh_private_.param("fael_exploration/init_x", fael_expl_params->init_x_, 0.0))
        {
            ROS_WARN("No init_x specified. Looking for %s. Default is '0.0'.",
                     (ns + "/init_x").c_str());
        }
        if (!nh_private_.param("fael_exploration/init_y", fael_expl_params->init_y_, 0.0))
        {
            ROS_WARN("No init_y specified. Looking for %s. Default is '0.0'.",
                     (ns + "/init_y").c_str());
        }
        if (!nh_private_.param("fael_exploration/init_z", fael_expl_params->init_z_, 0.0))
        {
            ROS_WARN("No init_z specified. Looking for %s. Default is '0.0'.",
                     (ns + "/init_z").c_str());
        }

        // 最大速度（默认1m/s）
        if (!nh_private_.param("fael_exploration/max_vel", fael_expl_params->max_vel_, 1.0))
        {
            ROS_WARN("No max_vel specified. Looking for %s. Default is '1.0'.",
                     (ns + "/max_vel").c_str());
        }

        // 重规划阈值（等待太久阈值1秒，周期性重规划阈值2秒）
        fael_expl_params->replan_too_long_thre_ = 1.0;
        if (!nh_private_.param("fael_exploration/replan_too_long_thre", fael_expl_params->replan_too_long_thre_, 1.0))
        {
            ROS_WARN("No replan_too_long_thre specified. Looking for %s. Default is '1.0'.",
                     (ns + "/replan_too_long_thre").c_str());
        }
        if (!nh_private_.param("fael_exploration/replan_periodic_thre", fael_expl_params->replan_periodic_thre_, 2.0))
        {
            ROS_WARN("No replan_periodic_thre specified. Looking for %s. Default is '2.0'.",
                     (ns + "/replan_periodic_thre").c_str());
        }

        return true;
    }

    //  初始化
    //  确保系统就绪（控制接口 + 初始运动），并发布探索开始时间：
    bool Explorer_FSM::init()
    {
        // 第一步：初始化控制-规划接口（之前分析的ControlPlannerInterface）
        if (!interface_->init())
        {
            ROS_INFO_THROTTLE(1.0, "control planner interface init failed.");
            return false;
        }

        // 第二步：如果启用初始运动→让无人机飞到初始偏移位置
        if (fael_expl_params->init_motion_enable_)
        {
            if (!initMotion())
            {
                ROS_INFO_THROTTLE(1.0, "explorer init motion failed.");
                return false;
            }
        }

        // 第三步：等待“探索开始”发布者有订阅者（确保有人接收开始时间）
        if (!start_explore_pub_.getNumSubscribers() > 0)
        {
            return false;
        }

        // 第四步：记录并发布探索开始时间
        // publish exploration status
        fael_expl_data->start_explore_time_ = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                                                                      std::chrono::high_resolution_clock::now().time_since_epoch())
                                                                      .count()) /
                                              1000000;
        std_msgs::Float64 is_explore_start;
        is_explore_start.data = fael_expl_data->start_explore_time_;
        start_explore_pub_.publish(is_explore_start);

        return true;
    }

    //  初始运动，让无人机从当前位置飞到 “当前位置 + 偏移量”
    bool Explorer_FSM::initMotion()
    {
        ROS_INFO("Performing initialization motion");
        ROS_INFO("Current pose: %f, %f, %f", interface_->current_pose_.position.x,
                 interface_->current_pose_.position.y, interface_->current_pose_.position.z);

        geometry_msgs::Pose pose;
        // 目标位置=当前位置+偏移量（x/y/z分别加）
        pose.position.x = interface_->current_pose_.position.x + fael_expl_params->init_x_;
        pose.position.y = interface_->current_pose_.position.y + fael_expl_params->init_y_;
        pose.position.z = interface_->current_pose_.position.z + fael_expl_params->init_z_;
        // 目标朝向=当前朝向（不用转头，只平移）
        pose.orientation.x = interface_->current_pose_.orientation.x;
        pose.orientation.y = interface_->current_pose_.orientation.y;
        pose.orientation.z = interface_->current_pose_.orientation.z;
        pose.orientation.w = interface_->current_pose_.orientation.w;
        // 让“手脚”控制无人机飞到这个目标位姿
        interface_->goToWayPose(pose);

        return true;    // 初始运动指令发送成功
    }

    // 调用规划器，获取路径
    bool Explorer_FSM::callForPlanner(const int &iteration_num,
                                      std::vector<control_planner_interface::Path> &path_segments,
                                      bool &is_exploration_finish)
    {
        return interface_->callForPlanner(iteration_num, path_segments, is_exploration_finish);
    }

    // 版本1：只执行路径，不返回详细结果
    void Explorer_FSM::followThePath(const std::vector<control_planner_interface::Path> &path_segments)
    {
        fael_expl_data->follow_start_time_ = ros::WallTime::now().toSec();
        interface_->executePath(path_segments);
    }

    // 版本2：执行路径，并返回详细结果（是否成功、开始时间、路径长度、目标位姿）
    void Explorer_FSM::followThePath(const std::vector<control_planner_interface::Path> &path_segments,
                                     bool &success, double &start_time, double &path_length, geometry_msgs::Pose &goal_pose)
    {
        interface_->executePath(path_segments, success, start_time, path_length, goal_pose);// 让“手脚”执行路径，并把结果存到输出参数里
        fael_expl_data->follow_start_time_ = start_time;// 记录执行开始时间
    }

    // 检查是否等太久
    bool Explorer_FSM::isWaitTooLong()
    {
        if (fael_expl_data->is_start_bspline)// 如果已经开始走B样条路径
        {
            return fael_expl_data->bspline_duration_time_ - (ros::Time::now() - fael_expl_data->bspline_start_time_).toSec() < fael_expl_params->replan_too_long_thre_;
        }
        else // 没走B样条路径→不用判断超时
        {
            return false;
        }
    }

    // 检查是否到周期性重规划时间
    bool Explorer_FSM::isPeriodicReplan()
    {
        // 当前时间 - 执行路径的开始时间 > 周期性重规划阈值（比如2秒）→返回true（要重规划）
        if ((ros::WallTime::now().toSec() - fael_expl_data->follow_start_time_) > fael_expl_params->replan_periodic_thre_)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    // 收到“探索完成”消息的回调
    void Explorer_FSM::explorationFinishCallback(const std_msgs::BoolConstPtr &finish)
    {
        // 如果本地还没标记“探索完成”，且消息说“完成了”
        if (!fael_expl_data->exploration_finished_ && finish->data)
        {
            fael_expl_data->exploration_finished_ = true;
            // 如果当前是“规划路径”或“执行路径”状态→切回“规划路径”处理结束逻辑
            if (fael_expl_data->state_ == FAEL_EXPL_STATE::PLAN_TRAJ ||
                fael_expl_data->state_ == FAEL_EXPL_STATE::EXEC_TRAJ)
            {
                transitState(FAEL_EXPL_STATE::PLAN_TRAJ, "explorationFinishCallback");
                fael_expl_data->need_to_next_iteration_ = true;
            }
        }
    }

    // 收到“停止移动”消息的回调
    void Explorer_FSM::stopMoveCallback(const std_msgs::BoolConstPtr &stop_move_msg)
    {
         // 如果消息说“要停止”→本地标记“需要停止”
        if (stop_move_msg->data)
        {
            fael_expl_data->need_to_stop_move_ = stop_move_msg->data;
        }
    }

    // 版本1：收到Path消息触发（比如其他模块发一条路径当触发信号）
    void Explorer_FSM::triggerCallback(const nav_msgs::PathConstPtr &msg)
    {
        if (msg->poses[0].pose.position.z < -0.1)
            return;
        if (fael_expl_data->state_ != FAEL_EXPL_STATE::WAIT_TRIGGER)
            return;
        ROS_WARN("Triggered!");
        transitState(FAEL_EXPL_STATE::PLAN_TRAJ, "triggerCallback");
    }

    // 版本2：收到服务请求触发（比如通过命令行或其他模块发服务请求）
    bool Explorer_FSM::triggerCallback(fael_exploration::Trigger::Request &req, fael_exploration::Trigger::Response &resp)
    {
        if (!req.trigger || fael_expl_data->state_ != FAEL_EXPL_STATE::WAIT_TRIGGER)
        {
            resp.success = false;
            return false;
        }
        else
        {
            ROS_WARN("Triggered!");
            transitState(FAEL_EXPL_STATE::PLAN_TRAJ, "triggerCallback");
            resp.success = true;

            // // publish exploration status
            // fael_expl_data->start_explore_time_ = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
            //                                               std::chrono::high_resolution_clock::now().time_since_epoch())
            //                                               .count()) /
            //                       1000000;
            // std_msgs::Float64 is_explore_start;
            // is_explore_start.data = fael_expl_data->start_explore_time_;
            // start_explore_pub_.publish(is_explore_start);

            return true;
        }
    }

    // 收到“强制回原点”消息的回调
    void Explorer_FSM::forcedBackToOriginCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        // 如果当前是“规划路径”或“执行路径”状态，且目标点离原点很近（x/y差小于1米）
        if ((fael_expl_data->state_ == FAEL_EXPL_STATE::PLAN_TRAJ ||
             fael_expl_data->state_ == FAEL_EXPL_STATE::EXEC_TRAJ) &&
            (fabs(msg->pose.position.x - fael_expl_params->init_x_) < 1 &&
             fabs(msg->pose.position.y - fael_expl_params->init_y_) < 1))
        {
            ROS_ERROR("-----------------forcedBackToOriginCallback!----------------");
            transitState(FAEL_EXPL_STATE::PLAN_TRAJ, "forcedBackToOriginCallback"); // 切到“规划路径”
            fael_expl_data->need_to_next_iteration_ = true; // 需要下一轮规划
            fael_expl_data->exploration_finished_ = true;   // 标记“探索完成”
        }
    }

    // 收到“目标扫描完成”消息的回调（比如无人机到了一个点，扫描完环境，要去下一个点）
    void Explorer_FSM::goalScanCallback(const std_msgs::BoolConstPtr &goal_scan_msg)
    {
        // 如果当前是“执行路径”状态，且消息说“扫描完成”，且探索没结束
        if (fael_expl_data->state_ == FAEL_EXPL_STATE::EXEC_TRAJ && goal_scan_msg->data && !fael_expl_data->exploration_finished_)
        {
            fael_expl_data->need_to_next_iteration_ = true; // 需要下一轮规划（去下一个点）
            transitState(FAEL_EXPL_STATE::PLAN_TRAJ, "goalScanCallback");   // 切到“规划路径”
            ROS_ERROR("FAEL_Replan: goalScanCallback~~");   // 打印“扫描完成，重规划”
        }
    }

    // 收到里程计消息的回调（里程计是无人机的“GPS”，告诉当前位置）
    void Explorer_FSM::odomCallback(const nav_msgs::OdometryConstPtr &odom_msg)
    {
        // 从里程计消息里提取当前位姿，存到“状态本”里
        fael_expl_data->current_pose_ = odom_msg->pose.pose;
    }

    // 收到B样条轨迹消息的回调
    void Explorer_FSM::bsplineCallback(const bspline::BsplineConstPtr &msg)
    {
        fael_expl_data->is_start_bspline = true;    // 标记“开始走B样条路径”
        fael_expl_data->bspline_start_time_ = msg->start_time;  // 记录轨迹开始时间
        fael_expl_data->bspline_duration_time_ = msg->duration_time;    // 记录轨迹总时长
    }
}