//
// Created by hjl on 2021/11/23.
//

//一边接收无人机的实时状态（里程计），一边调用路径规划器获取路径，
//同时将规划结果和控制指令转发给底层控制管理类（PCIManager），最终实现无人机的路径跟踪或目标点移动。
#include "control_planner_interface/control_planner_interface.h"

namespace interface
{

    ControlPlannerInterface::ControlPlannerInterface(ros::NodeHandle &nh, ros::NodeHandle &nh_private,
                                                     std::shared_ptr<PCIManager> &pci_manager)
        : nh_(nh), nh_private_(nh_private), pci_manager_(pci_manager),
          planner_client_(nh_, "topo_planner", true)    
    {
        // 订阅“odometry”话题（相对节点命名空间的路径），回调函数为odometryCallback
        odometry_sub_ = nh_.subscribe("odometry", 1, &ControlPlannerInterface::odometryCallback, this);

        if (!loadParams())
        {
            ROS_ERROR("control planner interface can not load params. Shut down ROS node.");
            ros::shutdown();
        }
        ROS_INFO("control planner interface construct");
    }

    ControlPlannerInterface::ControlPlannerInterface(ros::NodeHandle &nh_private,
                                                     std::shared_ptr<PCIManager> &pci_manager)
        : nh_private_(nh_private), pci_manager_(pci_manager),
          planner_client_(nh_private_, "/topo_planner", true)
    {

        odometry_sub_ = nh_private_.subscribe("/odometry", 1, &ControlPlannerInterface::odometryCallback, this);

        if (!loadParams())
        {
            ROS_ERROR("control planner interface can not load params. Shut down ROS node.");
            ros::shutdown();
        }
        ROS_INFO("control planner interface construct");
    }


    bool ControlPlannerInterface::loadParams()
    {

        const std::string &ns = ros::this_node::getName();

        frame_id_ = "world";
        if (!ros::param::get(ns + "/frame_id", frame_id_))
        {
            ROS_WARN("No frame_id specified. Looking for %s. Default is 'world'.",
                     (ns + "/frame_id").c_str());
        }

        pci_manager_->setFrameId(frame_id_);

        if (!pci_manager_->loadParams(ns))
            return false;

        return true;
    }

    // 定义里程计回调函数：收到里程计消息时自动执行
    void ControlPlannerInterface::odometryCallback(const nav_msgs::OdometryConstPtr &odom)
    {
        current_pose_ = odom->pose.pose;
        pci_manager_->setCurrentPose(current_pose_);

        pose_is_ready_ = true;
    }

    bool ControlPlannerInterface::init()
    {
        pose_is_ready_ = false;

        // checking odometry is ready.
        ros::Rate rr(10);
        while (!pose_is_ready_)
        {
            ROS_WARN("Waiting for odometry.");  // 处理ROS的“待办消息”（比如里程计消息可能已经到了，这里触发回调函数去处理）
            ros::spinOnce();
            rr.sleep();
        }

        if (!pci_manager_->initialize())
            return false;

        return true;
    }

    void ControlPlannerInterface::goToWayPose(geometry_msgs::Pose &pose)    // 定义“去目标位姿”函数：让无人机飞到指定的位置和姿态
    {
        pci_manager_->goToWaypoint(pose);   // 把目标位姿传给PCIManager：“控制工具，你让无人机飞到这个点”
    }

    bool ControlPlannerInterface::isGoalReached()
    {
        return pci_manager_->isGoalReached();
    }

    void ControlPlannerInterface::cancelCurrentGoal()
    {
        pci_manager_->cancelCurrentGoal();
    }
    // 版本1：执行规划好的路径（路径是多个“路径段”组成的列表）
    void ControlPlannerInterface::executePath(const std::vector<control_planner_interface::Path> &path_segments)
    {
        std::vector<geometry_msgs::Pose> modified_path; // 定义一个“修改后的路径”变量（PCIManager可能会调整路径点，存在这里）
        pci_manager_->executePath(path_segments, modified_path);    // 把路径段传给PCIManager：让无人机按这个路径飞，调整后的路径放modified_path
    }

    // 版本2：执行路径+返回更多信息（成功与否、开始时间、路径长度、目标位姿）
    void ControlPlannerInterface::executePath(const std::vector<control_planner_interface::Path> &path_segments, bool &success,
                                              double &start_time, double &path_length, geometry_msgs::Pose &goal_pose)
    {
        pci_manager_->executePath(path_segments, success, start_time, path_length, goal_pose);  // 让PCIManager执行路径，并把结果存到输出参数里（上层调用后能知道飞行是否成功、路径多长等）
    }

    // 调用路径规划器
    bool ControlPlannerInterface::callForPlanner(const int &iteration_id,
                                                 std::vector<control_planner_interface::Path> &path_segments,
                                                 bool &is_exploration_finish)
    {
        control_planner_interface::ExplorerPlannerGoal goal;
        goal.iteration_id = iteration_id;   // 把迭代号赋值给目标对象（规划器知道要规划哪一轮的路径）
        ROS_INFO("call for the path to explore..");
        if (planner_client_.sendGoalAndWait(goal, ros::Duration(60.0), ros::Duration(10.0)) ==
            actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("the path get.");
            path_segments = planner_client_.getResult()->paths;
            is_exploration_finish = planner_client_.getResult()->is_exploration_finish;
            return true;
        }
        else
        {
            ROS_INFO("planner timeout.");
            planner_client_.cancelAllGoals();
            return false;
        }
    }

    void ControlPlannerInterface::stopMove()
    {
        pci_manager_->stopMove();
    }
}