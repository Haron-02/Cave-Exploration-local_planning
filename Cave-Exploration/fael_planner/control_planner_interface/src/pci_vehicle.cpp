//
// Created by hjl on 2021/11/30.
//

// PCIVehicle类 的实现，是无人机 / 移动机器人的 “具体执行层”
// 把上层的 “路径指令” 或 “目标点指令”，包装成底层执行模块能识别的消息（通过 ROS Action 通信），
// 同时提供 “状态查询”“取消任务”“紧急停止” 等功能，是连接 “控制逻辑” 和 “底层执行器” 的关键类
#include "control_planner_interface/pci_vehicle.h"

namespace interface
{

    PCIVehicle::PCIVehicle(const ros::NodeHandle &nh,
                           const ros::NodeHandle &nh_private) : PCIManager(nh, nh_private),
                                                                vehicle_execute_client_(nh_, "vehicle_execute", true),
                                                                vehicle_stop_client_(nh_, "vehicle_stop", true),
                                                                n_seq_(0)
    {
        ROS_INFO("pci vehicle construct");
    }

    PCIVehicle::PCIVehicle(const ros::NodeHandle &nh_private) : PCIManager(nh_private),
                                                                vehicle_execute_client_(nh_private_, "/vehicle_execute", true),
                                                                vehicle_stop_client_(nh_private_, "/vehicle_stop", true),
                                                                n_seq_(0)
    {
        ROS_INFO("pci vehicle construct");
    }

    // 定义“加载参数”函数
    bool PCIVehicle::loadParams(const std::string ns)
    {
        return true;
    }

    bool PCIVehicle::initialize()
    {
        pci_status_ = PCIStatus::kReady;

        return true;
    }

    void PCIVehicle::setCurrentPose(const geometry_msgs::Pose &pose)
    {
        current_pose_.position.x = pose.position.x;
        current_pose_.position.y = pose.position.y;
        current_pose_.position.z = pose.position.z;
        // 存姿态四元数的值x,y,z,w
        current_pose_.orientation.x = pose.orientation.x;
        current_pose_.orientation.y = pose.orientation.y;
        current_pose_.orientation.z = pose.orientation.z;
        current_pose_.orientation.w = pose.orientation.w;
    }

    // 定义“设置坐标系”函数：告诉类用哪个参考系（比如“world”世界坐标系）
    void PCIVehicle::setFrameId(const std::string &frame_id)
    {
        frame_id_ = frame_id;   // 把传入的坐标系名字存到“frame_id_”里
    }

    // 版本 1：异步执行（发完指令不等结果）
    bool PCIVehicle::executePath(const std::vector<control_planner_interface::Path> &path_segments,
                                 std::vector<geometry_msgs::Pose> &modified_path)
    {
        n_seq_++;
        control_planner_interface::VehicleExecuteGoal path_goal;    // 创建“执行路径的目标消息”：相当于写一张“任务单”，告诉底层要干什么
        path_goal.header.seq = n_seq_;
        path_goal.header.stamp = ros::Time::now();
        path_goal.header.frame_id = frame_id_;
        path_goal.paths = path_segments;
        vehicle_execute_client_.sendGoal(path_goal);
        ros::spinOnce();

        return true;
    }

    // 版本 2：同步执行（发完指令等结果，返回详细信息）
    bool PCIVehicle::executePath(const std::vector<control_planner_interface::Path> &path_segments,
                                 bool &success, double &start_time, double &path_length, geometry_msgs::Pose &goal_pose)
    {
        n_seq_++;
        control_planner_interface::VehicleExecuteGoal path_goal;
        path_goal.header.seq = n_seq_;
        path_goal.header.stamp = ros::Time::now();
        path_goal.header.frame_id = frame_id_;
        path_goal.paths = path_segments;
        if (vehicle_execute_client_.sendGoalAndWait(path_goal, ros::Duration(60.0), ros::Duration(10.0)) ==
            actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            success = vehicle_execute_client_.getResult()->success;
            path_length = vehicle_execute_client_.getResult()->executed_path_length;
            start_time = vehicle_execute_client_.getResult()->executed_start_time;
            goal_pose = vehicle_execute_client_.getResult()->goal_pose;
        }
        return true;
    }

    // 去目标点 
    bool PCIVehicle::goToWaypoint(const geometry_msgs::Pose &pose)
    {
        ROS_WARN("***PCIVehicle::goToWaypoint***");
        control_planner_interface::Path path;
        path.path.push_back(current_pose_);
        path.path.push_back(pose);
        std::vector<control_planner_interface::Path> path_segments;
        path_segments.push_back(path);
        std::vector<geometry_msgs::Pose> modified_path;
        executePath(path_segments, modified_path);

        return true;
    }

    // 判断是否到达目标
    bool PCIVehicle::isGoalReached()
    {
        if (vehicle_execute_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            return true;
        else
        {
            return false;
        }
    }

    // 取消当前任务（cancelCurrentGoal）
    void PCIVehicle::cancelCurrentGoal()
    {
        vehicle_execute_client_.cancelGoal();
    }

    // 紧急停止
    void PCIVehicle::stopMove()
    {
        ROS_WARN("stopMove++++++++++++++++++");
        control_planner_interface::VehicleExecuteGoal goal;
        goal.paths.clear();
        vehicle_stop_client_.sendGoal(goal);
    }
}