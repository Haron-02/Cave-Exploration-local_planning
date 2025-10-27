//  无人机 / 移动机器人的 “路径执行器”
//  把上层规划好的 “全局路径”，转换成机器人能实时执行的 “局部目标”
//  “接任务（规划好的路径）→ 做处理（补全路径、避障检查）→ 给指令（告诉机器人当前飞哪个点）→ 给反馈（执行得怎么样、有没有停下来）”
//
// Created by hjl on 2021/12/15.
//

#include "path_execution/path_execution.h"

namespace path_execution
{

    //开机初始化
    PathExecution::PathExecution(
        const ros::NodeHandle &nh_private) : nh_private_(nh_private),
                                             // 1. 初始化“路径执行请求处理器”（处理外部发来的“执行路径”请求）
                                             execute_server_(nh_private_, "/path_execution/vehicle_execute", boost::bind(&PathExecution::executeCallback, this, _1),
                                                             false),
                                             // 2. 初始化“停止请求处理器”（处理外部发来的“停止移动”请求）
                                             stop_move_server_(nh_private_, "/path_execution/vehicle_stop", boost::bind(&PathExecution::stopMoveCallback, this, _1),
                                                               false)
    {

        // 3. 订阅“里程计”（相当于机器人的GPS，实时获取当前位置/速度）
        odom_sub_ = nh_private_.subscribe("/odometry", 1, &PathExecution::odomCallback, this);
        // 4. 初始化“路径可视化发布器”（在RViz里显示各种路径，方便人看）
        local_path_pub_ = nh_private_.advertise<nav_msgs::Path>("/path_execution/local_path", 1);
        way_pose_pub_ = nh_private_.advertise<geometry_msgs::PoseStamped>("/path_execution/way_pose", 1);   // 当前目标点
        executed_path_pub_ = nh_private_.advertise<nav_msgs::Path>("/path_execution/executed_path", 1); // 已执行路径
        smooth_path_pub_ = nh_private_.advertise<visualization_msgs::Marker>("/path_execution/local_smooth_path", 1);   //平滑路径
        look_ahead_goal_pub_ = nh_private_.advertise<geometry_msgs::PoseStamped>("/path_execution/look_ahead_goal", 1); //前瞻目标点
        // 5. 从参数服务器读配置（没有配置就用默认值）
        nh_private_.param("path_execution/init_z", init_z_, 2.0);   // 初始高度（默认2米）
        nh_private_.param("path_execution/stop_vel_thres", stop_vel_thres_, 0.05);
        nh_private_.param("path_execution/stop_rot_thres", stop_rot_thres_, 0.09);
        nh_private_.param("path_execution/local_path_dist", local_path_dist_, 5.0); //局部路径最大距离（只处理前面5米的路径，太远不管）

        // 6. 启动两个请求处理器（开始接收外部请求）
        execute_server_.start();
        stop_move_server_.start();
        ROS_DEBUG("started server");
    }

    // 装2D地图（平面地图，比如地面机器人用）
    void PathExecution::setMap2DManager(Map2DManager::Ptr &map2d_manager)
    {
        this->map2d_manager_ = map2d_manager;   // 把2D地图存到小助手的“map2d_manager_”变量里
        global_frame_ = map2d_manager_->m2dp_.frame_id_;    // 记地图的坐标系（比如“world”，和机器人位置的坐标系一致）
        grid_size_ = map2d_manager_->m2dp_.grid_size_;  // 记地图的网格大小（比如0.1米/格，后面加中间点用）
    }

    // 装3D地图（立体地图，比如无人机用，能知道空中有没有障碍）
    void PathExecution::setMap3DManager(Map3DManager::Ptr &map3d_manager)
    {
        this->map3d_manager_ = map3d_manager;
        global_frame_ = map3d_manager->m3dp_.frame_id_;
        grid_size_ = map3d_manager->m3dp_.grid_size_;
    }

    // 维度：在平面飞还是立体飞”（2=平面，3=立体）
    void PathExecution::setPlannerDim(const int &planner_dim)
    {
        this->planner_dim_ = planner_dim;   // 存维度（后面处理路径会按维度区分）
    }

    // 里程计回调：收到新位置后更新当前位置
    void PathExecution::odomCallback(const nav_msgs::OdometryConstPtr &odom)
    {
        last_pose_ = current_pose_; // 把“当前位置”存成“上一次的位置”（方便算速度）
        current_pose_ = odom->pose.pose;    // 用新收到的位置更新“当前位置”

        last_odom_ = current_odom_; // 同理，更新“上一次的完整里程计数据”
        current_odom_ = *odom;  // 更新“当前的完整里程计数据”
    }

    // 停止请求回调：收到“停下”请求后执行
    void PathExecution::stopMoveCallback(const control_planner_interface::VehicleExecuteGoalConstPtr &stop_goal)
    {
        // wait until we're stopped before returning success
        ROS_WARN("start stop ...");
        // ROS_INFO("start stop ...");

        stop_move_server_.setSucceeded();   // 告诉发请求的模块“停止请求已处理”
    }

    // 判断是否停下来
    bool PathExecution::isStopped()
    {
        // 1. 算“当前位置”和“上一次位置”的距离差（平移距离）
        double dist = sqrt(pow(current_pose_.position.x - last_pose_.position.x, 2) +
                           pow(current_pose_.position.y - last_pose_.position.y, 2) +
                           pow(current_pose_.position.z - last_pose_.position.z, 2));
        // 2. 算“当前朝向”和“上一次朝向”的角度差（处理超过180度的情况，比如转了270度相当于反着转90度）
        double yaw_delta = fabs(tf::getYaw(current_pose_.orientation) - tf::getYaw(last_pose_.orientation));
        if (yaw_delta > M_PI)
        {
            yaw_delta = 2 * M_PI - yaw_delta;   // 把大于180角度的差值转换成小于180度
        }
        // 3. 算“上一次”到“当前”的时间差
        double odom_delta_t = (current_odom_.header.stamp - last_odom_.header.stamp).toSec();
        // 4. 速度=距离/时间，转向速度=角度/时间，都小于阈值→算停下
        if ((dist / odom_delta_t) < stop_vel_thres_ && (yaw_delta / odom_delta_t) < stop_rot_thres_)
        {
            ROS_INFO("the robot is stopped.");
            return true;
        }
        else
        {
            return false;
        }
    }

    // 飞行路径请求回调：收到路径请求后执行
    void PathExecution::executeCallback(const control_planner_interface::VehicleExecuteGoalConstPtr &executed_path)
    {
        // 1. 初始化“执行结果”（默认失败，后面成功了再改）
        control_planner_interface::VehicleExecuteResult executed_result;
        executed_result.success = false;
        executed_result.executed_path_length = 0.0;
        executed_result.executed_start_time = ros::WallTime::now().toSec();
        // 2. 如果发请求的模块说“取消执行”，直接回复失败
        if (execute_server_.isPreemptRequested())
        {
            execute_server_.setPreempted(executed_result);
        }
        // 3. 关键步骤：找“当前该飞的点”（前瞻点）
        bool success;
        double executed_start_time, executed_path_length;
        geometry_msgs::Pose goal_pose;

        findLlookAheadPoint(executed_path->paths, success, executed_start_time, executed_path_length, goal_pose);

        // 4. 填充执行结果，告诉发请求的模块“执行得怎么样”
        executed_result.success = success;
        executed_result.executed_start_time = executed_start_time;
        executed_result.executed_path_length = executed_path_length;
        executed_result.goal_pose = goal_pose;
        execute_server_.setSucceeded(executed_result);  // 回复“执行完成”
    }

    // 找前瞻点：path_segments是收到的路径；success=找没找到；goal_pose是找到的目标点
    void PathExecution::findLlookAheadPoint(const std::vector<control_planner_interface::Path> &path_segments, bool &success,
                                            double &executed_start_time, double &executed_path_length, geometry_msgs::Pose &goal_pose)
    {
        // 1. 先初始化结果（默认没找到）
        {
            success = false;
            executed_path_length = 0.0;
            executed_start_time = ros::WallTime::now().toSec();
        }

        double start_time = ros::WallTime::now().toSec();
        geometry_msgs::Pose look_ahead_point;   // 要找的“当前该飞的点”
        bool is_get_look_ahead_point = false;   // 标记“是否找到”

        // 2. 如果路径不为空（有路径可处理）
        if(!path_segments.empty())
        {
            Path path = path_segments[0].path;
        
            for(int i = (path.size()-1); i>=0; i--)
            {
                // 把路径上的点转成3D格式，问地图“这个点能走？”
                GridPoint3D point(path[i].position.x, path[i].position.y, path[i].position.z);
                if (map3d_manager_->isFree(point))  // “能走”
                {
                    // 4. 算机器人朝这个点飞的朝向（不能乱飞，得对着目标点）
                    GridPoint3D current_position(current_pose_.position.x, current_pose_.position.y, current_pose_.position.z);
                                                tf::Vector3 axis(0, 0, 1);  // 绕z轴转（控制机器人的朝向，比如朝东还是朝北）
                    float angle = std::atan2(point.y() - current_pose_.position.y,   
                                                point.x() - current_pose_.position.x);// 用atan2算朝向角度（目标点在机器人的哪个方向）
                    tf::Quaternion quaternion(axis, angle);
                    // 5. 填目标点的位置和朝向
                    look_ahead_point.orientation.x = quaternion.getX();
                    look_ahead_point.orientation.y = quaternion.getY();
                    look_ahead_point.orientation.z = quaternion.getZ();
                    look_ahead_point.orientation.w = quaternion.getW();
                    look_ahead_point.position.x = point.x();
                    look_ahead_point.position.y = point.y();
                    look_ahead_point.position.z = point.z();
                    // 6. 标记“找到点了”，跳出循环（不用再查前面的点了）
                    is_get_look_ahead_point = true;
                    success = true;
                    executed_start_time = start_time;
                    goal_pose = look_ahead_point;
                    break;
                }
            }

        }
        else
        {
            ROS_ERROR("path_segments_ empty!"); // 路径是空的，报错
        }



        // if (!setPathToFollow(path_segments))
        // {
        //     ROS_ERROR("Failed to pass global plan to the controller, aborting.");
        //     return;
        // }

        // if (!global_path_to_follow_.empty())
        // {
        //     ROS_WARN("Get global_path_to_follow_");
            
        //     lookAheadPointControlLoop(success, executed_start_time, executed_path_length, goal_pose);
        //     if (success)
        //     {
        //         geometry_msgs::PoseStamped look_ahead_goal_pose_msg;
        //         look_ahead_goal_pose_msg.header.frame_id = global_frame_;
        //         look_ahead_goal_pose_msg.header.stamp = ros::Time::now();
        //         look_ahead_goal_pose_msg.pose = goal_pose;
        //         look_ahead_goal_pub_.publish(look_ahead_goal_pose_msg);
        //     }
        // }
    }

    // 合并路径并存起来：path_segments是收到的路径段
    bool PathExecution::setPathToFollow(const std::vector<control_planner_interface::Path> &path_segments)
    {
        if (path_segments.empty())
        {
            ROS_WARN("the received path_segments is emtpy..");
            return false;
        }

        path_segments_.clear(); // 清空旧的路径段
        global_path_to_follow_.clear(); // 清空旧的“要飞的全局路径”
        Path path;  // 临时存合并后的路径
        // 1. 遍历所有路径段，合并成一条
        for (int i = 0; i < path_segments.size(); i++)
        {
            ROS_INFO("this path segment size is %zu ", path_segments[i].path.size());   // 打印当前路径段有多少个点
            path.insert(path.end(), path_segments[i].path.begin(), path_segments[i].path.end());  // 把当前路径段的点加到合并路径里
            Path interpolate_segment = interpolatePath(path_segments[i].path);   // 给当前路径段“加中间点”（插值），让飞行更平滑
            path_segments_.push_back(interpolate_segment);  // 存插值后的路径段
            global_path_to_follow_.insert(global_path_to_follow_.end(), interpolate_segment.begin(),
                                          interpolate_segment.end());   // 把插值后的点加到“要飞的全局路径”里
        }

        // 2. 打印路径信息，发布“已经飞过的路径”（可视化）
        ROS_INFO(" the followed global path size is %zu", path.size()); // 合并后的路径有多少个点
        ROS_INFO("global path to be executed size: %lu", global_path_to_follow_.size());    // 插值后有多少个点
        publishPath(global_path_to_follow_, executed_path_pub_);    // 画路径

        return true;
    }

    // 给路径加中间点：path是原路径；返回插值后的路径
    Path PathExecution::interpolatePath(const Path &path) const
    {
        ROS_INFO("interpolate the path..");
        Path interpolated_path; // 存插值后的路径

        if (path.size() >= 2)   // 至少两个点才需要加中间点
        {
            for (int i = 0; i < (path.size() - 1); ++i)
            {
                interpolated_path.push_back(path[i]);   // 先把当前点加进去

                // Interpolate each segment.
                tf::Vector3 start(path[i].position.x, path[i].position.y, path[i].position.z);
                tf::Vector3 end(path[i + 1].position.x, path[i + 1].position.y, path[i + 1].position.z);
                double distance = (end - start).length();   // 两点之间的距离
                double step_size = grid_size_;  // 步长（比如0.1米，从地图网格大小来）
                // 算第一个中间点的位置（从当前点往下一步）
                tf::Vector3 inner_point = start + (end - start).normalized() * step_size;

                // 2. 算朝向（朝向下一个点）
                double yaw = std::atan2(end.y() - start.y(), end.x() - start.x());
                tf::Quaternion quat(tf::Vector3(0, 0, 1), yaw);

                // 3. 循环加中间点，直到超过下一个点
                while ((inner_point - start).length() < distance)
                {
                    tf::Pose poseTF(quat, inner_point); // 把“位置+朝向”转成机器人能懂的位姿
                    geometry_msgs::Pose pose;
                    tf::poseTFToMsg(poseTF, pose);  // 转成ROS能传的消息格式
                    interpolated_path.push_back(pose);  // 加中间点
                    inner_point += (end - start).normalized() * step_size;  // 算下一个中间点
                }
            }

            interpolated_path.push_back(path.back());   // 加最后一个点

            return interpolated_path;   // 返回加完中间点的路径
        }
        else
        {
            return path;
        }
    }

    // 画路径：path是要画的路径；pub是用哪个显示器（发布器）画
    void PathExecution::publishPath(const Path &path, const ros::Publisher &pub)
    {
        // given an empty path we won't do anything
        if (path.empty())
            return;

        // create a path message
        nav_msgs::Path gui_path;
        gui_path.poses.resize(path.size()); // 路径有多少个点，消息就有多少个点
        gui_path.header.frame_id = global_frame_;   // 用和地图一致的坐标系（避免画歪）
        gui_path.header.stamp = ros::Time::now();

        // extract the plan in world co-ordinates, we assume the path is all in the same frame
        for (unsigned int i = 0; i < path.size(); i++)
        {
            gui_path.poses[i].pose = path[i];
            gui_path.poses[i].header = gui_path.header;
        }
        pub.publish(gui_path);
    }

    // 循环查找“当前能飞的前瞻点”
    void PathExecution::lookAheadPointControlLoop(bool &success, double &executed_start_time,
                                                  double &executed_path_length, geometry_msgs::Pose &goal_pose)
    {
        // 1. 初始化返回结果（默认“没找到前瞻点”）
        {
            success = false;
            executed_start_time = ros::WallTime::now().toSec();
            executed_path_length = 0.0;
        }

        // 2. 定义局部变量：标记是否找到前瞻点、已执行路径长度
        bool is_get_look_ahead_point = false;
        double executed_length = 0.0;
        ROS_WARN("the will be executed point num of the interpolated path is %zu", global_path_to_follow_.size());

        // 3. 如果“待执行的全局路径”是空的→直接返回失败
        if (global_path_to_follow_.empty())
        {
            ROS_WARN("global_path_to_follow_.empty()");
            success = false;
            executed_start_time = ros::WallTime::now().toSec();
            executed_path_length = 0.0;
            return;
        }

        // 4. 初始化循环变量：路径段索引、开始时间、前瞻点
        int ind = 0;
        double start_time = ros::WallTime::now().toSec();
        geometry_msgs::Pose look_ahead_point;

        // 5. 循环处理每个路径段（ind是当前处理的路径段索引）
        while (ind < path_segments_.size())
        {
            Path current_path = path_segments_[ind];
            bool being_shortened = false;

            // 6. 如果是2D规划
            if (planner_dim_ == 2)
            {
                // 倒着查当前路径段的点（从最后一个点往第一个点查）→找“最远的可通行点”
                for (int i = static_cast<int>(current_path.size() - 1); i >= 0; --i)
                {
                    // 把当前路径点转成2D格式（只取x/y，忽略z）
                    Point2D point(current_path[i].position.x, current_path[i].position.y);
                    // 找到这段path中在map2d范围内最远的free点
                    if (map2d_manager_->isFree(point))
                    {
                        // 找到到最远的free点的路径
                        std::vector<Point2D> path_2d = makeLocalPlan2D(current_path[i], current_pose_, true);
                        if (path_2d.size() > 1)
                        {
                            ROS_WARN("get local current_path to the point %d currently ", i);

                            geometry_msgs::PoseStamped way_pose;
                            way_pose.header.frame_id = global_frame_;
                            way_pose.header.stamp = ros::Time::now();
                            way_pose.pose = current_path[i];
                            way_pose_pub_.publish(way_pose);

                            // 将path_2d和end_pose变成path_msg并发布
                            Path path_msg = generatePath(path_2d);
                            publishPath(path_msg, local_path_pub_);

                            // 计算“当前位置”和“要找的前瞻点”之间的路径
                            Point2D current_position(current_pose_.position.x, current_pose_.position.y);
                            Point2D look_ahead_goal;
                            // 倒着查局部路径的点，找符合距离要求的前瞻点
                            for (int j = static_cast<int>(path_2d.size() - 1); j >= 0; --j)
                            {
                                // 算“当前位置”到“第j个局部路径点”的距离
                                double distance = sqrt(pow(path_2d[j].x() - current_position.x(), 2) +
                                                       pow(path_2d[j].y() - current_position.y(), 2));
                                // 情况1：距离超过“局部路径最大距离”（比如5米，避免飞太远）
                                if (distance > local_path_dist_)
                                {
                                    if (j == 1)
                                    {
                                        look_ahead_goal = current_position + (path_2d[j] - current_position).normalized() * local_path_dist_;
                                        executed_length = local_path_dist_;
                                    }
                                    else
                                    {
                                        being_shortened = true;
                                        continue;
                                    }
                                }
                                // 情况2：距离小于1米（太近，往前多飞1米，避免频繁调整）
                                else if (distance < 1.0)
                                {
                                    look_ahead_goal = path_2d[j] + (path_2d[j] - current_position).normalized() * 1.0;
                                    executed_length = distance + 1.0;
                                }
                                // 情况3：距离正常（1米~最大距离之间）
                                else
                                {
                                    if (being_shortened)
                                    {
                                        look_ahead_goal = path_2d[j] + (path_2d[j + 1] - path_2d[j]).normalized() * (local_path_dist_ - distance);
                                        executed_length = local_path_dist_;
                                    }
                                    else
                                    {
                                        look_ahead_goal = path_2d[j];
                                        executed_length = distance;
                                    }
                                }
                                break;
                            }
                            tf::Vector3 axis(0, 0, 1);
                            float angle = std::atan2(look_ahead_goal.y() - current_pose_.position.y,
                                                     look_ahead_goal.x() - current_pose_.position.x);
                            tf::Quaternion quaternion(axis, angle);
                            // 填充前瞻点的位置和朝向
                            look_ahead_point.orientation.x = quaternion.getX();
                            look_ahead_point.orientation.y = quaternion.getY();
                            look_ahead_point.orientation.z = quaternion.getZ();
                            look_ahead_point.orientation.w = quaternion.getW();
                            look_ahead_point.position.x = look_ahead_goal.x();
                            look_ahead_point.position.y = look_ahead_goal.y();
                            look_ahead_point.position.z = init_z_;  // 2D模式下z用初始高度
                            is_get_look_ahead_point = true;
                            break;
                        }
                    }
                } // end for
            }
            //如果是3D规划
            else if (planner_dim_ == 3)
            {
                // 同样倒着查当前路径段的点→找“最远的可通行点”
                for (int i = static_cast<int>(current_path.size() - 1); i >= 0; --i)
                {
                    GridPoint3D point(current_path[i].position.x, current_path[i].position.y, current_path[i].position.z);
                    // 找到这段path中在当前local_map中最远的free点
                    if (map3d_manager_->isFree(point))
                    {
                        // 找到到最远的free点的路径  把当前路径点转成3D格式（x/y/z都要）
                        std::vector<GridPoint3D> path_3d = makeLocalPlan3D(current_path[i], current_pose_, true);
                        if (path_3d.size() > 1) // 如果局部路径有至少2个点→路径有效
                        {
                            geometry_msgs::PoseStamped way_pose;
                            way_pose.header.frame_id = global_frame_;
                            way_pose.header.stamp = ros::Time::now();
                            way_pose.pose = current_path[i];
                            way_pose_pub_.publish(way_pose);

                            // 将path_3d和end_pose变成path_msg
                            Path path_msg = generatePath(path_3d);
                            publishPath(path_msg, local_path_pub_);

                            // 发布look_ahead_point 计算“当前位置”和“前瞻点”
                            GridPoint3D current_position(current_pose_.position.x, current_pose_.position.y, current_pose_.position.z);
                            GridPoint3D look_ahead_goal = path_3d.back();   // 先默认用局部路径的最后一个点
                            for (int j = static_cast<int>(path_3d.size() - 1); j >= 0; --j)
                            {
                                // 算3D距离（x/y/z三个方向的距离平方和开根号）
                                double distance = sqrt(pow(path_3d[j].x() - current_position.x(), 2) +
                                                       pow(path_3d[j].y() - current_position.y(), 2) +
                                                       pow(path_3d[j].z() - current_position.z(), 2));
                                if (distance > local_path_dist_)
                                {
                                    if (j == 1)
                                    {
                                        look_ahead_goal = current_position + (path_3d[j] - current_position).normalized() * local_path_dist_;
                                        executed_length = local_path_dist_;
                                    }
                                    else
                                    {
                                        being_shortened = true;
                                        continue;
                                    }
                                }
                                else if (distance < 1.0)
                                {
                                    look_ahead_goal = path_3d[j] + (path_3d[j] - current_position).normalized() * 1.0;
                                    executed_length = distance + 1.0;
                                }
                                else
                                {
                                    if (being_shortened)
                                    {
                                        look_ahead_goal = path_3d[j] + (path_3d[j + 1] - path_3d[j]).normalized() * (local_path_dist_ - distance);
                                        executed_length = local_path_dist_;
                                    }
                                    else
                                    {
                                        look_ahead_goal = path_3d[j];
                                        executed_length = distance;
                                    }
                                }
                                break;
                            }

                            //  计算3D前瞻点的朝向
                            tf::Vector3 axis(0, 0, 1);
                            float angle = std::atan2(look_ahead_goal.y() - current_pose_.position.y,
                                                     look_ahead_goal.x() - current_pose_.position.x);
                            tf::Quaternion quaternion(axis, angle);
                            //  填充3D前瞻点的位置和朝向（z用3D路径的z，不是固定值）
                            look_ahead_point.orientation.x = quaternion.getX();
                            look_ahead_point.orientation.y = quaternion.getY();
                            look_ahead_point.orientation.z = quaternion.getZ();
                            look_ahead_point.orientation.w = quaternion.getW();
                            look_ahead_point.position.x = look_ahead_goal.x();
                            look_ahead_point.position.y = look_ahead_goal.y();
                            look_ahead_point.position.z = look_ahead_goal.z();
                            is_get_look_ahead_point = true;

                            break;
                        }
                        else
                        {
                            ROS_ERROR("path_3d in Path Execution is empty~~~~~~~~~~~~");
                        }
                    }
                } // for end
            }
            else
            {
                ROS_ERROR("perception_dim_ error!");
                std::cerr << planner_dim_ << std::endl;
            }

            if (is_get_look_ahead_point || path_segments_.size() <= 2 || ind++ >= 1)
            {
                break;
            }
        } // while end

        //  如果找到前瞻点→更新返回结果
        if (is_get_look_ahead_point)
        {
            success = true;
            executed_start_time = start_time;
            executed_path_length = executed_length;
            goal_pose = look_ahead_point;
            return;
        }

        return;
    }

    //  找“从当前位置（start）到目标点（goal）”的2D绕障路径
    std::vector<Point2D>
    PathExecution::makeLocalPlan2D(geometry_msgs::Pose &goal, geometry_msgs::Pose &start, const bool &is_clear_nearby)
    {
        std::vector<Point2D> optimal_path;  // 存储绕障后的路径
        // 1. 把ROS Pose转成2D点（只取x/y）
        Point2D start_2d(start.position.x, start.position.y);   //起点（当前位置）
        Point2D goal_2d(goal.position.x, goal.position.y);  //终点（目标点）
        map2d_manager_->getShortestOptimalPath(goal_2d, start_2d, is_clear_nearby, optimal_path);
        return optimal_path;
    }

    //  找“从当前位置到目标点”的3D绕障路径
    std::vector<GridPoint3D>
    PathExecution::makeLocalPlan3D(geometry_msgs::Pose &goal, geometry_msgs::Pose &start, const bool &is_clear_nearby)
    {
        // 1. 把ROS Pose转成3D点（x/y/z都要）
        GridPoint3D start_3d(start.position.x, start.position.y, start.position.z);
        GridPoint3D goal_3d(goal.position.x, goal.position.y, goal.position.z);
        std::vector<GridPoint3D> optimal_path;  // 存储3D绕障路径
        // 2. 调用3D地图管理器的“找最短可通行路径”接口
        map3d_manager_->getShortestOptimalPath(goal_3d, start_3d, is_clear_nearby, optimal_path);
        return optimal_path;
    }

    //  把3D点列表（path_3d）转成ROS Pose列表（带朝向）
    Path PathExecution::generatePath(std::vector<GridPoint3D> &path_3d)
    {
        std::vector<geometry_msgs::Pose> path;  // 输出的Pose列表

        if (!path_3d.empty())
        {
            // 遍历每个3D点
            for (int i = 0; i < path_3d.size(); i++)
            {
                geometry_msgs::Pose way_pose;   // 单个Pose（位置+朝向）
                 // 1. 填充位置（直接从3D点拿x/y/z）
                way_pose.position.x = path_3d[i].x();
                way_pose.position.y = path_3d[i].y();
                way_pose.position.z = path_3d[i].z();
                // ROS_WARN("path_3d[%d]:(%f,%f,%f)", i, path_3d[i].x(), path_3d[i].y(), path_3d[i].z());

                // 2. 计算朝向（让机器人对着下一个点飞）
                tf::Vector3 axis(0, 0, 1);  // 绕z轴旋转（水平朝向）
                float angle;

                if (path_3d.size() == 1)     // 情况1：只有1个点→对着当前位置飞
                {
                    angle = std::atan2(path_3d[i].y() - current_pose_.position.y,
                                       path_3d[i].x() - current_pose_.position.x);
                }
                else if ((i + 1) < path_3d.size())  // 情况2：中间点→对着下一个点飞
                {
                    angle = std::atan2((path_3d[i + 1].y() - path_3d[i].y()),
                                       (path_3d[i + 1].x() - path_3d[i].x()));
                }
                else    // 情况3：最后一个点→对着上一个点飞
                {
                    angle = std::atan2((path_3d[i].y() - path_3d[i - 1].y()),
                                       (path_3d[i].x() - path_3d[i - 1].x()));
                }
                // 3. 把角度转成四元数（机器人能识别的朝向格式）
                tf::Quaternion quaternion(axis, angle);
                way_pose.orientation.x = quaternion.getX();
                way_pose.orientation.y = quaternion.getY();
                way_pose.orientation.z = quaternion.getZ();
                way_pose.orientation.w = quaternion.getW();

                path.push_back(way_pose);   // 把当前Pose加入列表
            }
        }
        else    // 3D点列表为空→警告
        {
            ROS_WARN("the path_2d is empty, generate path failed"); /* 这行代码 ？？？ 2D—>3D !!! */
        }

        return path;
    }

    // 函数功能：把2D点列表（path_2d）转成ROS Pose列表（z用当前高度）
    Path PathExecution::generatePath(std::vector<Point2D> &path_2d)
    {
        std::vector<geometry_msgs::Pose> path;  // 输出的Pose列表

        // 如果2D点列表不为空
        if (!path_2d.empty())
        {
            // 遍历每个2D点
            for (int i = 0; i < path_2d.size(); i++)
            {
                geometry_msgs::Pose way_pose;   // 单个Pose
                // 1. 填充位置（x/y从2D点拿，z用机器人当前高度）
                way_pose.position.x = path_2d[i].x();
                way_pose.position.y = path_2d[i].y();
                way_pose.position.z = current_pose_.position.z;

                // 2. 计算朝向（逻辑和3D完全一样）
                tf::Vector3 axis(0, 0, 1);
                float angle;
                if (path_2d.size() == 1)
                {
                    angle = std::atan2(path_2d[i].y() - current_pose_.position.y,
                                       path_2d[i].x() - current_pose_.position.x);
                }
                else if ((i + 1) < path_2d.size())
                {
                    angle = std::atan2((path_2d[i + 1].y() - path_2d[i].y()),
                                       (path_2d[i + 1].x() - path_2d[i].x()));
                }
                else
                {
                    angle = std::atan2((path_2d[i].y() - path_2d[i - 1].y()),
                                       (path_2d[i].x() - path_2d[i - 1].x()));
                }
                // 3. 角度转四元数
                tf::Quaternion quaternion(axis, angle);
                way_pose.orientation.x = quaternion.getX();
                way_pose.orientation.y = quaternion.getY();
                way_pose.orientation.z = quaternion.getZ();
                way_pose.orientation.w = quaternion.getW();

                path.push_back(way_pose);   // 加入列表
            }
        }
        else    // 2D点列表为空→警告
        {
            ROS_WARN("the path_2d is empty, generate path failed");
        }

        return path;    // 返回Pose列表
    }

    //  把多条2D路径（path_list）转成RViz能显示的标记数组
    visualization_msgs::MarkerArray PathExecution::generatePath2DListMarker(const vector<Path2D> &path_list) const
    {
        visualization_msgs::MarkerArray path_list_markers;  // 输出的标记数组    
        path_list_markers.markers.resize(path_list.size()); // 标记数量=路径数量（每条路径对应一个标记）
        // 遍历每条路径
        for (int i = 0; i < path_list.size(); ++i)
        {
            visualization_msgs::Marker path_marker; // 单条路径的标记

            // 1. 标记基本信息（坐标系、时间、命名空间）
            path_marker.header.frame_id = global_frame_;    // 和地图同坐标系
            path_marker.header.stamp = ros::Time::now();    // 当前时间
            path_marker.ns = "path";    // 命名空间（区分不同标记）
            path_marker.action = visualization_msgs::Marker::ADD;   // 动作：添加标记
            path_marker.pose.orientation.w = 1.0;   // 默认朝向（无旋转）

            // setting id for each marker
            path_marker.id = i; // 标记ID（唯一，每条路径一个ID）

            // defining types
            path_marker.type = visualization_msgs::Marker::LINE_STRIP;  // 类型：线串（点连起来成线）

            // setting scale
            path_marker.scale.x = 0.05;
            path_marker.scale.y = 0.05;
            path_marker.scale.z = 0.05;

            path_marker.color.r = 1.0f;
            path_marker.color.g = 1.0f;
            path_marker.color.b = 0.0f;
            path_marker.color.a = 1.0f;

            // assignment
            // 3. 填充路径点（每个点的x/y从2D路径拿，z用当前高度）
            geometry_msgs::Point point;
            for (const auto &node : path_list[i])
            {
                point.x = node.x();
                point.y = node.y();
                point.z = current_pose_.position.z;
                path_marker.points.push_back(point);    // 加入标记的点列表
            }

            path_list_markers.markers[i] = path_marker; // 保存当前路径的标记
        }

        return path_list_markers;   // 返回标记数组（RViz显示多条黄色路径）
    }

    // 把单条2D平滑路径（path）转成RViz标记
    visualization_msgs::Marker PathExecution::generateSmoothPath2DMarker(const Path2D &path) const
    {
        visualization_msgs::Marker path_marker; // 平滑路径的标记

        // init headers
        // 1. 基本信息（和多条路径一致）
        path_marker.header.frame_id = global_frame_;
        path_marker.header.stamp = ros::Time::now();
        path_marker.ns = "path";
        path_marker.action = visualization_msgs::Marker::ADD;
        path_marker.pose.orientation.w = 1.0;

        // 2. 标记配置（区别：ID=0，线更宽，颜色紫色）
        // setting id for each marker
        path_marker.id = 0;

        // defining types
        path_marker.type = visualization_msgs::Marker::LINE_STRIP;

        // setting scale
        path_marker.scale.x = 0.1;
        path_marker.scale.y = 0.1;
        path_marker.scale.z = 0.1;

        path_marker.color.r = 1.0f;
        path_marker.color.g = 0.0f;
        path_marker.color.b = 1.0f;
        path_marker.color.a = 1.0f;

        // assignment
        // 3. 填充路径点（和多条路径一致）
        geometry_msgs::Point point;
        for (const auto &node : path)
        {
            point.x = node.x();
            point.y = node.y();
            point.z = current_pose_.position.z;
            path_marker.points.push_back(point);
        }

        return path_marker;  // 返回紫色粗线标记（RViz显示平滑路径）
    }
}
