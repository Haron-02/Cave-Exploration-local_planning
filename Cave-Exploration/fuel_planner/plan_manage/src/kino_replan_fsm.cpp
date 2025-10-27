//  基于动力学约束的轨迹重规划有限状态机
#include <plan_manage/kino_replan_fsm.h>

namespace fast_planner
{
  void KinoReplanFSM::init(ros::NodeHandle &nh)
  {
    current_wp_ = 0;  // 当前航点索引
    exec_state_ = FSM_EXEC_STATE::INIT; // 初始状态为"初始化"
    have_target_ = false; // 标记是否有目标点
    have_odom_ = false; // 标记是否有里程计数据

    /*  fsm param  */
    nh.param("fsm/flight_type", target_type_, -1);  // 目标类型（手动/预设）
    nh.param("fsm/thresh_replan", replan_thresh_, -1.0);  // 重规划阈值（距离起点小于此值不重规划）
    nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);  // 不重规划阈值（距离终点小于此值不重规划）

    nh.param("fsm/waypoint_num", waypoint_num_, -1);  // 预设航点数量
    // 读取预设航点坐标
    for (int i = 0; i < waypoint_num_; i++)
    {
      nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
    }

    /* initialize main modules */
    // 初始化核心模块
    planner_manager_.reset(new FastPlannerManager); // 规划器管理器（负责实际轨迹生成）
    planner_manager_->initPlanModules(nh);  // 初始化规划器模块
    visualization_.reset(new PlanningVisualization(nh));  // 可视化工具（显示轨迹、目标等）

    /* callback */
    // 初始化定时器和订阅器
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &KinoReplanFSM::execFSMCallback, this);  // 状态机执行定时器（100Hz）
    safety_timer_ = nh.createTimer(ros::Duration(0.05), &KinoReplanFSM::checkCollisionCallback, this);  // 碰撞检查定时器（20Hz）

    waypoint_sub_ =
        nh.subscribe("/waypoint_generator/waypoints", 1, &KinoReplanFSM::waypointCallback, this); // 订阅航点消息
    odom_sub_ = nh.subscribe("/odom_world", 1, &KinoReplanFSM::odometryCallback, this); // 订阅里程计消息

    replan_pub_ = nh.advertise<std_msgs::Empty>("/planning/replan", 10);  // 发布重规划信号
    new_pub_ = nh.advertise<std_msgs::Empty>("/planning/new", 10);  // 发布新轨迹信号
    bspline_pub_ = nh.advertise<bspline::Bspline>("/planning/bspline", 10); // 发布B样条轨迹
  }

  //  接收外部航点消息（目标点），更新目标状态并触发状态转换：
  void KinoReplanFSM::waypointCallback(const nav_msgs::PathConstPtr &msg)
  {
    if (msg->poses[0].pose.position.z < -0.1)   // 过滤无效目标（z过小）
      return;
    cout << "Triggered!" << endl;
    trigger_ = true;  // 标记触发规划

    // 根据目标类型设置终点（手动目标/预设航点循环）
    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
    {
      end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, 1.0;
    }
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      end_pt_(0) = waypoints_[current_wp_][0];
      end_pt_(1) = waypoints_[current_wp_][1];
      end_pt_(2) = waypoints_[current_wp_][2];
      current_wp_ = (current_wp_ + 1) % waypoint_num_;  // 循环切换预设航点
    }

    visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));  // 可视化目标点
    end_vel_.setZero(); // 终点速度设为0
    have_target_ = true;  // 标记有目标点

    // 根据当前状态触发状态转换（等待目标→生成新轨迹；执行轨迹→重规划轨迹）
    if (exec_state_ == WAIT_TARGET)
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
    else if (exec_state_ == EXEC_TRAJ)
      changeFSMExecState(REPLAN_TRAJ, "TRIG");
  }

  //里程计回调：接收机器人实时位姿（位置、速度、姿态），用于规划初始状态：
  void KinoReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    // 更新位置、速度
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    // 更新姿态（四元数）
    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;  // 标记有里程计数据
  }

  void KinoReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {
    string state_str[5] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_"
                                                                                 "TRAJ"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  void KinoReplanFSM::printFSMExecState()
  {
    string state_str[5] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_"
                                                                                 "TRAJ"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  //  状态机核心（execFSMCallback） - 根据当前状态执行相应操作并进行状态转换：
  void KinoReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!trigger_)
        cout << "wait for goal." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        return;
      }
      if (!trigger_)
      {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_)
        return;
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ:  // 生成新轨迹：基于当前位姿和目标点规划轨迹
    {
      // 设置规划起点（当前位姿、速度、加速度）
      start_pt_ = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();

      // 计算初始偏航角（从姿态四元数转换）
      Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      start_yaw_(0) = atan2(rot_x(1), rot_x(0));
      start_yaw_(1) = start_yaw_(2) = 0.0;

      // 调用动力学重规划
      bool success = callKinodynamicReplan();
      if (success)
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        // have_target_ = false;
        // changeFSMExecState(WAIT_TARGET, "FSM");
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case EXEC_TRAJ: // 执行轨迹：判断是否需要重规划
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - info->start_time_).toSec();  // 当前轨迹执行时间
      t_cur = min(info->duration_, t_cur);  // 限制在轨迹时长内

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);  // 当前轨迹上的位置

      /* && (end_pt_ - pos).norm() < 0.5 */
      // 若轨迹执行完毕（接近终点），切换到等待目标
      if (t_cur > info->duration_ - 1e-2)
      {
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "FSM");
        return;
      }
      // 若接近终点（小于不重规划阈值），不重规划
      else if ((end_pt_ - pos).norm() < no_replan_thresh_)
      {
        // cout << "near end" << endl;
        return;
      }
      // 若接近起点（小于重规划阈值），不重规划
      else if ((info->start_pos_ - pos).norm() < replan_thresh_)
      {
        // cout << "near start" << endl;
        return;
      }
      // 否则触发重规划
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ: // 重规划轨迹：基于当前轨迹状态更新轨迹
    {
      // 获取当前轨迹上的状态（位置、速度、加速度、偏航角）
      LocalTrajData *info = &planner_manager_->local_data_;
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - info->start_time_).toSec();

      start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
      start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
      start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

      start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_cur)[0];
      start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_cur)[0];
      start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_cur)[0];

      // 发布重规划信号
      std_msgs::Empty replan_msg;
      replan_pub_.publish(replan_msg);

      // 调用重规划
      bool success = callKinodynamicReplan();
      if (success)
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }
    }
  }

  //  安全检查 - 定时检查当前轨迹和目标点的碰撞状态，必要时触发重规划：
  void KinoReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
  {
    LocalTrajData *info = &planner_manager_->local_data_;

    if (have_target_)
    {
      auto edt_env = planner_manager_->edt_environment_;

      double dist = planner_manager_->pp_.dynamic_ ? edt_env->evaluateCoarseEDT(end_pt_, /* time to program start + */ info->duration_) : edt_env->evaluateCoarseEDT(end_pt_, -1.0);

      if (dist <= 0.3)  // 目标点不安全（距离<0.3m）
      {
        /* try to find a max distance goal around */
        bool new_goal = false;
        const double dr = 0.5, dtheta = 30, dz = 0.3; // 搜索步长
        double new_x, new_y, new_z, max_dist = -1.0;
        Eigen::Vector3d goal;

        // 环形搜索：半径、角度、高度方向
        for (double r = dr; r <= 5 * dr + 1e-3; r += dr)
        {
          for (double theta = -90; theta <= 270; theta += dtheta)
          {
            for (double nz = 1 * dz; nz >= -1 * dz; nz -= dz)
            {
              new_x = end_pt_(0) + r * cos(theta / 57.3); // 角度转弧度
              new_y = end_pt_(1) + r * sin(theta / 57.3);
              new_z = end_pt_(2) + nz;

              Eigen::Vector3d new_pt(new_x, new_y, new_z);
              dist = planner_manager_->pp_.dynamic_ ? edt_env->evaluateCoarseEDT(new_pt,
                                                                                 /* time to program start+ */ info->duration_)
                                                    : edt_env->evaluateCoarseEDT(new_pt, -1.0);

              if (dist > max_dist)
              {
                /* reset end_pt_ */
                goal(0) = new_x;
                goal(1) = new_y;
                goal(2) = new_z;
                max_dist = dist;
              }
            }
          }
        }

        if (max_dist > 0.3)
        {
          cout << "change goal, replan." << endl;
          end_pt_ = goal;
          have_target_ = true;
          end_vel_.setZero();

          if (exec_state_ == EXEC_TRAJ)
          {
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }

          visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
        }
        else
        {
          // have_target_ = false;
          // cout << "Goal near collision, stop." << endl;
          // changeFSMExecState(WAIT_TARGET, "SAFETY");
          cout << "goal near collision, keep retry" << endl;
          changeFSMExecState(REPLAN_TRAJ, "FSM");

          std_msgs::Empty emt;
          replan_pub_.publish(emt);
        }
      }
    }

    /* ---------- check trajectory ---------- */
    if (exec_state_ == FSM_EXEC_STATE::EXEC_TRAJ)
    {
      double dist;
      bool safe = planner_manager_->checkTrajCollision(dist);

      if (!safe)
      {
        // cout << "current traj in collision." << endl;
        ROS_WARN("current traj in collision.");
        changeFSMExecState(REPLAN_TRAJ, "SAFETY");
      }
    }
  }

  //  轨迹规划与发布
  bool KinoReplanFSM::callKinodynamicReplan()
  {
    auto t1 = ros::Time::now();
    ros::Time time_r = ros::Time::now();
    bool plan_success =
        planner_manager_->kinodynamicReplan(start_pt_, start_vel_, start_acc_, end_pt_, end_vel_);

    auto t2 = ros::Time::now();
    if (plan_success)
    {
      replan_time_.push_back((t2 - t1).toSec());
      double mean1 = 0.0;
      for (auto t : replan_time_)
        mean1 += t;
      mean1 /= replan_time_.size();
      ROS_WARN("Replan number: %ld, mean traj: %lf", replan_time_.size(), mean1);
    }

    if (plan_success)
    {
      planner_manager_->planYaw(start_yaw_);

      // 发布B样条轨迹（供控制器跟踪）
      auto info = &planner_manager_->local_data_;
      info->start_time_ = time_r;

      /* publish traj */
      bspline::Bspline bspline;
      bspline.order = planner_manager_->pp_.bspline_degree_;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();

      // 填充轨迹控制点和 knots
      for (int i = 0; i < pos_pts.rows(); ++i)
      {
        geometry_msgs::Point pt;
        pt.x = pos_pts(i, 0);
        pt.y = pos_pts(i, 1);
        pt.z = pos_pts(i, 2);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      // 填充偏航角轨迹
      Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
      for (int i = 0; i < yaw_pts.rows(); ++i)
      {
        double yaw = yaw_pts(i, 0);
        bspline.yaw_pts.push_back(yaw);
      }
      bspline.yaw_dt = info->yaw_traj_.getKnotSpan();

      bspline_pub_.publish(bspline);

      /* visulization */
      // 可视化轨迹（几何路径、B样条曲线）
      auto plan_data = &planner_manager_->plan_data_;

      visualization_->drawGeometricPath(plan_data->kino_path_, 0.05, Eigen::Vector4d(1, 0, 1, 1));
      visualization_->drawBspline(info->position_traj_, 0.08, Eigen::Vector4d(1.0, 1.0, 0.0, 1), true,
                                  0.15, Eigen::Vector4d(1, 0, 0, 1));

      return true;
    }
    else  // 规划失败
    {
      cout << "generate new traj fail." << endl;
      return false;
    }
  }

  // KinoReplanFSM::
} // namespace fast_planner
