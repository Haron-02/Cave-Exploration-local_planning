//  “初始化→等目标→生成轨迹→执行轨迹→异常重规划” 
#include <plan_manage/topo_replan_fsm.h>

namespace fast_planner {

//  初始化函数（init）——“搭建规划的基础框架”
void TopoReplanFSM::init(ros::NodeHandle& nh) {
  current_wp_ = 0;
  exec_state_ = FSM_EXEC_STATE::INIT;
  have_target_ = false;
  collide_ = false;

  /*  fsm param  */
  nh.param("fsm/flight_type", target_type_, -1);
  nh.param("fsm/thresh_replan", replan_time_threshold_, -1.0);
  nh.param("fsm/thresh_no_replan", replan_distance_threshold_, -1.0);
  nh.param("fsm/waypoint_num", waypoint_num_, -1);
  nh.param("fsm/act_map", act_map_, false);
  for (int i = 0; i < waypoint_num_; i++) {
    nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
  }

  /* initialize main modules */
  planner_manager_.reset(new FastPlannerManager);
  planner_manager_->initPlanModules(nh);
  visualization_.reset(new PlanningVisualization(nh));

  /* callback */
  exec_timer_ = nh.createTimer(ros::Duration(0.01), &TopoReplanFSM::execFSMCallback, this);
  safety_timer_ = nh.createTimer(ros::Duration(0.05), &TopoReplanFSM::checkCollisionCallback, this);
  // frontier_timer_ = nh.createTimer(ros::Duration(0.1), &TopoReplanFSM::frontierCallback, this);

  // 订阅“目标点话题”（比如从键盘/APP发送的目标点，话题名：/waypoint_generator/waypoints）
  waypoint_sub_ =
      nh.subscribe("/waypoint_generator/waypoints", 1, &TopoReplanFSM::waypointCallback, this);
  // 订阅“机器人状态话题”（odom：位置、速度、朝向，话题名：/odom_world）
  odom_sub_ = nh.subscribe("/odom_world", 1, &TopoReplanFSM::odometryCallback, this);

  // 创建发布者（给其他模块发信号）
  replan_pub_ = nh.advertise<std_msgs::Empty>("/planning/replan", 20);
  new_pub_ = nh.advertise<std_msgs::Empty>("/planning/new", 20);
  bspline_pub_ = nh.advertise<bspline::Bspline>("/planning/bspline", 20);
}

//  接收目标点回调函数
void TopoReplanFSM::waypointCallback(const nav_msgs::PathConstPtr& msg) {
  if (msg->poses[0].pose.position.z < -0.1) return;
  cout << "Triggered!" << endl;

  vector<Eigen::Vector3d> global_wp;  // 存“全局航点”（给规划器的路线清单）
  // 按“目标类型”处理目标点
  if (target_type_ == TARGET_TYPE::REFENCE_PATH) {  
    for (int i = 0; i < waypoint_num_; ++i) { // 把预设的waypoint_num_个航点加入全局航点
      Eigen::Vector3d pt; // 存单个航点的x/y/z
      pt(0) = waypoints_[i][0];
      pt(1) = waypoints_[i][1];
      pt(2) = waypoints_[i][2];
      global_wp.push_back(pt);
    }
  } else {
    if (target_type_ == TARGET_TYPE::MANUAL_TARGET) {
      target_point_(0) = msg->poses[0].pose.position.x;
      target_point_(1) = msg->poses[0].pose.position.y;
      target_point_(2) = 1.0;
      std::cout << "manual: " << target_point_.transpose() << std::endl;
    } else if (target_type_ == TARGET_TYPE::PRESET_TARGET) {
      target_point_(0) = waypoints_[current_wp_][0];
      target_point_(1) = waypoints_[current_wp_][1];
      target_point_(2) = waypoints_[current_wp_][2];

      current_wp_ = (current_wp_ + 1) % waypoint_num_;
      std::cout << "preset: " << target_point_.transpose() << std::endl;
    }

    global_wp.push_back(target_point_); // 把单个目标点加入全局航点
    // 在RViz里画目标点：红色，半径0.3米（让开发者看到目标在哪）
    visualization_->drawGoal(target_point_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
  }

  // 把全局航点传给下层规划器（告诉它“要走的路线”）
  planner_manager_->setGlobalWaypoints(global_wp);
  end_vel_.setZero();
  have_target_ = true;
  trigger_ = true;

  // 若当前状态是“等目标”（WAIT_TARGET），切换到“生成新轨迹”（GEN_NEW_TRAJ）
  if (exec_state_ == WAIT_TARGET) {
    changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
  }
}

//  接收机器人状态回调函数
void TopoReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  // 1. 存机器人当前位置（x/y/z）
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;
  odom_pos_(2) = msg->pose.pose.position.z;

  // 2. 存机器人当前速度（x/y/z方向的线速度）
  odom_vel_(0) = msg->twist.twist.linear.x;
  odom_vel_(1) = msg->twist.twist.linear.y;
  odom_vel_(2) = msg->twist.twist.linear.z;

  // 3. 存机器人当前朝向（四元数格式，后续转成角度）
  odom_orient_.w() = msg->pose.pose.orientation.w;
  odom_orient_.x() = msg->pose.pose.orientation.x;
  odom_orient_.y() = msg->pose.pose.orientation.y;
  odom_orient_.z() = msg->pose.pose.orientation.z;

  have_odom_ = true;  // 标记“有odom数据了”（规划的前提）
}

void TopoReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call) {
  string state_str[6] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "REPLAN_"
                                                                                             "NEW" };
  int pre_s = int(exec_state_);
  exec_state_ = new_state;
  cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
}

//  打印当前状态（printFSMExecState）
void TopoReplanFSM::printFSMExecState() {
  string state_str[6] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "REPLAN_"
                                                                                             "NEW" };
  cout << "state: " + state_str[int(exec_state_)] << endl;
}

void TopoReplanFSM::execFSMCallback(const ros::TimerEvent& e) {
  static int fsm_num = 0;
  fsm_num++;
  if (fsm_num == 100) { // 每100次循环（0.01*100=1秒）打印一次当前状态和关键信息（监控用）
    printFSMExecState();
    if (!have_odom_) cout << "no odom." << endl;
    if (!trigger_) cout << "no trigger_." << endl;
    fsm_num = 0;
  }

  switch (exec_state_) {
    case INIT: {
      if (!have_odom_) return;
      if (!trigger_) return;
      changeFSMExecState(WAIT_TARGET, "FSM");

      break;
    }

    case WAIT_TARGET: {
      if (!have_target_)
        return;
      else
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");

      break;
    }

    case GEN_NEW_TRAJ: {
      start_pt_ = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();

      Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      start_yaw_(0) = atan2(rot_x(1), rot_x(0));
      start_yaw_(1) = start_yaw_(2) = 0.0;

      new_pub_.publish(std_msgs::Empty());
      /* topo path finding and optimization */
      bool success = callTopologicalTraj(1);
      if (success)
        changeFSMExecState(EXEC_TRAJ, "FSM");
      else {
        ROS_WARN("Replan: retrying============================================");
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case EXEC_TRAJ: {
      /* determine if need to replan */

      GlobalTrajData* global_data = &planner_manager_->global_data_;
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - global_data->global_start_time_).toSec();

      if (t_cur > global_data->global_duration_ - 1e-2) {
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "FSM");
      } else {
        LocalTrajData* info = &planner_manager_->local_data_;
        t_cur = (time_now - info->start_time_).toSec();

        if (t_cur > replan_time_threshold_) {
          if (!global_data->localTrajReachTarget()) {
            ROS_WARN("Replan: periodic call=======================================");
            changeFSMExecState(REPLAN_TRAJ, "FSM");
          } else {
            Eigen::Vector3d cur_pos = info->position_traj_.evaluateDeBoorT(t_cur);
            Eigen::Vector3d end_pos = info->position_traj_.evaluateDeBoorT(info->duration_);
            if ((cur_pos - end_pos).norm() > replan_distance_threshold_) {
              ROS_WARN("Replan: periodic call=======================================");
              changeFSMExecState(REPLAN_TRAJ, "FSM");
            }
          }
        }
      }
      break;
    }

    case REPLAN_TRAJ: {
      LocalTrajData* info = &planner_manager_->local_data_;
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - info->start_time_).toSec();

      start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
      start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
      start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

      start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_cur)[0];
      start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_cur)[0];
      start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_cur)[0];

      bool success = callTopologicalTraj(2);
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        ROS_WARN("Replan fail, retrying...");
      }

      break;
    }
    case REPLAN_NEW: {
      LocalTrajData* info = &planner_manager_->local_data_;
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - info->start_time_).toSec();

      start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
      start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
      start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

      /* inform server */
      new_pub_.publish(std_msgs::Empty());

      // bool success = callSearchAndOptimization();
      bool success = callTopologicalTraj(1);
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

      break;
    }
  }
}

//  安全检查函数（checkCollisionCallback）——“避免撞墙”
void TopoReplanFSM::checkCollisionCallback(const ros::TimerEvent& e) {
  LocalTrajData* info = &planner_manager_->local_data_;

  /* ---------- check goal safety ---------- */
  // if (have_target_)
  if (false) {
    auto edt_env = planner_manager_->edt_environment_;

    double dist = planner_manager_->pp_.dynamic_ ?
        edt_env->evaluateCoarseEDT(target_point_, /* time to program start */ info->duration_) :
        edt_env->evaluateCoarseEDT(target_point_, -1.0);

    if (dist <= 0.3) {
      /* try to find a max distance goal around */
      bool new_goal = false;
      const double dr = 0.5, dtheta = 30, dz = 0.3;

      double new_x, new_y, new_z, max_dist = -1.0;
      Eigen::Vector3d goal;

      for (double r = dr; r <= 5 * dr + 1e-3; r += dr) {
        for (double theta = -90; theta <= 270; theta += dtheta) {
          for (double nz = 1 * dz; nz >= -1 * dz; nz -= dz) {
            new_x = target_point_(0) + r * cos(theta / 57.3);
            new_y = target_point_(1) + r * sin(theta / 57.3);
            new_z = target_point_(2) + nz;
            Eigen::Vector3d new_pt(new_x, new_y, new_z);

            dist = planner_manager_->pp_.dynamic_ ?
                edt_env->evaluateCoarseEDT(new_pt,
                                           /* time to program start */ info->duration_) :
                edt_env->evaluateCoarseEDT(new_pt, -1.0);

            if (dist > max_dist) {
              /* reset target_point_ */
              goal(0) = new_x;
              goal(1) = new_y;
              goal(2) = new_z;
              max_dist = dist;
            }
          }
        }
      }

      // 若找到安全点（离障碍>0.3米），更新目标点，触发重规划
      if (max_dist > 0.3) {
        cout << "change goal, replan." << endl;
        target_point_ = goal;
        have_target_ = true;
        end_vel_.setZero();

        if (exec_state_ == EXEC_TRAJ) {
          changeFSMExecState(REPLAN_NEW, "SAFETY");
        }

        // 在RViz里画新目标点（红色）
        visualization_->drawGoal(target_point_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
      } else {
        // have_target_ = false;
        // cout << "Goal near collision, stop." << endl;
        // changeFSMExecState(WAIT_TARGET, "SAFETY");
        cout << "goal near collision, keep retry" << endl;
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
    }
  }

  /* ---------- check trajectory ---------- */
  // 第二步：当前轨迹安全检查（核心，必须启用） 
  if (exec_state_ == EXEC_TRAJ || exec_state_ == REPLAN_TRAJ) { //执行/重规划轨迹时才检查
    double dist;
    bool safe = planner_manager_->checkTrajCollision(dist);
    if (!safe) {
      if (dist > 0.2) {
        ROS_WARN("current traj %lf m to collision", dist);
        ROS_WARN("Replan: collision detected==================================");
        collide_ = true;
        changeFSMExecState(REPLAN_TRAJ, "SAFETY");
      } else {
        ROS_ERROR("current traj %lf m to collision", dist);
        replan_pub_.publish(std_msgs::Empty());
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "SAFETY");
      }
    } else {
      collide_ = false;
    }
  }
}

//  边界检测回调（frontierCallback）——“找未知区域边界（用于探索）”
void TopoReplanFSM::frontierCallback(const ros::TimerEvent& e) {
  if (!have_odom_) return;
  planner_manager_->searchFrontier(odom_pos_);
  visualization_->drawFrontier(planner_manager_->plan_data_.frontiers_);
}

bool TopoReplanFSM::callSearchAndOptimization() {
}

bool TopoReplanFSM::callTopologicalTraj(int step) { // step=1：生成全局轨迹，step=2：重规划局部轨迹
  bool plan_success;

  if (step == 1) plan_success = planner_manager_->planGlobalTraj(start_pt_);

  replan_time_.push_back(0.0);
  auto t1 = ros::Time::now();
  plan_success = planner_manager_->topoReplan(collide_);
  replan_time_[replan_time_.size() - 1] += (ros::Time::now() - t1).toSec();

  // 3. 若轨迹生成成功，继续处理偏航角、发布轨迹、可视化
  if (plan_success) {
    if (!act_map_) {  // 普通导航：朝向跟运动方向一致
      planner_manager_->planYaw(start_yaw_);
    } else {  // 主动建图：朝向要方便传感器扫未知区域
      replan_time2_.push_back(0); // 记录建图时的偏航规划时间
      auto t1 = ros::Time::now();
      planner_manager_->planYawActMap(start_yaw_);
      replan_time2_[replan_time2_.size() - 1] += (ros::Time::now() - t1).toSec();
    }

    // 3.2 获取局部轨迹数据（要发布给底盘的轨迹）
    LocalTrajData* locdat = &planner_manager_->local_data_;

    /* publish newest trajectory to server */

    /* publish traj */
    bspline::Bspline bspline; // 自定义的B样条消息格式
    bspline.order = planner_manager_->pp_.bspline_degree_;  // B样条阶数（比如3阶）
    bspline.start_time = locdat->start_time_; // 轨迹开始时间
    bspline.traj_id = locdat->traj_id_; // 轨迹ID（区分不同轨迹）

    // 加“时间节点”（底盘知道每个控制点对应的时间）
    Eigen::MatrixXd pos_pts = locdat->position_traj_.getControlPoint(); // 加“位置控制点”（底盘需要这些点来生成平滑运动）

    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }

    // 加“偏航角控制点”（控制机器人朝向）
    Eigen::VectorXd knots = locdat->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }

    Eigen::MatrixXd yaw_pts = locdat->yaw_traj_.getControlPoint();  // 偏航角轨迹的时间步长
    for (int i = 0; i < yaw_pts.rows(); ++i) {
      double yaw = yaw_pts(i, 0);
      bspline.yaw_pts.push_back(yaw);
    }
    bspline.yaw_dt = locdat->yaw_traj_.getKnotSpan();

    bspline_pub_.publish(bspline);  // 发布轨迹（底盘订阅这个话题，按轨迹运动）

    /* visualize new trajectories */

    MidPlanData* plan_data = &planner_manager_->plan_data_; // 规划过程数据

    visualization_->drawPolynomialTraj(planner_manager_->global_data_.global_traj_, 0.05,
                                       Eigen::Vector4d(0, 0, 0, 1), 0);
    visualization_->drawBspline(locdat->position_traj_, 0.08, Eigen::Vector4d(1.0, 0.0, 0.0, 1), true,
                                0.15, Eigen::Vector4d(1, 1, 0, 1), 99);
    visualization_->drawBsplinesPhase2(plan_data->topo_traj_pos1_, 0.08);

    // visualization_->drawBspline(locdat->position_traj_, 0.08, Eigen::Vector4d(1.0, 0.0, 0.0, 1),
    // false, 0.15,
    //                             Eigen::Vector4d(1.0, 1.0, 1.0, 1), 99, 99);
    // visualization_->drawBspline(plan_data->no_visib_traj_, 0.08, Eigen::Vector4d(0.0,
    // 1.0, 1.0, 0.8),
    //                             true, 0.15, Eigen::Vector4d(1.0, 1.0, 1.0, 1), 98, 98);

    // visualization_->drawTopoPathsPhase2(plan_data->topo_select_paths_, 0.05);

    visualization_->drawViewConstraint(plan_data->view_cons_);
    visualization_->drawYawTraj(locdat->position_traj_, locdat->yaw_traj_, plan_data->dt_yaw_);
    // visualization_->drawYawPath(locdat->position_traj_, plan_data->path_yaw_,
    // plan_data->dt_yaw_path_);

    // visualization_->drawBspline(plan_data->no_visib_traj_, 0.08,
    //                             Eigen::Vector4d(0.0, 0.0, 1.0, 0.8), false,
    //                             0.1, Eigen::Vector4d(), 98);

    // visualization_->drawTopoGraph(planner_manager_->topo_graphs_,
    // 0.15, 0.05, Eigen::Vector4d(1, 0, 0, 1),
    //                               Eigen::Vector4d(0, 1, 0, 1),
    //                               Eigen::Vector4d(0, 0, 1, 1));

    // visualization_->drawTopoPathsPhase1(plan_data->topo_paths_, 0.05);
    // visualization_->drawBsplinesPhase1(plan_data->topo_traj_pos1_,
    // 0.065);

    // // benchmark, calculate replan time min max mean
    // double mean1 = 0.0;
    // double mean2 = 0.0;
    // for (auto t : replan_time_)
    //   mean1 += t;
    // for (auto t : replan_time2_)
    //   mean2 += t;
    // mean1 /= replan_time_.size();
    // mean2 /= replan_time2_.size();
    // ROS_WARN("Replan number: %d, mean traj: %lf, mean yaw: %lf", replan_time_.size(), mean1, mean2);

    return true;
  } else {
    return false;
  }
}
// TopoReplanFSM::
}  // namespace fast_planner
