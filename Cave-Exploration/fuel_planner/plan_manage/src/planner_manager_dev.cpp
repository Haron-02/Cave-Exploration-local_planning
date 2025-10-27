//  主动建图和局部探索规划（未知环境中找中间目标并生成安全轨迹）
//  主动建图时的偏航角规划（控制机器人朝向，让传感器扫到更多环境）；
//  未知环境中的局部探索规划（找 “中间目标”，生成安全平滑的运动轨迹）。
#include <plan_manage/planner_manager.h>
#include <plan_env/sdf_map.h>
#include <future>
#include <pcl_conversions/pcl_conversions.h>

namespace fast_planner
{
  //  偏航角规划
  void FastPlannerManager::planYawActMap(const Eigen::Vector3d &start_yaw)  // 输入：start_yaw（初始朝向，包含角度、角速度、角加速度）
  {
    ROS_INFO("Plan yaw active mapping-----------------");
    auto t1 = ros::Time::now(); // 记录规划开始时间（最后算耗时，看规划快不快）
    auto t2 = ros::Time::now(); // 临时时间变量（中间算小步骤耗时）

    double t_traj = local_data_.duration_;
    // search subsequent yaws
    const int seg_num = 12; // 把总时长分成12段（seg_num=12），每段时间dt_yaw；每隔2段采1个点（subsp=2），采样间隔dt_path
    double dt_yaw = local_data_.duration_ / seg_num; // time of B-spline segment
    const int subsp = 2;                             // subsampling factor to create yaw path
    double dt_path = dt_yaw * subsp;                 // time of yaw path segment
    std::cout << "duration: " << local_data_.duration_ << ", seg_num: " << seg_num
              << ", dt_yaw: " << dt_yaw << ", dt_path: " << dt_path << std::endl;

    Eigen::Vector3d start_yaw3d = start_yaw;  // 复制初始朝向（避免修改原始数据）
    while (start_yaw3d[0] < -M_PI)  // 若角度小于-180°，加360°
      start_yaw3d[0] += 2 * M_PI;
    while (start_yaw3d[0] > M_PI) // 若角度大于180°，减360°
      start_yaw3d[0] -= 2 * M_PI;
    double last_yaw = start_yaw3d[0]; // 记录上一个朝向（用于后续让朝向变化更平滑）

    const double forward_t = 4.0 / pp_.max_vel_;  // 前瞻时间：比如最大速度5m/s，就看0.8秒后的位置（确定当前该朝哪）
    vector<Eigen::Vector3d> pts; // subsampled waypoints
    vector<double> spyaw;        // subsampled yaws
    bool unmapped = false;  // 标记是否有未映射区域（暂时没用到，预留）
    vector<Eigen::Vector3d> waypts;
    vector<int> waypt_idx;

    for (int idx = 0; idx <= seg_num; ++idx)  // idx从0到12（12段B样条，共13个点）
    {
      if (idx % subsp != 0) // 每隔2段采1个点（idx=0,2,4...12），跳过中间的点
        continue;
      // calc the yaw angle of pc->pf
      double tc = idx * dt_yaw; // 当前段的时间（比如idx=2，dt_yaw=0.167，tc=0.334秒）
      double tf = min(local_data_.duration_, tc + forward_t); // 前瞻时间（不超过总时长，避免越界）
      Eigen::Vector3d pc = local_data_.position_traj_.evaluateDeBoorT(tc);  // 从位置轨迹中，查tc时刻的位置（比如0.334秒时在哪）
      Eigen::Vector3d pd = local_data_.position_traj_.evaluateDeBoorT(tf) - pc; // 前瞻位置 - 当前位置 = 运动方向向量（比如往东北走）

      double yc;  // 当前采样点的目标朝向
      if (pd.norm() > 1e-6) // 如果运动方向向量不为0（机器人在动，不是静止）
      {
        // deal with +-180 degree
        yc = atan2(pd(1), pd(0)); // 用反正切算朝向（比如pd是(1,1)，yc就是45°，朝东北）
        calcNextYaw(last_yaw, yc);  // 调整朝向：让当前朝向和上一个朝向变化平滑
      }
      else
      {
        yc = spyaw.back();  // spyaw.back()是上一个采样点的朝向
      }
      last_yaw = yc;  // 更新“上一个朝向”
      // add points in already mapped space
      pts.push_back(pc);  // 存当前位置
      spyaw.push_back(yc);  // 存当前朝向
      waypt_idx.push_back(idx); // 存当前索引
      waypts.emplace_back(yc, 0, 0);  // 存朝向控制点（后两个0是占位，只关注朝向角度）
    }
    spyaw[0] = start_yaw3d[0];  // 强制第一个采样点的朝向=初始朝向（避免计算误差，保证开头正确）
    // std::cout << "discretize time: " << (ros::Time::now() - t2).toSec() <<
    // std::endl;

    // 5. 找粗略的朝向路径（用朝向规划器算一条大概的朝向路线）
    t2 = ros::Time::now();
    vector<double> path;
    // 调用heading_planner_（朝向规划器），输入采样位置、采样朝向、时间间隔、位置轨迹控制点，输出粗略朝向路径
    heading_planner_->searchPathOfYaw(pts, spyaw, dt_path, local_data_.position_traj_.getControlPoint(),
                                      path);  // 把粗略路径的值赋给waypts（作为后续优化的“初始参考”）
    for (int i = 0; i < path.size(); ++i)
    {
      waypts[i][0] = path[i];
    }

    // yaw traj optimization
    // 6. 优化朝向轨迹（用B样条优化，让轨迹更平滑、符合机器人能力）
    Eigen::MatrixXd yaw(seg_num + 3, 1);  // B样条3阶需要“段数+3”个控制点（12段→15个控制点），存优化后的朝向
    yaw.setZero();  // 先初始化为0（后续逐步更新）
    // boundary state
    Eigen::Matrix3d states2pts;  // 转换矩阵：把“状态（角度、角速度、角加速度）” 转成B样条的前3个控制点
    states2pts << 1.0, -dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw, 1.0, 0.0, -(1 / 6.0) * dt_yaw * dt_yaw, 1.0,
        dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw;   // 把“初始朝向状态”转成B样条的前3个控制点（保证轨迹从初始状态开始）
    yaw.block(0, 0, 3, 1) = states2pts * start_yaw3d;

    // 6.2 算“轨迹结尾的朝向”（要和结尾的运动方向一致）
    Eigen::Vector3d end_v = local_data_.velocity_traj_.evaluateDeBoorT(local_data_.duration_ - 0.1);  // 算轨迹快结束时的速度（比如总时长2秒，算1.9秒时的速度）
    Eigen::Vector3d end_yaw(atan2(end_v(1), end_v(0)), 0, 0); // 用速度算结尾朝向（朝运动方向）
    calcNextYaw(last_yaw, end_yaw(0));  // 调整结尾朝向，保证平滑变化
    yaw.block(seg_num, 0, 3, 1) = states2pts * end_yaw; // 把“结尾朝向状态”转成B样条的最后3个控制点（保证轨迹结尾符合预期）

    // call B-spline optimization solver
    bspline_optimizers_[1]->setWaypoints(waypts, waypt_idx);  // 设置“参考控制点”和索引（优化时要靠近这些点）
    vector<Eigen::Vector3d> start = {Eigen::Vector3d(start_yaw3d[0], 0, 0),
                                     Eigen::Vector3d(start_yaw3d[1], 0, 0),
                                     Eigen::Vector3d(start_yaw3d[2], 0, 0)};
    vector<Eigen::Vector3d> end = {Eigen::Vector3d(end_yaw[0], 0, 0), Eigen::Vector3d(0, 0, 0)};
    bspline_optimizers_[1]->setBoundaryStates(start, end);
    // 定义“优化目标”：1. 轨迹平滑（SMOOTHNESS）；2. 靠近参考控制点（WAYPOINTS）；3. 符合开头状态（START）；4. 符合结尾状态（END）
    int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::WAYPOINTS | BsplineOptimizer::START |
                    BsplineOptimizer::END;
    bspline_optimizers_[1]->optimize(yaw, dt_yaw, cost_func, 1, 1); // 执行优化：输入初始控制点、时间间隔、优化目标，输出最优控制点

    // update traj info
    // 7. 更新规划结果（存到local_data_，机器人后续用）
    local_data_.yaw_traj_.setUniformBspline(yaw, 3, dt_yaw);  // 把优化后的控制点转成“偏航角轨迹”（B样条3阶）
    local_data_.yawdot_traj_ = local_data_.yaw_traj_.getDerivative(); // 算“角速度轨迹”（朝向轨迹的1阶导数）
    local_data_.yawdotdot_traj_ = local_data_.yawdot_traj_.getDerivative(); // 算“角加速度轨迹”（角速度轨迹的1阶导数）


    plan_data_.path_yaw_ = path;  // 粗略的朝向路径
    plan_data_.dt_yaw_ = dt_yaw;  // 每段B样条的时间
    plan_data_.dt_yaw_path_ = dt_yaw * subsp; // 采样点时间间隔


    std::cout << "plan heading: " << (ros::Time::now() - t1).toSec() << std::endl;
    // // debug waypt error:
    // {
    //   double duration = local_data_.yaw_traj_.getTimeSum();
    //   for (int i = 0; i < path.size(); ++i) {
    //     double yt1 = local_data_.yaw_traj_.evaluateDeBoorT(i * dt_yaw * subsp)[0];
    //     double yt2 = path[i];
    //     std::cout << "error: " << fabs(yt1 - yt2) << std::endl;
    //   }
    // }
  }

  //  找 “已知区域” 和 “未知区域” 的交界（比如墙根），机器人往这走能发现新环境
  void FastPlannerManager::searchFrontier(const Eigen::Vector3d &p)
  {
    // show frontier for reference
    frontier_finder_->searchFrontiers();  // 调用frontier_finder_（边界查找器）找边界，同时会把边界可视化（比如在RViz里画出来，方便开发者看）
  }

  void FastPlannerManager::test()
  {
    auto t1 = ros::Time::now();
    std::cout << "test-------------------" << std::endl;

    Graph graph_yaw;
    int vid = 0;
  }

  // 输入：start（起点位置）、start_vel（起点速度）、start_acc（起点加速度）、goal（最终目标）
  // 输出：bool（规划成功返回true，失败返回false）
  bool FastPlannerManager::localExplore(Eigen::Vector3d start, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d goal)
  {
    local_data_.start_time_ = ros::Time::now(); // 记录“局部探索规划”开始时间（算总耗

    Eigen::Vector3d gi;
    double dist_to_goal = (goal - start).norm();  // 算“起点到最终目标的直线距离”
    if (dist_to_goal < 5.0)  // 如果离目标<5米（距离阈值，可调整），直接去目标
    {
      gi = goal;
      std::cout << "Select final gi" << std::endl;
    }
    else  // 离目标远，采样“候选中间目标”（找安全且能探索的点） 
    {
      // sample unifromly within sphere in the unknown free space
      // Do random number initialization
      // 1.1 初始化随机数生成器（用于随机采样候选点，每次采样点不一样）
      random_device rd; // 随机种子（从硬件获取，保证每次随机结果不同）
      default_random_engine eng = default_random_engine(rd());  // 随机数引擎
      uniform_real_distribution<double> rand_u(-1.0, 1.0);  // 生成[-1,1]的均匀随机数

      // Get sampled points
      // 1.2 采样“候选中间目标”（只采安全、能探索的点）
      vector<Eigen::Vector3d> points; // Sampled points // 存“候选中间目标”
      const int sample_num = 16;  // 采样16个候选点（数量越多，选最优的概率越高，但计算越慢）
      const double radius1 = 3.5; // 采样区域内半径（3.5米，太近的点没探索意义）
      const double radius2 = 5.0; // 采样区域外半径（5.0米，太远的点不安全）
      // 循环采样，直到采够16个安全点
      while (points.size() < sample_num)
      {
        Eigen::Vector3d pt; // 临时存一个采样点
        pt[0] = radius2 * rand_u(eng);  // x坐标：比如radius2=5，就是[-5,5]
        pt[1] = radius2 * rand_u(eng);  // y坐标：同上

        // inside disc and in forward direction [-45,45]
        if (pt.head(2).norm() > radius1 && pt.head(2).norm() < radius2 && // 平面距离在3.5-5米之间
            atan2(pt[1], pt[0]) < M_PI / 3.0 && atan2(pt[1], pt[0]) > -M_PI / 3.0)  // 角度在±60°之间（只往前）
        {
          // 1.2.3 调整采样点位置：从“相对起点”转到“世界坐标系”（比如起点在(10,20)，pt是(1,0)，就变成(11,20)）
          pt += start;
          // 1.2.4 设置采样点高度：0.5-1.5米（比如无人机在1米左右飞，避免撞地或太高）
          pt[2] = 0.5 * rand_u(eng) + 1;

          // check if in free space
          // 1.2.5 筛选2：只采“安全点”（不在障碍物里，且离障碍物>0.2米）
          Eigen::Vector3i pt_idx; // 把采样点坐标转成“距离图的索引”（方便查距离图）
          sdf_map_->posToIndex(pt, pt_idx);
          // 查距离图：如果是自由空间（FREE）且离障碍物>0.2米，就加入候选点
          if (sdf_map_->getOccupancy(pt_idx) == SDFMap::FREE && sdf_map_->getDistance(pt_idx) > 0.2)
          {
            points.push_back(pt);
          }
        }
      }

      // 1.2.6 把候选点转成ROS消息（方便在RViz里可视化，看采样的点对不对）
      pcl::PointCloud<pcl::PointXYZ> points_cloud;  // PCL点云格式
      for (int i = 0; i < points.size(); ++i)
      {
        pcl::PointXYZ pt;
        pt.x = points[i][0];
        pt.y = points[i][1];
        pt.z = points[i][2];
        points_cloud.points.push_back(pt);
      }
      points_cloud.width = points_cloud.points.size();
      points_cloud.height = 1;
      points_cloud.is_dense = true;
      points_cloud.header.frame_id = "world";
      sensor_msgs::PointCloud2 cloud_msg;
      pcl::toROSMsg(points_cloud, cloud_msg);

      std::cout << "Generate sample" << std::endl;

      // Evaluate infomation gain for each point
      vector<double> gains; // Gains for sampled points

      // parallel
      // const int              thread_num = 8;
      // vector<future<double>> futures(thread_num);
      // for (int i = 0; i < points.size(); i += thread_num) {
      //   for (int j = 0; j < thread_num; ++j) {
      //     Eigen::Vector3d dir = points[thread_num * i + j] - start;
      //     double          yk  = atan2(dir[1], dir[0]);
      //     futures[j] = std::async(std::launch::async, &HeadingPlanner::calcInfoGain,
      //     heading_planner_.get(),
      //                             points[thread_num * i + j], yk, j);
      //   }
      //   for (int j = 0; j < thread_num; ++j) {
      //     double gain = futures[j].get();
      //     gains.push_back(gain);
      //   }
      // }

      // sequential
      for (int i = 0; i < points.size(); ++i)
      {
        Eigen::Vector3d dir = points[i] - start;
        double yi = atan2(dir[1], dir[0]);
        double gain = heading_planner_->calcInfoGain(points[i], yi, 0);
        gains.push_back(gain);
      }

      // std::cout << "Calc gain" << std::endl;

      // for (int i = 0; i < points.size(); ++i) {
      //   std::cout << "pt: " << points[i].transpose() << ", g: " << gains[i] <<
      //   std::endl;
      // }

      // Select point with highest score
      const double dg = (goal - start).norm() + radius1;
      const double we = 1.0;
      const double wg = 1000.0;
      int idx = -1;
      double max_score = -1;
      for (int i = 0; i < points.size(); ++i)
      {
        double s = we * gains[i] + wg * (dg - (goal - points[i]).norm()) / dg;
        // std::cout << "score, gain: " << we * gains[i]
        //           << ", goal: " << wg * (dg - (goal - points[i]).norm()) / dg <<
        //           std::endl;
        if (s > max_score)
        {
          idx = i;
          max_score = s;
        }
      }
      gi = points[idx]; // Selected intermediate goal
      std::cout << "Select intermediate gi: " << gi.transpose() << std::endl;

      points_cloud.clear();
      points_cloud.points.push_back(pcl::PointXYZ(gi[0], gi[1], gi[2]));
      points_cloud.width = points_cloud.points.size();
      points_cloud.height = 1;
      points_cloud.is_dense = true;
      points_cloud.header.frame_id = "world";
      pcl::toROSMsg(points_cloud, cloud_msg);
    }

    // Plan locally using the intermediate goal gi

    // Search astar path and use it as initial value
    path_finder_->reset();
    int status = path_finder_->search(start, gi);
    if (status == Astar::NO_PATH)
    {
      return false;
    }
    auto path = path_finder_->getPath();
    double len = topo_prm_->pathLength(path);
    int seg_num = len / pp_.ctrl_pt_dist * 1.2;
    int ctrl_pt_num = seg_num + 3;
    double dt = (len / pp_.max_vel_) / seg_num;
    vector<Eigen::Vector3d> pts;
    topo_prm_->pathToGuidePts(path, seg_num + 1, pts); // Ctrl points of Bspline
    std::cout << "Find path" << std::endl;
    plan_data_.kino_path_ = path;

    // construct initial value
    Eigen::Matrix3d states2pts;
    states2pts << 1.0, -dt, (1 / 3.0) * dt * dt, 1.0, 0.0, -(1 / 6.0) * dt * dt, 1.0, dt,
        (1 / 3.0) * dt * dt;

    Eigen::Matrix3d state_xyz; // set start value
    state_xyz.row(0) = start;
    state_xyz.row(1) = start_vel;
    state_xyz.row(2) = start_acc;
    Eigen::Matrix3d p_tmp = states2pts * state_xyz;
    Eigen::Vector3d p0 = p_tmp.row(0);
    Eigen::Vector3d p1 = p_tmp.row(1);
    Eigen::Vector3d p2 = p_tmp.row(2);
    pts.insert(pts.begin(), p0);
    pts[1] = p1;
    pts[2] = p2;

    std::cout << "Set boundary value" << std::endl;

    // state_xyz.setZero();    // set end value
    // state_xyz.row(0)    = gi;
    // p_tmp               = states2pts * state_xyz;
    // Eigen::Vector3d p_2 = p_tmp.row(0);  // last 3 control pts
    // Eigen::Vector3d p_1 = p_tmp.row(1);
    // Eigen::Vector3d p_0 = p_tmp.row(2);
    // pts.push_back(p_0);
    // pts[pts.size() - 2] = p_1;
    // pts[pts.size() - 3] = p_2;

    // Optimize
    Eigen::MatrixXd ctrl_pts(pts.size(), 3);
    for (int i = 0; i < pts.size(); ++i)
    {
      ctrl_pts.row(i) = pts[i];
    }

    int cost_func = BsplineOptimizer::NORMAL_PHASE;
    bspline_optimizers_[0]->optimize(ctrl_pts, dt, cost_func, 1, 1);

    std::cout << "Optimze" << std::endl;

    // Refinement
    for (int i = 0; i < 3; ++i)
    {
      NonUniformBspline traj(ctrl_pts, 3, dt);
      traj.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_);
      double ratio = traj.checkRatio();
      std::cout << "ratio: " << ratio << std::endl;

      dt = ratio * dt;
      states2pts << 1.0, -dt, (1 / 3.0) * dt * dt, 1.0, 0.0, -(1 / 6.0) * dt * dt, 1.0, dt,
          (1 / 3.0) * dt * dt;
      p_tmp = states2pts * state_xyz;
      ctrl_pts.block<3, 3>(0, 0) = p_tmp;
      bspline_optimizers_[0]->optimize(ctrl_pts, dt, cost_func, 1, 1);
    }

    std::cout << "Local explore time: " << (ros::Time::now() - local_data_.start_time_).toSec()
              << std::endl;

    local_data_.position_traj_.setUniformBspline(ctrl_pts, 3, dt);
    updateTrajInfo();

    return true;
  }

} // namespace fast_planner