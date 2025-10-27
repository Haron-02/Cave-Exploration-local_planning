//  观测点规划的图节点类
//  计算两个观测点之间的移动代价，兼顾路径安全性、运动约束（速度、偏航角等）
#include <active_perception/graph_node.h>
#include <path_searching/astar2.h>
#include <plan_env/sdf_map.h>
#include <plan_env/raycast.h>

namespace fast_planner
{
  // Static data
  double ViewNode::vm_;
  double ViewNode::am_;
  double ViewNode::yd_;
  double ViewNode::ydd_;
  double ViewNode::w_dir_;
  shared_ptr<Astar> ViewNode::astar_; //  路径搜索器
  shared_ptr<RayCaster> ViewNode::caster_;  //  射线检测器
  shared_ptr<SDFMap> ViewNode::map_;  //  环境地图

  // Graph node for viewpoints planning
  ViewNode::ViewNode(const Vector3d &p, const double &y)
  {
    pos_ = p; // 位置设为输入的p（比如三维坐标x,y,z）
    yaw_ = y; // 偏航角设为输入的y（比如机器人朝哪个方向看）
    parent_ = nullptr;  // 父节点初始为空（还没找到“从哪个点过来的”）
    vel_.setZero(); // vel is zero by default, should be set explicitly
  }

  //  计算到目标节点的代价
  double ViewNode::costTo(const ViewNode::Ptr &node)
  {
    vector<Vector3d> path;  // 用来存路径的临时变量
    double c = ViewNode::computeCost(pos_, node->pos_, yaw_, node->yaw_, vel_, yaw_dot_, path);
    // std::cout << "cost from " << id_ << " to " << node->id_ << " is: " << c << std::endl;
    return c;
  }

  //  搜索两点间的安全路径并计算长度
  double ViewNode::searchPath(const Vector3d &p1, const Vector3d &p2, vector<Vector3d> &path)
  {
    // Try connect two points with straight line
    bool safe = true; // 先假设直线是安全的
    Vector3i idx; // 用来存地图格子坐标的临时变量
    caster_->input(p1, p2); // 给射线检测器输入起点p1和终点p2
    while (caster_->nextId(idx))  // 射线检测器一步步检查直线上的每个格子
    {
      // 检查这个格子：如果是障碍物（1）、未知区域（UNKNOWN），或者不在地图范围内，就不安全
      if (map_->getInflateOccupancy(idx) == 1 || map_->getOccupancy(idx) == SDFMap::UNKNOWN ||
          !map_->isInBox(idx))
      {
        safe = false; // 标记为不安全
        break;  // 跳出循环，不用再检查了
      }
    }
    if (safe) // 如果直线安全
    {
      path = {p1, p2};  // 路径就是两点直线
      return (p1 - p2).norm();  // 返回直线距离（作为路径长度）
    }
    // Search a path using decreasing resolution
    // 直线不安全，就用A*算法找路径（分辨率递减，这里只有0.4）
    vector<double> res = {0.4}; // 路径搜索的分辨率（0.4米/步，数值越小越精细）
    for (int k = 0; k < res.size(); ++k)
    {
      astar_->reset();  // 重置A*算法（清空之前的计算）
      astar_->setResolution(res[k]);  // 设置当前分辨率
      if (astar_->search(p1, p2) == Astar::REACH_END) // 如果A*找到了从p1到p2的路径
      {
        path = astar_->getPath(); // 取出路径
        return astar_->pathLength(path);  // 返回路径总长度
      }
    }
    // Use Astar early termination cost as an estimate
    // 如果A*也没找到路径，就返回一个很大的惩罚值（1000）
    path = {p1, p2};  // 路径默认还是直线（但其实不安全）
    return 1000;
  }

  //  综合计算移动代价（核心逻辑）
  double ViewNode::computeCost(const Vector3d &p1, const Vector3d &p2, const double &y1, const double &y2,
                               const Vector3d &v1, const double &yd1, vector<Vector3d> &path)
  {
    // Cost of position change
    double pos_cost = ViewNode::searchPath(p1, p2, path) / vm_; // 位置移动的代价：路径长度除以最大速度（相当于“花多少时间移动”）

    // Consider velocity change
    if (v1.norm() > 1e-3) // 如果当前速度的大小（ norm ）大于0.001（相当于“在移动”）
    {
      Vector3d dir = (p2 - p1).normalized();  // 目标方向（从p1到p2的单位向量）
      Vector3d vdir = v1.normalized();  // 当前速度方向（单位向量）
      double diff = acos(vdir.dot(dir));  // 计算两个方向的夹角（用点积求余弦，再反余弦得角度）
      pos_cost += w_dir_ * diff;  // 计算两个方向的夹角（用点积求余弦，再反余弦得角度）
      // double vc = v1.dot(dir);
      // pos_cost += w_dir_ * pow(vm_ - fabs(vc), 2) / (2 * vm_ * am_);
      // if (vc < 0)
      //   pos_cost += w_dir_ * 2 * fabs(vc) / am_;
    }

    // Cost of yaw change
    double diff = fabs(y2 - y1);  // 计算两个朝向的差值（绝对值）
    diff = min(diff, 2 * M_PI - diff);  // 取最小角度差（比如350度和10度，其实只差10度）
    double yaw_cost = diff / yd_; // 除以最大偏航角速度（相当于“花多少时间转头”）
    return max(pos_cost, yaw_cost); // 总代价取“移动时间”和“转头时间”中较大的那个

    // // Consider yaw rate change
    // if (fabs(yd1) > 1e-3)
    // {
    //   double diff1 = y2 - y1;
    //   while (diff1 < -M_PI)
    //     diff1 += 2 * M_PI;
    //   while (diff1 > M_PI)
    //     diff1 -= 2 * M_PI;
    //   double diff2 = diff1 > 0 ? diff1 - 2 * M_PI : 2 * M_PI + diff1;
    // }
    // else
    // {
    // }
  }
}