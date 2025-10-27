#include "bspline/non_uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "bspline/Bspline.h"
#include "bspline/BsplineProcess.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>
#include <poly_traj/polynomial_traj.h>
#include <active_perception/perception_utils.h>

#include <plan_manage/backward.hpp>
namespace backward
{
  backward::SignalHandling sh;
}
// 11. 简化代码：不用每次写“fast_planner::”，直接用类名
using fast_planner::NonUniformBspline;  // B样条轨迹类
using fast_planner::PerceptionUtils;  // 感知工具类
using fast_planner::Polynomial;
using fast_planner::PolynomialTraj;

ros::Publisher cmd_vis_pub, pos_cmd_pub, traj_pub;
nav_msgs::Odometry odom;
quadrotor_msgs::PositionCommand cmd;

// Info of generated traj
vector<NonUniformBspline> traj_;
double traj_duration_;
ros::Time start_time_;
int traj_id_;
int pub_traj_id_;

shared_ptr<PerceptionUtils> percep_utils_;

// Info of replan
bool receive_traj_ = false;
double replan_time_;

// Executed traj, commanded and real ones
vector<Eigen::Vector3d> traj_cmd_, traj_real_;

// Data for benchmark comparison
ros::Time start_time, end_time, last_time;
double energy;

// Loop correction
Eigen::Matrix3d R_loop;
Eigen::Vector3d T_loop;
bool isLoopCorrection = false;

//  计算路径长度
double calcPathLength(const vector<Eigen::Vector3d> &path)
{
  if (path.empty()) // 若路径为空（没走），长度0
    return 0;
  double len = 0.0; // 总长度初始为0
  for (int i = 0; i < path.size() - 1; ++i)
  {
    len += (path[i + 1] - path[i]).norm();
  }
  return len; // 返回总长度
}

// 功能：在RViz里画轨迹（用一串小球）
// 输入：path=轨迹点，resolution=小球大小，color=颜色（RGBA），id=轨迹ID（区分不同轨迹）
void displayTrajWithColor(vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color,
                          int id)
{
  visualization_msgs::Marker mk;  // 创建可视化消息（小球列表）
  mk.header.frame_id = "world"; 
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;
  traj_pub.publish(mk);

  mk.action = visualization_msgs::Marker::ADD;   // 动作：添加新轨迹
  // 小球的朝向（默认朝上，不用改）
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;
  // 小球颜色（比如color=(0,1,1,1)是青色，a=1表示不透明）
  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);
  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;
  geometry_msgs::Point pt;
  for (int i = 0; i < int(path.size()); i++)
  {
    pt.x = path[i](0);
    pt.y = path[i](1);
    pt.z = path[i](2);
    mk.points.push_back(pt);
  }
  traj_pub.publish(mk);
  ros::Duration(0.001).sleep();
}

//  3.3 画视野范围（drawFOV）
// 功能：在RViz里画机器人的“视野”（比如相机能看到的范围，用线框表示）
// 输入：list1=视野起点列表，list2=视野终点列表（每对起点+终点画一条线）
void drawFOV(const vector<Eigen::Vector3d> &list1, const vector<Eigen::Vector3d> &list2)
{
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.id = 0;
  mk.ns = "current_pose";
  mk.type = visualization_msgs::Marker::LINE_LIST;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;
  mk.color.a = 1.0;
  mk.scale.x = 0.04;
  mk.scale.y = 0.04;
  mk.scale.z = 0.04;

  // Clean old marker
  mk.action = visualization_msgs::Marker::DELETE;
  cmd_vis_pub.publish(mk);

  if (list1.size() == 0)
    return;

  // Pub new marker
  geometry_msgs::Point pt;
  for (int i = 0; i < int(list1.size()); ++i)
  {
    pt.x = list1[i](0);
    pt.y = list1[i](1);
    pt.z = list1[i](2);
    mk.points.push_back(pt);

    pt.x = list2[i](0);
    pt.y = list2[i](1);
    pt.z = list2[i](2);
    mk.points.push_back(pt);
  }
  mk.action = visualization_msgs::Marker::ADD;
  cmd_vis_pub.publish(mk);
}

//  画指令向量
void drawCmd(const Eigen::Vector3d &pos, const Eigen::Vector3d &vec, const int &id,
             const Eigen::Vector4d &color)
{
  visualization_msgs::Marker mk_state;
  mk_state.header.frame_id = "world";
  mk_state.header.stamp = ros::Time::now();
  mk_state.id = id;
  mk_state.type = visualization_msgs::Marker::ARROW;
  mk_state.action = visualization_msgs::Marker::ADD;

  mk_state.pose.orientation.w = 1.0;
  mk_state.scale.x = 0.1;
  mk_state.scale.y = 0.2;
  mk_state.scale.z = 0.3;

  geometry_msgs::Point pt;
  pt.x = pos(0);
  pt.y = pos(1);
  pt.z = pos(2);
  mk_state.points.push_back(pt);

  pt.x = pos(0) + vec(0);
  pt.y = pos(1) + vec(1);
  pt.z = pos(2) + vec(2);
  mk_state.points.push_back(pt);

  mk_state.color.r = color(0);
  mk_state.color.g = color(1);
  mk_state.color.b = color(2);
  mk_state.color.a = color(3);

  cmd_vis_pub.publish(mk_state);
}

//  重规划信号回调
void replanCallback(std_msgs::Empty msg)
{
  // Informed of new replan, end the current traj after some time
  const double time_out = 0.3;
  ros::Time time_now = ros::Time::now();
  double t_stop = (time_now - start_time_).toSec() + time_out + replan_time_;
  traj_duration_ = min(t_stop, traj_duration_);
}

//  新轨迹信号回调
void newCallback(std_msgs::Empty msg)
{
  // Clear the executed traj data
  traj_cmd_.clear();
  traj_real_.clear();
}

void odomCallbck(const nav_msgs::Odometry &msg)
{
  if (msg.child_frame_id == "X" || msg.child_frame_id == "O")
    return;
  odom = msg;
  traj_real_.push_back(
      Eigen::Vector3d(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));

  if (traj_real_.size() > 10000)
    traj_real_.erase(traj_real_.begin(), traj_real_.begin() + 1000);
}

//  闭环校正回调
void pgTVioCallback(geometry_msgs::Pose msg)
{
  // World to odom
  Eigen::Quaterniond q =
      Eigen::Quaterniond(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
  R_loop = q.toRotationMatrix();
  T_loop << msg.position.x, msg.position.y, msg.position.z;

  // cout << "R_loop: " << R_loop << endl;
  // cout << "T_loop: " << T_loop << endl;
}

//  轨迹可视化回调
void visCallback(const ros::TimerEvent &e)
{
  // Draw the executed traj (desired state)
  // displayTrajWithColor(traj_cmd_, 0.05, Eigen::Vector4d(1, 0, 0, 1), pub_traj_id_);
  // displayTrajWithColor(traj_cmd_, 0.05, Eigen::Vector4d(0, 1, 0, 1), pub_traj_id_);
  displayTrajWithColor(traj_cmd_, 0.2, Eigen::Vector4d(0, 1, 1, 1), pub_traj_id_);

  // displayTrajWithColor(traj_real_, 0.03, Eigen::Vector4d(0.925, 0.054, 0.964,
  // 1),
  //                      1);
}

//  B 样条服务回调
bool bsplineCallback(bspline::BsplineProcess::Request &req,
                     bspline::BsplineProcess::Response &resp)
{
  bspline::Bspline bspline_req = req.bspline;
  // Received traj should have ascending traj_id
  if (bspline_req.traj_id <= traj_id_)
  {
    ROS_ERROR("out of order bspline.");
    resp.is_success = false;
    return true;
  }

  // Parse the msg
  // 构建位置的B样条
  // 创建表格存位置控制点：行数=控制点数量，列数=3（x/y/z）
  Eigen::MatrixXd pos_pts(bspline_req.pos_pts.size(), 3);
  // 创建向量存时间节点（每个控制点对应的时间）
  Eigen::VectorXd knots(bspline_req.knots.size());
  // 把消息里的时间节点存到knots
  for (int i = 0; i < bspline_req.knots.size(); ++i)
  {
    knots(i) = bspline_req.knots[i];
  }
  // 把消息里的位置控制点存到pos_pts（x/y/z分别存）
  for (int i = 0; i < bspline_req.pos_pts.size(); ++i)
  {
    pos_pts(i, 0) = bspline_req.pos_pts[i].x;
    pos_pts(i, 1) = bspline_req.pos_pts[i].y;
    pos_pts(i, 2) = bspline_req.pos_pts[i].z;
  }
  // 用控制点和时间节点创建“位置B样条轨迹”（order是阶数，0.1是默认时间步长）
  NonUniformBspline pos_traj(pos_pts, bspline_req.order, 0.1);
  pos_traj.setKnot(knots);

  // 构建yaw角的B样条
  Eigen::MatrixXd yaw_pts(bspline_req.yaw_pts.size(), 1);
  for (int i = 0; i < bspline_req.yaw_pts.size(); ++i)
    yaw_pts(i, 0) = bspline_req.yaw_pts[i];
  NonUniformBspline yaw_traj(yaw_pts, 3, bspline_req.yaw_dt);
  // time_r = ros::Time::now() + ros::Duration(fp_->replan_time_);
  // bspline_req.start_time = info->start_time_ = (ros::Time::now() - time_r).toSec() > 0 ? ros::Time::now() : time_r;
  start_time_ = bspline_req.start_time; // max(预设的轨迹开始执行时间, 轨迹规划完的实际时间)
  traj_id_ = bspline_req.traj_id;

  // 存储一次的所有轨迹(位置、速度、加速度、yaw角、yaw角速度、yaw角加速度)
  traj_.clear();
  traj_.push_back(pos_traj);  // 0: 位置轨迹
  traj_.push_back(traj_[0].getDerivative());  // 1: 速度轨迹（位置的1阶导数）
  traj_.push_back(traj_[1].getDerivative());  // 2: 加速度轨迹（速度的1阶导数）
  traj_.push_back(yaw_traj);  // 3: 偏航角轨迹
  traj_.push_back(yaw_traj.getDerivative());  // 4: 偏航角速度（偏航角的1阶导数）
  traj_.push_back(traj_[2].getDerivative());  // 5: 偏航角加速度（加速度的1阶导数，备用）
  traj_duration_ = traj_[0].getTimeSum(); // 当前轨迹的总时长

  receive_traj_ = true;

  // Record the start time of flight
  if (start_time.isZero())
  {
    ROS_WARN("start flight");
    start_time = ros::Time::now();
  }

  resp.is_success = true;
  return true;
}

//B 样条话题回调
void bsplineCallback(const bspline::BsplineConstPtr &msg)
{
  // Received traj should have ascending traj_id
  if (msg->traj_id <= traj_id_)
  {
    ROS_ERROR("out of order bspline.");
    return;
  }

  // Parse the msg
  // 构建位置的B样条
  // 2. 解析位置B样条（和服务版完全一样）
  Eigen::MatrixXd pos_pts(msg->pos_pts.size(), 3);
  Eigen::VectorXd knots(msg->knots.size());
  for (int i = 0; i < msg->knots.size(); ++i)
  {
    knots(i) = msg->knots[i];
  }
  for (int i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(i, 0) = msg->pos_pts[i].x;
    pos_pts(i, 1) = msg->pos_pts[i].y;
    pos_pts(i, 2) = msg->pos_pts[i].z;
  }
  NonUniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  // 构建yaw角的B样条
  // 3. 解析偏航角B样条（和服务版完全一样）
  Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
  for (int i = 0; i < msg->yaw_pts.size(); ++i)
    yaw_pts(i, 0) = msg->yaw_pts[i];
  NonUniformBspline yaw_traj(yaw_pts, 3, msg->yaw_dt);
  // time_r = ros::Time::now() + ros::Duration(fp_->replan_time_);
  // msg->start_time = info->start_time_ = (ros::Time::now() - time_r).toSec() > 0 ? ros::Time::now() : time_r;
  start_time_ = msg->start_time; // max(预设的轨迹开始执行时间, 轨迹规划完的实际时间)
  traj_id_ = msg->traj_id;

  // 存储一次的所有轨迹(位置、速度、加速度、yaw角、yaw角速度、yaw角加速度)
  traj_.clear();
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());
  traj_.push_back(yaw_traj);
  traj_.push_back(yaw_traj.getDerivative());
  traj_.push_back(traj_[2].getDerivative());
  traj_duration_ = traj_[0].getTimeSum();

  receive_traj_ = true;

  // Record the start time of flight
  if (start_time.isZero())
  {
    ROS_WARN("start flight");
    start_time = ros::Time::now();
  }
}

//  指令生成回调
void cmdCallback(const ros::TimerEvent &e)
{
  // No publishing before receive traj data
  if (!receive_traj_)
    return;

  ros::Time time_now = ros::Time::now();
  double t_cur = (time_now - start_time_).toSec();
  Eigen::Vector3d pos, vel, acc, jer;
  double yaw, yawdot;

  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    // Current time within range of planned traj
    pos = traj_[0].evaluateDeBoorT(t_cur);
    vel = traj_[1].evaluateDeBoorT(t_cur);
    acc = traj_[2].evaluateDeBoorT(t_cur);
    yaw = traj_[3].evaluateDeBoorT(t_cur)[0];
    yawdot = traj_[4].evaluateDeBoorT(t_cur)[0];
    jer = traj_[5].evaluateDeBoorT(t_cur);
  }
  else if (t_cur >= traj_duration_)
  {
    // Current time exceed range of planned traj
    // keep publishing the final position and yaw
    pos = traj_[0].evaluateDeBoorT(traj_duration_);
    vel.setZero();
    acc.setZero();
    yaw = traj_[3].evaluateDeBoorT(traj_duration_)[0];
    yawdot = 0.0;

    // Report info of the whole flight
    double len = calcPathLength(traj_cmd_);
    double flight_t = (end_time - start_time).toSec();
    ROS_WARN_THROTTLE(2, "flight time: %lf, path length: %lf, mean vel: %lf, energy is: % lf ", flight_t,
                      len, len / flight_t, energy);
  }
  else
  {
    cerr << "[Traj server]: invalid time." << endl;
  }

  if (isLoopCorrection)
  {
    pos = R_loop.transpose() * (pos - T_loop);
    vel = R_loop.transpose() * vel;
    acc = R_loop.transpose() * acc;

    Eigen::Vector3d yaw_dir(cos(yaw), sin(yaw), 0);
    yaw_dir = R_loop.transpose() * yaw_dir;
    yaw = atan2(yaw_dir[1], yaw_dir[0]);
  }

  cmd.header.stamp = time_now;
  cmd.trajectory_id = traj_id_;
  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);
  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);
  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);
  cmd.yaw = yaw;
  cmd.yaw_dot = yawdot;
  pos_cmd_pub.publish(cmd);

  // Draw cmd
  // Eigen::Vector3d dir(cos(yaw), sin(yaw), 0.0);
  // drawCmd(pos, 2 * dir, 2, Eigen::Vector4d(1, 1, 0, 0.7));
  // drawCmd(pos, vel, 0, Eigen::Vector4d(0, 1, 0, 1));
  // drawCmd(pos, acc, 1, Eigen::Vector4d(0, 0, 1, 1));
  // drawCmd(pos, pos_err, 3, Eigen::Vector4d(1, 1, 0, 0.7));
  percep_utils_->setPose(pos, yaw);
  vector<Eigen::Vector3d> l1, l2;
  percep_utils_->getFOV(l1, l2);
  drawFOV(l1, l2);

  // Record info of the executed traj
  if (traj_cmd_.size() == 0)
  {
    // Add the first position
    traj_cmd_.push_back(pos);
  }
  else if ((pos - traj_cmd_.back()).norm() > 1e-6)
  {
    // Add new different commanded position
    traj_cmd_.push_back(pos);
    double dt = (time_now - last_time).toSec();
    energy += jer.squaredNorm() * dt;
    end_time = ros::Time::now();
  }
  last_time = time_now;

  // if (traj_cmd_.size() > 100000)
  //   traj_cmd_.erase(traj_cmd_.begin(), traj_cmd_.begin() + 1000);
}

void test()
{
  // Test B-spline
  // Generate the first B-spline's control points from a sin curve
  vector<Eigen::Vector3d> samples;
  const double dt1 = M_PI / 6.0;
  for (double theta = 0; theta <= 2 * M_PI; theta += dt1)
  {
    Eigen::Vector3d sample(theta, sin(theta), 1);
    samples.push_back(sample);
  }
  Eigen::MatrixXd points(samples.size(), 3);
  for (int i = 0; i < samples.size(); ++i)
    points.row(i) = samples[i].transpose();

  //生成多项式轨迹(基于正弦点，先做一条平滑轨迹)
  Eigen::VectorXd 
  times(samples.size() - 1);  // 每段的时间
  times.setConstant(dt1); // 每段时间=步长
  times[0] += dt1;  // 第一段加时间（启动缓冲）
  times[times.rows() - 1] += dt1; // 最后一段加时间（停止缓冲）
  Eigen::Vector3d zero(0, 0, 0);  // 速度/加速度初始为0

  PolynomialTraj poly;
  PolynomialTraj::waypointsTraj(points, zero, zero, zero, zero, times, poly);

  const int degree = 5; // B样条阶数（5阶更平滑）
  double duration = poly.getTotalTime();  // 多项式轨迹总时长
  vector<Eigen::Vector3d> traj_pts; // 多项式轨迹的点
  for (double ts = 0; ts <= duration; ts += 0.01)
    traj_pts.push_back(poly.evaluate(ts, 0)); // 采样多项式轨迹点
  // displayTrajWithColor(traj_pts, 0.05, Eigen::Vector4d(1, 0, 0, 1), 99);

  // Fit the polynomialw with B-spline
  const int seg_num = 30; // B样条段数（30段，足够平滑）
  double dt = duration / seg_num; // 每段时间
  vector<Eigen::Vector3d> point_set, boundary_der;
  for (double ts = 0; ts <= 1e-3 + duration; ts += dt)
    point_set.push_back(poly.evaluate(ts, 0));

  // 边界导数：起点速度、终点速度、起点加速度、终点加速度
  boundary_der.push_back(poly.evaluate(0, 1));
  boundary_der.push_back(poly.evaluate(duration, 1));

  boundary_der.push_back(poly.evaluate(0, 2));
  boundary_der.push_back(poly.evaluate(duration, 2));

  // 5. 生成B样条轨迹 
  Eigen::MatrixXd ctrl_pts; // B样条控制点
  NonUniformBspline::parameterizeToBspline(dt, point_set, boundary_der, degree, ctrl_pts);  
  NonUniformBspline fitted(ctrl_pts, degree, dt); // 拟合的B样条轨迹

  traj_pts.clear(); // 清空多项式轨迹点
  double duration2 = fitted.getTimeSum(); // B样条总时长
  for (double ts = 0; ts <= duration2; ts += 0.01)
    traj_pts.push_back(fitted.evaluateDeBoorT(ts)); // 采样B样条轨迹点

  // 把控制点转成列表，画出来（黄色，0.1米大
  vector<Eigen::Vector3d> ctrl_pts_vec;
  for (int i = 0; i < ctrl_pts.rows(); ++i)
  {
    Eigen::Vector3d pr = ctrl_pts.row(i).transpose();
    ctrl_pts_vec.push_back(pr);
  }
  displayTrajWithColor(ctrl_pts_vec, 0.1, Eigen::Vector4d(1, 1, 0, 1), 98);
  displayTrajWithColor(traj_pts, 0.05, Eigen::Vector4d(1, 0, 0, 1), 99);

  auto vel = fitted.getDerivative();
  auto acc = vel.getDerivative();

  ros::Duration(0.1).sleep();

  // Pub the traj
  auto t1 = ros::Time::now();
  double tn = (ros::Time::now() - t1).toSec();
  while (tn < duration && ros::ok())
  {
    // Eigen::Vector3d p = bspline.evaluateDeBoorT(tn);
    // Eigen::Vector3d v = vel.evaluateDeBoorT(tn);
    // Eigen::Vector3d a = acc.evaluateDeBoorT(tn);
    // 从B样条中插值位置、速度、加速度
    Eigen::Vector3d p = fitted.evaluateDeBoorT(tn);
    Eigen::Vector3d v = vel.evaluateDeBoorT(tn);
    Eigen::Vector3d a = acc.evaluateDeBoorT(tn);

    cmd.header.stamp = ros::Time::now();
    cmd.position.x = p(0);
    cmd.position.y = p(1);
    cmd.position.z = p(2);
    cmd.velocity.x = v(0);
    cmd.velocity.y = v(1);
    cmd.velocity.z = v(2);
    cmd.acceleration.x = a(0);
    cmd.acceleration.y = a(1);
    cmd.acceleration.z = a(2);
    pos_cmd_pub.publish(cmd); // 发布指令

    ros::Duration(0.02).sleep();
    tn = (ros::Time::now() - t1).toSec(); // 更新已执行时间
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");

  // 2. 创建“B样条服务”（接收上层规划器的轨迹服务请求）
  ros::ServiceServer bspline_server = node.advertiseService("planning/bspline_service", bsplineCallback);

  // ros::Subscriber bspline_sub = node.subscribe("planning/bspline", 10, bsplineCallback);
  // 3. 创建订阅器（接收各种消息）
  ros::Subscriber replan_sub = node.subscribe("planning/replan", 10, replanCallback);
  ros::Subscriber new_sub = node.subscribe("planning/new", 10, newCallback);  //新轨迹信号
  ros::Subscriber odom_sub = node.subscribe("/odom_world", 50, odomCallbck);
  ros::Subscriber pg_T_vio_sub = node.subscribe("/loop_fusion/pg_T_vio", 10, pgTVioCallback); //闭环校正

  cmd_vis_pub = node.advertise<visualization_msgs::Marker>("planning/position_cmd_vis", 10);  //指令可视化
  pos_cmd_pub = node.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50); //位置指令（核心）
  traj_pub = node.advertise<visualization_msgs::Marker>("planning/travel_traj", 10);  //轨迹可视化

  ros::Timer cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback);
  ros::Timer vis_timer = node.createTimer(ros::Duration(0.25), visCallback);

  // 6. 从参数服务器读配置（可在launch文件中修改，不用改代码）
  nh.param("traj_server/pub_traj_id", pub_traj_id_, -1);
  nh.param("fsm/replan_time", replan_time_, 0.1);
  nh.param("loop_correction/isLoopCorrection", isLoopCorrection, false);  // 是否开启闭环校正

  Eigen::Vector3d init_pos;
  nh.param("traj_server/init_x", init_pos[0], 0.0); 
  nh.param("traj_server/init_y", init_pos[1], 0.0);
  nh.param("traj_server/init_z", init_pos[2], 0.0);

  ROS_WARN("[Traj server]: init...");
  ros::Duration(1.0).sleep();

  // Control parameter
  cmd.kx = {5.7, 5.7, 6.2}; // 位置比例系数（x/y/z）
  cmd.kv = {3.4, 3.4, 4.0}; // 速度比例系数（x/y/z）

  std::cout << start_time.toSec() << std::endl;
  std::cout << end_time.toSec() << std::endl;

  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;
  cmd.position.x = init_pos[0];
  cmd.position.y = init_pos[1];
  cmd.position.z = init_pos[2];
  cmd.velocity.x = 0.0;
  cmd.velocity.y = 0.0;
  cmd.velocity.z = 0.0;
  cmd.acceleration.x = 0.0;
  cmd.acceleration.y = 0.0;
  cmd.acceleration.z = 0.0;
  cmd.yaw = 0.0;
  cmd.yaw_dot = 0.0;

  percep_utils_.reset(new PerceptionUtils(nh));

  // test();
  // Initialization for exploration, move upward and downward init_motion
  // for (int i = 0; i < 100; ++i)
  // {
  //   cmd.position.z += 0.01;
  //   pos_cmd_pub.publish(cmd);
  //   ros::Duration(0.01).sleep();
  // }
  // for (int i = 0; i < 100; ++i)
  // {
  //   cmd.position.z -= 0.01;
  //   pos_cmd_pub.publish(cmd);
  //   ros::Duration(0.01).sleep();
  // }


  // ros::Duration(1.0).sleep();
  // for (int i = 0; i < 100; ++i)
  // {
  //   cmd.position.x -= 0.01;
  //   pos_cmd_pub.publish(cmd);
  //   ros::Duration(0.01).sleep();
  // }

  R_loop = Eigen::Quaterniond(1, 0, 0, 0).toRotationMatrix();
  T_loop = Eigen::Vector3d(0, 0, 0);

  ROS_WARN("[Traj server]: ready.");
  ros::spin();

  return 0;
}
