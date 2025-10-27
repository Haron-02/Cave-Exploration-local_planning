//  接收机器人的 B 样条轨迹，实时解析出每个时刻的位置、速度、加速度等控制指令，并发布给机器人执行，同时可视化轨迹和指令。
#include "bspline/non_uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "bspline/Bspline.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>
#include <plan_manage/plan_container.hpp>
#include <memory>

using fast_planner::LocalTrajData;
using fast_planner::LocalTrajServer;
using fast_planner::LocalTrajState;
using fast_planner::NonUniformBspline;


ros::Publisher cmd_vis_pub, pos_cmd_pub, traj_pub;  // 发布器：用于发布可视化标记、位置指令、轨迹
quadrotor_msgs::PositionCommand cmd;  // 位置指令对象：存储要发给机器人的位置、速度、加速度等信息
double pos_gain[3] = { 5.7, 5.7, 6.2 };
double vel_gain[3] = { 3.4, 3.4, 4.0 };
int pub_traj_id_; // 轨迹可视化的ID（用于区分不同轨迹）

std::shared_ptr<LocalTrajServer> local_traj_; // 本地轨迹服务器（智能指针，负责管理接收到的轨迹，比如存储、查询某时刻的状态）
// bool                      receive_traj_ = false;
// vector<NonUniformBspline> traj_;
// double                    traj_duration_;
// ros::Time                 start_time_;
// int                       traj_id_;

vector<Eigen::Vector3d> executed_cmd_;  // 存储已经执行过的指令位置（用于可视化机器人走过的路径）

//  画轨迹点
void displayTrajWithColor(vector<Eigen::Vector3d> traj, double resolution, Eigen::Vector4d color,
                          int id) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world"; // 参考坐标系（世界坐标系）
  mk.header.stamp = ros::Time::now(); // 时间戳（当前时间）
  mk.type = visualization_msgs::Marker::SPHERE_LIST;  // 类型：一系列小球
  mk.action = visualization_msgs::Marker::DELETE; // 先删除旧的标记（避免残留）
  mk.id = id;
  traj_pub.publish(mk); // 发布删除指令

  mk.action = visualization_msgs::Marker::ADD;  // 再添加新的标记
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;  // 姿态（无旋转）
  mk.color.r = color(0);  // 颜色（r,g,b,a）
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);
  mk.scale.x = resolution;  // 小球大小
  mk.scale.y = resolution;
  mk.scale.z = resolution;

  geometry_msgs::Point pt;  // 点的消息格式
  for (int i = 0; i < int(traj.size()); i++) {  // 遍历所有轨迹点
    pt.x = traj[i](0);  // 填充x,y,z坐标
    pt.y = traj[i](1);
    pt.z = traj[i](2);
    mk.points.push_back(pt);  // 加入标记中
  }
  traj_pub.publish(mk); // 发布轨迹可视化消息
  ros::Duration(0.001).sleep(); // 稍等一下，确保消息发送
}

//  画指令箭头
// 参数：pos（起点）、vec（方向向量）、id（标记ID）、color（颜色）
void drawCmd(const Eigen::Vector3d& pos, const Eigen::Vector3d& vec, const int& id,
             const Eigen::Vector4d& color) {
  visualization_msgs::Marker mk_state;  // 可视化标记对象
  mk_state.header.frame_id = "world"; // 世界坐标系
  mk_state.header.stamp = ros::Time::now(); // 当前时间
  mk_state.id = id;
  mk_state.type = visualization_msgs::Marker::ARROW;  // 类型：箭头
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

void pauseCallback(std_msgs::Empty msg) {
  local_traj_->resetDuration();
}

void visCallback(const ros::TimerEvent& e) {
  displayTrajWithColor(executed_cmd_, 0.05, Eigen::Vector4d(0, 1, 0, 1), pub_traj_id_);
}

//  bsplineCallback：接收 B 样条轨迹并处理
// 功能：收到规划器发布的B样条轨迹后，解析并存储到轨迹服务器
void bsplineCallback(bspline::BsplineConstPtr msg) {
  // 解析位置轨迹：从消息中提取控制点和节点（B样条的关键参数）
  // parse pos traj
  Eigen::MatrixXd pos_pts(msg->pos_pts.size(), 3);  // 位置控制点（n行3列，n个点）
  Eigen::VectorXd knots(msg->knots.size()); // 节点向量（B样条的时间分割点）
  for (int i = 0; i < msg->knots.size(); ++i) {
    knots(i) = msg->knots[i]; // 填充节点数据
  }
  for (int i = 0; i < msg->pos_pts.size(); ++i) { // 填充位置控制点
    pos_pts(i, 0) = msg->pos_pts[i].x;  // x坐标
    pos_pts(i, 1) = msg->pos_pts[i].y;  // y坐标
    pos_pts(i, 2) = msg->pos_pts[i].z;  // z坐标
  }
  // 创建位置B样条轨迹（控制点、阶数、时间间隔）
  NonUniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);  // 设置节点向量

  // parse yaw traj
  // 解析偏航角轨迹（机器人朝向）
  Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);  // 偏航角控制点（n行1列）
  for (int i = 0; i < msg->yaw_pts.size(); ++i) {
    yaw_pts(i, 0) = msg->yaw_pts[i];  // 填充偏航角数据
  }
  // 创建偏航角B样条轨迹
  NonUniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

  // 把解析出的轨迹打包成LocalTrajData（轨迹数据结构）
  LocalTrajData traj;
  traj.start_time_ = msg->start_time;
  traj.traj_id_ = msg->traj_id;
  traj.position_traj_ = pos_traj;
  traj.velocity_traj_ = traj.position_traj_.getDerivative();
  traj.acceleration_traj_ = traj.velocity_traj_.getDerivative();
  traj.yaw_traj_ = yaw_traj;
  traj.yawdot_traj_ = traj.yaw_traj_.getDerivative();
  traj.duration_ = pos_traj.getTimeSum();
  local_traj_->addTraj(traj); // 把轨迹添加到本地轨迹服务器（由服务器管理，后续可以查询）
}

//定时生成控制指令并发布
void cmdCallback(const ros::TimerEvent& e) {
  /* no publishing before receive traj_ */

  LocalTrajState traj_cmd;
  ros::Time time_now = ros::Time::now();
  bool status = local_traj_->evaluate(time_now, traj_cmd);
  if (status) {
    cmd.header.stamp = time_now;
    cmd.header.frame_id = "world";
    cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
    cmd.trajectory_id = traj_cmd.id;

    cmd.position.x = traj_cmd.pos(0);
    cmd.position.y = traj_cmd.pos(1);
    cmd.position.z = traj_cmd.pos(2);
    cmd.velocity.x = traj_cmd.vel(0);
    cmd.velocity.y = traj_cmd.vel(1);
    cmd.velocity.z = traj_cmd.vel(2);
    cmd.acceleration.x = traj_cmd.acc(0);
    cmd.acceleration.y = traj_cmd.acc(1);
    cmd.acceleration.z = traj_cmd.acc(2);
    cmd.yaw = traj_cmd.yaw;
    cmd.yaw_dot = traj_cmd.yawdot;
    pos_cmd_pub.publish(cmd);

    // draw cmd
    // drawCmd(pos, vel, 0, Eigen::Vector4d(0, 1, 0, 1));
    // drawCmd(pos, acc, 1, Eigen::Vector4d(0, 0, 1, 1));

    Eigen::Vector3d dir(cos(traj_cmd.yaw), sin(traj_cmd.yaw), 0.0);
    drawCmd(traj_cmd.pos, 2 * dir, 2, Eigen::Vector4d(1, 1, 0, 0.7));

    if (executed_cmd_.size() == 0 ||
        (executed_cmd_.size() > 0 && (traj_cmd.pos - executed_cmd_.back()).norm() > 1e-6)) {
      executed_cmd_.push_back(traj_cmd.pos);
      if (executed_cmd_.size() > 10000)
        executed_cmd_.erase(executed_cmd_.begin(), executed_cmd_.begin() + 1000);
    }
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle node; // 全局节点句柄（用于订阅/发布全局话题）
  ros::NodeHandle nh("~");  // 私有节点句柄（用于读取私有参数）

  // 创建订阅器：接收规划器发布的B样条轨迹、暂停指令
  ros::Subscriber bspline_sub = node.subscribe("planning/bspline", 10, bsplineCallback);
  ros::Subscriber pause_sub = node.subscribe("planning/pause", 10, pauseCallback);

  // 创建发布器：发布指令可视化、位置指令、轨迹可视化
  cmd_vis_pub = node.advertise<visualization_msgs::Marker>("planning/position_cmd_vis", 10);
  pos_cmd_pub = node.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);
  traj_pub = node.advertise<visualization_msgs::Marker>("planning/travel_traj", 10);

  ros::Timer cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback);
  ros::Timer vis_timer = node.createTimer(ros::Duration(0.25), visCallback);

  /* control parameter */
  // 设置控制指令中的增益参数（用于机器人控制器，让运动更平稳）
  cmd.kx[0] = pos_gain[0];
  cmd.kx[1] = pos_gain[1];
  cmd.kx[2] = pos_gain[2];

  cmd.kv[0] = vel_gain[0];
  cmd.kv[1] = vel_gain[1];
  cmd.kv[2] = vel_gain[2];

  // 从参数服务器读取轨迹可视化ID（如果没设置，用默认值-1）
  nh.param("traj_server/pub_traj_id", pub_traj_id_, -1);

  // 初始化本地轨迹服务器（智能指针实例化）
  local_traj_.reset(new LocalTrajServer);

  ros::Duration(1.0).sleep();

  ROS_WARN("[Traj server]: ready.");
  ros::spin();

  return 0;
}
