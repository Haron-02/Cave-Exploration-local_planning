#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <plan_manage/kino_replan_fsm.h>
#include <plan_manage/topo_replan_fsm.h>
#include <plan_manage/local_explore_fsm.h>

#include <plan_manage/backward.hpp>
namespace backward {
backward::SignalHandling sh;
}

using namespace fast_planner;

int main(int argc, char** argv) {
  ros::init(argc, argv, "fast_planner_node");
  ros::NodeHandle nh("~");

  int planner;
  nh.param("planner_node/planner", planner, -1);

  TopoReplanFSM topo_replan;
  KinoReplanFSM kino_replan;
  LocalExploreFSM local_explore;

  if (planner == 1) 
  {
    kino_replan.init(nh); //  初始化动力学规划器
  } 
  else if (planner == 2) 
  {
    topo_replan.init(nh); //  初始化拓扑重规划规划器
  } 
  else if (planner == 3)  
  {
    local_explore.init(nh); // 初始化局部探索规划器
  }

  ros::Duration(1.0).sleep(); // 休眠1秒，等待初始化完成（避免启动时的racecondition）
  ros::spin();  // 进入ROS事件循环，处理回调函数（保持节点运行）

  return 0;
}
