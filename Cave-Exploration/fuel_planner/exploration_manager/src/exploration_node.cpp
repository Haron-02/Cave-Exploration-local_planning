#include <ros/ros.h>
#include <exploration_manager/fast_exploration_fsm.h>

#include <plan_manage/backward.hpp>
namespace backward
{
  backward::SignalHandling sh;
}

using namespace fast_planner;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "exploration_node");
  ros::NodeHandle nh("~");

  FastExplorationFSM expl_fsm;
  expl_fsm.init(nh);

  // ros::Duration(5.0).sleep();
  // ROS_WARN("sleep for 5s has been end!");

  // 启动两个线程处理全局Callback队列
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();
  // 单线程
  // ros::spin();


  // 启动两个线程处理全局Callback队列
  // ros::MultiThreadedSpinner spinner(2);
  // spinner.spin();


  return 0;
}
