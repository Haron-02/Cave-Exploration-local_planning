#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <cascadePID.hpp>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/PositionCommand.h>

using namespace std;
using namespace Eigen;

ros::Publisher control_RPM_pub;

Vector3d pos_des, vel_des, acc_des;
double yaw_des = 0.0;
cascadePID quad_PID(20);

ros::Time t_init, t2;

nav_msgs::Odometry current_odom, last_odom;
int first_odom_flag = 0;
void Odom_callback(const nav_msgs::Odometry &odom)
{
    if (first_odom_flag == 1)
    {
        last_odom = current_odom;
    }
    else
    {
        first_odom_flag = 1;
    }
    current_odom = odom;
    ROS_INFO("current odom received");
}

geometry_msgs::PoseStamped pose_cmd;
int control_flag = 0;
void cmd_callback(const geometry_msgs::PoseStamped &msg)
{
    pose_cmd = msg;
    control_flag = 1;
}

void fuel_position_cmd_callback(const quadrotor_msgs::PositionCommand::ConstPtr &cmd)
{
    pos_des = Eigen::Vector3d(cmd->position.x, cmd->position.y, cmd->position.z);
    vel_des = Eigen::Vector3d(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);
    acc_des = Eigen::Vector3d(cmd->acceleration.x, cmd->acceleration.y, cmd->acceleration.z);

    yaw_des = cmd->yaw;
}

void run_control(const ros::TimerEvent &event)
{
    if (first_odom_flag == 1)
    {

        // ROS_INFO("start running controller");

        // run controller
        Vector3d current_euler;
        Eigen::Quaterniond quaternion_temp(current_odom.pose.pose.orientation.w, current_odom.pose.pose.orientation.x, current_odom.pose.pose.orientation.y, current_odom.pose.pose.orientation.z);
        Vector3d current_pos, current_vel;
        current_pos << current_odom.pose.pose.position.x, current_odom.pose.pose.position.y, current_odom.pose.pose.position.z;
        current_vel << current_odom.twist.twist.linear.x, current_odom.twist.twist.linear.y, current_odom.twist.twist.linear.z;
        quad_PID.FeedbackInput(current_pos, current_vel, quaternion_temp);
        // ROS_INFO("current_pos = %f,%f,%f",current_pos(0),current_pos(1),current_pos(2));
        Eigen::Vector3d eulerAngle_des;
        eulerAngle_des << 0.1, 0.1, 0.0;
        // quad_PID.setAngledes(eulerAngle_des);

        // ROS_INFO("set des angle");

        if (control_flag == 1)
        {
            pos_des << pose_cmd.pose.position.x, pose_cmd.pose.position.y, pose_cmd.pose.position.z;

            Eigen::Quaterniond q_des(pose_cmd.pose.orientation.w, pose_cmd.pose.orientation.x, pose_cmd.pose.orientation.y, pose_cmd.pose.orientation.z);
            Eigen::Matrix3d R_des;
            R_des = q_des.matrix();
            Eigen::Vector3d x_body;
            x_body << 1, 0, 0;
            x_body = R_des * x_body;

            yaw_des = atan2(x_body(1), x_body(0)); // pose_cmd.pose.orientation.z
            t2 = ros::Time::now();
            // yaw_des = 3.1415926;//20.8 * (t2-t_init).toSec()
        }

        quad_PID.setOdomdes(pos_des, vel_des, acc_des, yaw_des);

        // ROS_INFO("set des odom");

        quad_PID.RunController();
        Eigen::Vector3d Torque_des;
        Torque_des = quad_PID.getTorquedes();
        ROS_INFO("Torque_des = %f,%f,%f", Torque_des(0), Torque_des(1), Torque_des(2));
        Vector4d RPM_output;
        RPM_output = quad_PID.getRPMoutputs();
        // ROS_INFO("RPM_output = %f,%f,%f,%f",RPM_output(0),RPM_output(1),RPM_output(2),RPM_output(3));

        std_msgs::Float32MultiArray rpm_array;
        float rpm_data[4];
        rpm_data[0] = RPM_output(0);
        rpm_data[1] = RPM_output(1);
        rpm_data[2] = RPM_output(2);
        rpm_data[3] = RPM_output(3);
        // rpm_array.data = rpm_data;
        rpm_array.data.clear();
        rpm_array.data.push_back(RPM_output(0));
        rpm_array.data.push_back(RPM_output(1));
        rpm_array.data.push_back(RPM_output(2));
        rpm_array.data.push_back(RPM_output(3));
        control_RPM_pub.publish(rpm_array);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cascade_PID");

    ros::NodeHandle n("~");

    double init_x, init_y, init_z;
    double init_yaw;
    double controller_rate, angle_stable_time, damping_ratio;
    int drone_id;
    std::string quad_name;
    n.param("init_state_x", init_x, 0.0);
    n.param("init_state_y", init_y, 0.0);
    n.param("init_state_z", init_z, 1.0);
    n.param("init_state_yaw", init_yaw, 1.0);
    init_yaw = init_yaw / 180.0 * M_PI;
    yaw_des = init_yaw;
    n.param("controller_rate", controller_rate, 200.0);
    n.param("quadrotor_name", quad_name, std::string("quadrotor"));
    n.param("angle_stable_time", angle_stable_time, 0.1);
    n.param("damping_ratio", damping_ratio, 0.7);
    n.param("drone_id", drone_id, 0);

    control_RPM_pub = n.advertise<std_msgs::Float32MultiArray>("cmd_RPM", 100);
    ros::Subscriber odom_sub = n.subscribe("odom", 100, Odom_callback,
                                           ros::TransportHints().tcpNoDelay());
    ros::Subscriber cmd_sub = n.subscribe("cmd_pose", 100, cmd_callback,
                                          ros::TransportHints().tcpNoDelay());
    ros::Subscriber position_cmd_sub_ = n.subscribe("position_cmd", 10, fuel_position_cmd_callback,
                                                    ros::TransportHints().tcpNoDelay());

    quad_PID.setdroneid(drone_id);

    ros::Timer controller_timer = n.createTimer(ros::Duration(1.0 / controller_rate), run_control);

    Matrix3d Internal_mat;
    Internal_mat << 2.64e-3, 0, 0,
        0, 2.64e-3, 0,
        0, 0, 4.96e-3;
    double arm_length = 0.22;
    double k_F = 8.98132e-9;
    k_F = 3.0 * k_F;
    double mass = 1.9;

    // cascadePID quad_PID(controller_rate);
    quad_PID.setrate(controller_rate);
    quad_PID.setInternal(mass, Internal_mat, arm_length, k_F);
    quad_PID.setParam(angle_stable_time, damping_ratio);

    ros::Rate rate(controller_rate);
    rate.sleep();

    pos_des << init_x, init_y, init_z;
    vel_des << 0, 0, 0;
    acc_des << 0, 0, 0;

    t_init = ros::Time::now();

    ros::spin();

    return 0;
}

























// #include <nav_msgs/Odometry.h>  //用于接收无人机的里程计数据（位置、速度、姿态等）
// #include <ros/ros.h>
// #include <cascadePID.hpp>   //自定义的级联 PID 控制器类（核心控制逻辑实现）
// #include <std_msgs/Float32MultiArray.h>     //用于发布电机转速指令
// #include <geometry_msgs/PoseStamped.h>
// #include <quadrotor_msgs/PositionCommand.h>

// using namespace std;
// using namespace Eigen;  //用于向量和矩阵运算（如位置、速度、姿态的数学计算）

// ros::Publisher control_RPM_pub; // 声明一个"发布器"，用来向无人机发送"电机转速指令"

// Vector3d pos_des, vel_des, acc_des;  // 期望位置、速度、加速度
// double yaw_des = 0.0;
// cascadePID quad_PID(20); // 创建一个"级联PID控制器"对象（20是临时参数，后面会更新）

// ros::Time t_init, t2;   // 时间变量：t_init是程序启动时间，t2用于计算运行时长

// nav_msgs::Odometry current_odom, last_odom; // 里程计变量：current_odom是当前时刻的里程计，last_odom是上一个时刻的里程计
// int first_odom_flag = 0;

// // 当收到无人机的状态数据（里程计）时，自动调用这个函数
// void Odom_callback(const nav_msgs::Odometry &odom)  
// {
//     if (first_odom_flag == 1)   
//     {
//         last_odom = current_odom;
//     }
//     else
//     {
//         first_odom_flag = 1;
//     }
//     current_odom = odom;
//     ROS_INFO("current odom received");
// }

// geometry_msgs::PoseStamped pose_cmd;    // 存储收到的"简单位置指令"
// int control_flag = 0;

// // 当收到"简单位置指令"（比如从规划器发来的"去(x,y,z)点"）时，自动调用
// void cmd_callback(const geometry_msgs::PoseStamped &msg)    
// {
//     pose_cmd = msg;
//     control_flag = 1;
// }

// // 当收到"详细位置指令"（包含位置、速度、加速度）时，自动调用
// void fuel_position_cmd_callback(const quadrotor_msgs::PositionCommand::ConstPtr &cmd)
// {
//     // 把指令中的位置、速度、加速度存到对应的"期望状态"变量里
//     pos_des = Eigen::Vector3d(cmd->position.x, cmd->position.y, cmd->position.z);
//     vel_des = Eigen::Vector3d(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);
//     acc_des = Eigen::Vector3d(cmd->acceleration.x, cmd->acceleration.y, cmd->acceleration.z);

//     yaw_des = cmd->yaw;
// }

// // 由ROS定时器触发，按设定频率（比如200Hz）执行控制计算
// void run_control(const ros::TimerEvent &event)
// {
//     if (first_odom_flag == 1)   // 只有收到过状态数据，才开始控制
//     {

//         // ROS_INFO("start running controller");

//         // run controller
//         Vector3d current_euler;
//         // 把四元数（表示姿态的一种方式）转成Eigen库的格式，方便计算
//         Eigen::Quaterniond quaternion_temp(current_odom.pose.pose.orientation.w, current_odom.pose.pose.orientation.x, current_odom.pose.pose.orientation.y, current_odom.pose.pose.orientation.z);
//         Vector3d current_pos, current_vel;
//         current_pos << current_odom.pose.pose.position.x, current_odom.pose.pose.position.y, current_odom.pose.pose.position.z;
//         current_vel << current_odom.twist.twist.linear.x, current_odom.twist.twist.linear.y, current_odom.twist.twist.linear.z;
//         quad_PID.FeedbackInput(current_pos, current_vel, quaternion_temp);  // 把当前状态（位置、速度、姿态）传给PID控制器，作为"反馈"
//         // ROS_INFO("current_pos = %f,%f,%f",current_pos(0),current_pos(1),current_pos(2));
//         Eigen::Vector3d eulerAngle_des;
//         eulerAngle_des << 0.1, 0.1, 0.0;
//         // quad_PID.setAngledes(eulerAngle_des);

//         // ROS_INFO("set des angle");

//         if (control_flag == 1)
//         {
//             pos_des << pose_cmd.pose.position.x, pose_cmd.pose.position.y, pose_cmd.pose.position.z;

//             // 从指令的姿态（四元数）中计算期望偏航角（无人机朝向）
//             Eigen::Quaterniond q_des(pose_cmd.pose.orientation.w, pose_cmd.pose.orientation.x, pose_cmd.pose.orientation.y, pose_cmd.pose.orientation.z);
//             Eigen::Matrix3d R_des;
//             R_des = q_des.matrix(); // 四元数转旋转矩阵
//             Eigen::Vector3d x_body;
//             x_body << 1, 0, 0;  // 机身x轴方向（向前）
//             x_body = R_des * x_body;    // 计算机身x轴在世界坐标系中的方向

//             yaw_des = atan2(x_body(1), x_body(0)); // pose_cmd.pose.orientation.z     //用反正切算出期望偏航角（朝向）  偏航角的本质是「无人机机头朝向（机身x轴）与世界 x 轴正方向的夹角」
//             t2 = ros::Time::now();
//             // yaw_des = 3.1415926;//20.8 * (t2-t_init).toSec()
//         }

//         quad_PID.setOdomdes(pos_des, vel_des, acc_des, yaw_des);

//         // ROS_INFO("set des odom");

//         quad_PID.RunController();   // 执行控制计算
//         Eigen::Vector3d Torque_des; // 存储计算出的期望力矩（让无人机转动的力）
//         Torque_des = quad_PID.getTorquedes();   // 从控制器获取期望力矩
//         ROS_INFO("Torque_des = %f,%f,%f", Torque_des(0), Torque_des(1), Torque_des(2));
//         Vector4d RPM_output;    // 存储4个电机的期望转速
//         RPM_output = quad_PID.getRPMoutputs();  // 从控制器获取电机转速
//         // ROS_INFO("RPM_output = %f,%f,%f,%f",RPM_output(0),RPM_output(1),RPM_output(2),RPM_output(3));

//         std_msgs::Float32MultiArray rpm_array;  // 用于发布多个浮点数的消息
//         float rpm_data[4];
//         rpm_data[0] = RPM_output(0);
//         rpm_data[1] = RPM_output(1);
//         rpm_data[2] = RPM_output(2);
//         rpm_data[3] = RPM_output(3);
//         // rpm_array.data = rpm_data;
//         rpm_array.data.clear(); // 清空旧数据
//         rpm_array.data.push_back(RPM_output(0));
//         rpm_array.data.push_back(RPM_output(1));
//         rpm_array.data.push_back(RPM_output(2));
//         rpm_array.data.push_back(RPM_output(3));
//         control_RPM_pub.publish(rpm_array); // 发布转速指令，无人机收到后就会执行
//     }
// }

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "cascade_PID");

//     ros::NodeHandle n("~");

//     double init_x, init_y, init_z;
//     double init_yaw;
//     double controller_rate, angle_stable_time, damping_ratio;   // 控制频率、稳定时间、阻尼比（控制参数）
//     int drone_id;   // 无人机ID（多机时区分用）
//     std::string quad_name;  // 无人机名字
//     n.param("init_state_x", init_x, 0.0);
//     n.param("init_state_y", init_y, 0.0);
//     n.param("init_state_z", init_z, 1.0);
//     n.param("init_state_yaw", init_yaw, 1.0);   // 初始z位置（高度），默认1.0米
//     init_yaw = init_yaw / 180.0 * M_PI; // 角度转弧度（计算要用弧度）
//     yaw_des = init_yaw;
//     n.param("controller_rate", controller_rate, 200.0);
//     n.param("quadrotor_name", quad_name, std::string("quadrotor")); // 无人机名字，默认"quadrotor"
//     n.param("angle_stable_time", angle_stable_time, 0.1);   // 姿态稳定时间，控制参数
//     n.param("damping_ratio", damping_ratio, 0.7);   // 阻尼比（让控制更平稳）
//     n.param("drone_id", drone_id, 0);   // 无人机ID，默认0

//     control_RPM_pub = n.advertise<std_msgs::Float32MultiArray>("cmd_RPM", 100);
//     ros::Subscriber odom_sub = n.subscribe("odom", 100, Odom_callback,
//                                            ros::TransportHints().tcpNoDelay());
//     ros::Subscriber cmd_sub = n.subscribe("cmd_pose", 100, cmd_callback,
//                                           ros::TransportHints().tcpNoDelay());
//     ros::Subscriber position_cmd_sub_ = n.subscribe("position_cmd", 10, fuel_position_cmd_callback,
//                                                     ros::TransportHints().tcpNoDelay());

//     quad_PID.setdroneid(drone_id);

//     ros::Timer controller_timer = n.createTimer(ros::Duration(1.0 / controller_rate), run_control);

//     Matrix3d Internal_mat;
//     Internal_mat << 2.64e-3, 0, 0,
//         0, 2.64e-3, 0,
//         0, 0, 4.96e-3;
//     double arm_length = 0.22;
//     double k_F = 8.98132e-9;
//     k_F = 3.0 * k_F;
//     double mass = 1.9;

//     // cascadePID quad_PID(controller_rate);
//     quad_PID.setrate(controller_rate);
//     quad_PID.setInternal(mass, Internal_mat, arm_length, k_F);
//     quad_PID.setParam(angle_stable_time, damping_ratio);

//     ros::Rate rate(controller_rate);
//     rate.sleep();

//     pos_des << init_x, init_y, init_z;
//     vel_des << 0, 0, 0;
//     acc_des << 0, 0, 0;

//     t_init = ros::Time::now();

//     ros::spin();

//     return 0;
// }