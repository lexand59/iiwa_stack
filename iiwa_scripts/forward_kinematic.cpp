// ДЛЯ РЕАЛЬНОГО РОБОТА
// // *** include ***
// #include <cmath>
// #include <functional>
// #include <ros/ros.h>
// // services
// #include <iiwa_ros/service/control_mode.hpp>
// #include <iiwa_ros/service/path_parameters.hpp>
// #include <iiwa_ros/service/path_parameters_lin.hpp>
// #include <iiwa_ros/service/time_to_destination.hpp>
// // commands
// #include <iiwa_ros/command/cartesian_pose.hpp>
// #include <iiwa_ros/command/cartesian_pose_linear.hpp>
// #include <iiwa_ros/command/joint_position.hpp>
// // states
// #include <iiwa_ros/state/cartesian_wrench.hpp>
// #include <iiwa_ros/state/cartesian_pose.hpp>
// #include <iiwa_ros/state/joint_velocity.hpp>
// #include <iiwa_ros/state/joint_position.hpp>
// // conversions functions hpp_file
// #include <iiwa_ros/conversions.hpp>
// // messages
// #include <iiwa_msgs/DOF.h>
// #include <iiwa_msgs/CartesianQuantity.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/Twist.h>
// #include <std_msgs/Float32MultiArray.h>
// #include <std_msgs/String.h>

// #include <iostream>
// #include <csignal>
// #include <vector>

// void keyboardCallback(const std_msgs::String::ConstPtr& msg)
// {
//     ROS_INFO("Received keyboard input: %s", msg->data.c_str());
// }

// std::vector<float> joint_desired_pose={0, 0, 0}; //нулевое положение для шарниров?


// void pose_callback(std_msgs::Float32MultiArray msg)
// {
// //    if (fabs(desired_pose[0]-msg.data[0]) > 0.02 && fabs(desired_pose[1]-msg.data[1]) > 0.02 && fabs(desired_pose[2]-msg.data[2]) > 0.02 )
//     {
//         joint_desired_pose[0] = msg.data[0];
//         joint_desired_pose[1] = msg.data[1];
//         joint_desired_pose[2] = msg.data[2];
//     }
//     // else
//     // {
//     //     desired_pose = {0, 0, 0};
//     // }

// }

// void set_joint_positions(iiwa_ros::command::JointPosition &jp_command, iiwa_msgs::JointPosition &joint_position, const std::vector<double> &positions, double delay) {
//     joint_position.position.a1 = positions[0];
//     joint_position.position.a2 = positions[1];
//     joint_position.position.a3 = positions[2];
//     joint_position.position.a4 = positions[3];
//     joint_position.position.a5 = positions[4];
//     joint_position.position.a6 = positions[5];
//     joint_position.position.a7 = positions[6];
//     jp_command.setPosition(joint_position);
//     ros::Duration(delay).sleep();
// }

// int main(int argc, char **argv)
// {
//     // Инициализация узла ROS
//     ros::init(argc, argv, "forward_kinematic");
//     ros::NodeHandle nh;

//     // ros spinner
//     ros::AsyncSpinner spinner(1);
//     spinner.start();
    
//     // Wait a bit, so that you can be sure the subscribers are connected.
//     ros::Duration(0.5).sleep();

//     // *** decleare ***
//     // services
//     iiwa_ros::service::ControlModeService control_mode;
//     iiwa_ros::service::PathParametersService j_vel;
//     iiwa_ros::service::PathParametersLinService c_vel;
//     iiwa_ros::service::TimeToDestinationService time_to_dist;
//     // commands
//     iiwa_ros::command::CartesianPose cp_command;
//     iiwa_ros::command::CartesianPoseLinear cpl_command;
//     iiwa_ros::command::JointPosition jp_command;
//     // states
//     iiwa_ros::state::CartesianWrench cw_state;
//     iiwa_ros::state::CartesianPose cp_state;
//     iiwa_ros::state::JointVelocity jv_state;
//     iiwa_ros::state::JointPosition jp_state;
//     // cartesian position msg
//     // geometry_msgs::PoseStamped init_pos, new_pose; // Для использования обратной кинематики
//     // cartesian velocity msg
//     geometry_msgs::Twist cartesian_velocity;
//     double vel = 0.1;
//     double Jvel = 0.2;
//     cartesian_velocity.linear.x = vel;
//     cartesian_velocity.linear.y = vel;
//     cartesian_velocity.linear.z = vel;
//     cartesian_velocity.angular.x = vel;
//     cartesian_velocity.angular.y = vel;
//     cartesian_velocity.angular.z = vel;
//     ros::Subscriber sub = nh.subscribe("/lefttop_point", 1000, pose_callback);
//     ros::spinOnce();

//     // *** initialize ***
//     // services
//     control_mode.init("iiwa");
//     j_vel.init("iiwa");
//     c_vel.init("iiwa");
//     time_to_dist.init("iiwa");
//     // commands
//     cp_command.init("iiwa");
//     cpl_command.init("iiwa");
//     jp_command.init("iiwa");
//     // states
//     cw_state.init("iiwa");
//     cp_state.init("iiwa");
//     jv_state.init("iiwa");
//     jp_state.init("iiwa");

//     ros::Publisher chatter_pub = nh.advertise<std_msgs::Float32MultiArray>("Coordinate_public", 1000);
//     // // gripper
//     // std_msgs::Bool gripper;
//     // bool open = true;
//     // bool close = false;
//     // bool station = true;
//     // gripper.data = open;
//     // // gripper_command.publish(gripper);

//     // set the cartesian and joints velocity limit
//     j_vel.setSmartServoJointSpeedLimits(Jvel, Jvel);
//     c_vel.setMaxCartesianVelocity(cartesian_velocity); 
//     ros::Duration(0.5).sleep();  // wait to initialize ros topics
//     auto cartesian_position = cp_state.getPose();
//     auto joint_position = jp_state.getPosition();
//     std_msgs::Float32MultiArray msg;

//     set_joint_positions(jp_command, joint_position, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 5);
//     set_joint_positions(jp_command, joint_position, {0.78, 0.7, 0.0, -0.7, 0.0, 0.7, 0.0}, 5);
//     set_joint_positions(jp_command, joint_position, {0.0, 0.5, 0.0, -0.2, 0.0, 0.2, 0.0}, 5);
//     set_joint_positions(jp_command, joint_position, {-0.78, 0.7, 0.0, -0.7, 0.0, 0.7, 0.0}, 5);
//     set_joint_positions(jp_command, joint_position, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 5);

//     return 0;
// }


    // ДЛЯ СИМУЛЯТОРА GAZEBO
#include <cmath>
#include <functional>
#include <ros/ros.h>
// services
#include <iiwa_ros/service/control_mode.hpp>
#include <iiwa_ros/service/path_parameters.hpp>
#include <iiwa_ros/service/path_parameters_lin.hpp>
#include <iiwa_ros/service/time_to_destination.hpp>
// commands
#include <iiwa_ros/command/cartesian_pose.hpp>
#include <iiwa_ros/command/cartesian_pose_linear.hpp>
#include <iiwa_ros/command/joint_position.hpp>
// states
#include <iiwa_ros/state/cartesian_wrench.hpp>
#include <iiwa_ros/state/cartesian_pose.hpp>
#include <iiwa_ros/state/joint_velocity.hpp>
#include <iiwa_ros/state/joint_position.hpp>
// conversions functions hpp_file
#include <iiwa_ros/conversions.hpp>
// messages
#include <iiwa_msgs/DOF.h>
#include <iiwa_msgs/CartesianQuantity.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>

#include <iostream>
#include <csignal>
#include <vector>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

// void keyboardCallback(const std_msgs::String::ConstPtr& msg)
// {
//     ROS_INFO("Received keyboard input: %s", msg->data.c_str());
// }

trajectory_msgs::JointTrajectoryPoint createTrajectoryPoint(const std::vector<double>& positions, double time_from_start) {
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = positions;
    point.time_from_start = ros::Duration(time_from_start);
    return point;
}

int main(int argc, char **argv)
{
    // Инициализация узла ROS
    ros::init(argc, argv, "forward_kinematic");
    ros::NodeHandle nh;

    // Создание издателя для отправки команд движения
    ros::Publisher joint_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/iiwa/PositionJointInterface_trajectory_controller/command", 10);

    // Создание подписчика для получения команд с клавиатуры
    // ros::Subscriber keyboard_sub = nh.subscribe("keyboard_listener", 10, keyboardCallback);

    // Задание скорости обновления (10Hz)
    ros::Rate rate(10);

    // Ожидание 1.5 секунды перед началом выполнения траектории
    ros::Duration(1.5).sleep();

    // Создание сообщения о траектории движения
    trajectory_msgs::JointTrajectory msg;
    msg.joint_names = {"iiwa_joint_1", "iiwa_joint_2", "iiwa_joint_3", "iiwa_joint_4", "iiwa_joint_5", "iiwa_joint_6", "iiwa_joint_7"};

    // Определение точек траектории и времени достижения
    std::vector<trajectory_msgs::JointTrajectoryPoint> points;
    points.push_back(createTrajectoryPoint({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 3.0));
    points.push_back(createTrajectoryPoint({0.78, 0.7, 0.0, -0.7, 0.0, 0.7, 0.0}, 6.0)); //3.0 - 3 секудна
    points.push_back(createTrajectoryPoint({0.0, 0.5, 0.0, -0.2, 0.0, 0.2, 0.0}, 9.0));
    points.push_back(createTrajectoryPoint({-0.78, 0.7, 0.0, -0.7, 0.0, 0.7, 0.0}, 12.0));
    points.push_back(createTrajectoryPoint({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 15.0));

    // Добавление точек к сообщению о траектории
    msg.points.insert(msg.points.end(), points.begin(), points.end());

    // Отправка сообщения
    joint_pub.publish(msg);

    // Задержка для соблюдения частоты обновления
    rate.sleep();

    // Обработка коллбеков
    ros::spinOnce();

    return 0;
}