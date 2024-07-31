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