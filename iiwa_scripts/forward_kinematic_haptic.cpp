// HAPTIC    // Для симулятора gazebo
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
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>



#include <iostream>
#include <csignal>
#include <vector>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
// #include <omni_msgs/OmniButtonEvent.h>
#include <sensor_msgs/JointState.h>


// void hapticCallback(const sensor_msgs::JointState::ConstPtr& msg, ros::Publisher& joint_pub)
// {
//     trajectory_msgs::JointTrajectory traj_msg;
//     traj_msg.joint_names = {"iiwa_joint_1", "iiwa_joint_2", "iiwa_joint_3", "iiwa_joint_4", "iiwa_joint_5", "iiwa_joint_6", "iiwa_joint_7"};

//     trajectory_msgs::JointTrajectoryPoint point;
//     point.positions = {msg->position[0], msg->position[1], msg->position[2], msg->position[3], msg->position[4], msg->position[5], msg->position[6]};
//     point.time_from_start = ros::Duration(0.1);

//     traj_msg.points.push_back(point);
//     joint_pub.publish(traj_msg);
// }
void hapticCallback(const sensor_msgs::JointState::ConstPtr& msg, ros::Publisher& joint_pub)
{
    // Преобразование углов из радианов в градусы
    std::vector<double> joint_angles_degrees;
    for (double angle_rad : msg->position)
    {
        joint_angles_degrees.push_back(angle_rad * 180.0 / M_PI);
    }

    // Вывод углов в градусах
    ROS_INFO("Joint Angles (degrees): [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]", 
        joint_angles_degrees[0], joint_angles_degrees[1], joint_angles_degrees[2], 
        joint_angles_degrees[3], joint_angles_degrees[4], joint_angles_degrees[5] 
        /*joint_angles_degrees[6]*/);

    // Создание сообщения траектории
    trajectory_msgs::JointTrajectory traj_msg;
    traj_msg.joint_names = {"iiwa_joint_1", "iiwa_joint_2", "iiwa_joint_3", "iiwa_joint_4", "iiwa_joint_5", "iiwa_joint_6", "iiwa_joint_7"};

    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = {- msg->position[0], msg->position[1], - msg->position[3], - msg->position[4], msg->position[5], msg->position[6]/*, msg->position[6]*/};
    point.time_from_start = ros::Duration(0.1);

    traj_msg.points.push_back(point);
    joint_pub.publish(traj_msg);
}

int main(int argc, char **argv)
{
    // Инициализация узла ROS
    ros::init(argc, argv, "forward_kinematic");
    ros::NodeHandle nh;

    // Создание издателя для отправки команд движения
    ros::Publisher joint_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/iiwa/PositionJointInterface_trajectory_controller/command", 10);

    // Создание подписчика для получения данных от Haptic устройства
    // ros::Subscriber haptic_sub = nh.subscribe<sensor_msgs::JointState>("/phantom/joint_states", 10, boost::bind(hapticCallback, _1, boost::ref(joint_pub)));
    //ros::Subscriber kuka_pos_sub = nh.subscribe<iiwa_msgs::CartesianPose>("/iiwa/state/CartesianPose", 10,)
    // Задание скорости обновления (10Hz)
    ros::Rate rate(10);

    while (ros::ok())
    {
        // Обработка коллбеков
        ros::spinOnce();

        // Задержка для соблюдения частоты обновления
        rate.sleep();
    }

    return 0;
}
