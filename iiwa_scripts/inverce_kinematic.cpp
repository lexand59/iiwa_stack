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
// #include <iiwa_msgs/DOF.h>
// #include <iiwa_msgs/CartesianQuantity.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
// Haptic
#include <geometry_msgs/PoseStamped.h>
#include <omni_msgs/OmniButtonEvent.h>

// Глобальный объект для публикации
ros::Publisher pose_pub;

// Переменная для хранения состояния кнопки
bool buttonPressed = false; // проверь

// Коллбэк для подписки на данные от Haptic устройства
void hapticCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // Вывод координат позиции и ориентации
    ROS_INFO("Haptic Device Position: x=%.2f, y=%.2f, z=%.2f",
             msg->pose.position.x * 4, msg->pose.position.y * 4, msg->pose.position.z * 4);

    ROS_INFO("Haptic Device Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f",
             msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    if (buttonPressed) {

    geometry_msgs::PoseStamped pose_msg;
    // pose_msg.header.seq = 0;
    pose_msg.header.stamp = ros::Time::now(); // необходимо ли это?
    pose_msg.header.frame_id = "iiwa_link_0";
    pose_msg.pose.position.x = 0.0; /*msg->pose.position.x * 4;*/
    pose_msg.pose.position.y = 0.0; /*msg->pose.position.y * 4;*/
    pose_msg.pose.position.z = 1.1; /*msg->pose.position.z * 4 + 0.5;*/
    // pose_msg.pose.orientation = msg->pose.orientation; // общая ориентация
    pose_msg.pose.orientation.x = msg->pose.orientation.x;
    pose_msg.pose.orientation.y = msg->pose.orientation.y;
    pose_msg.pose.orientation.z = msg->pose.orientation.z;
    pose_msg.pose.orientation.w = msg->pose.orientation.w;

    pose_pub.publish(pose_msg);
    }
}

// Коллбэк для подписки на состояние кнопки
void buttonCallback(const omni_msgs::OmniButtonEvent::ConstPtr& msg)
{
    buttonPressed = msg->grey_button; 
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "inverce_kinematic");
    ros::NodeHandle nh;
    ros::Duration(0.5).sleep();

    // Подписка на топик с данными от Haptic устройства
    ros::Subscriber haptic_sub = nh.subscribe("/phantom/pose", 10, hapticCallback);

    // Подписка на топик с состоянием кнопки
    ros::Subscriber button_sub = nh.subscribe("/phantom/button", 10, buttonCallback);

    // Публикация в топик управления iiwa
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/iiwa/command/CartesianPose", 10);

    // Create the message object
    // geometry_msgs::PoseStamped pose_msg;
    // pose_msg.header.seq = 0;
    // pose_msg.header.frame_id = "iiwa_link_0";
    // pose_msg.pose.position.x = 0.3;
    // pose_msg.pose.position.y = 0.0;
    // pose_msg.pose.position.z = 0.6;
    // pose_msg.pose.orientation.x = 0.0;
    // pose_msg.pose.orientation.y = 1.0;
    // pose_msg.pose.orientation.z = 0.0;
    // pose_msg.pose.orientation.w = 0.0;

    ros::Rate loop_rate(1); // Set the loop rate (Hz)

    while (ros::ok())
    {
        // Update the timestamp
        // pose_msg.header.stamp = ros::Time::now();

        // Publish the message
        // pose_pub.publish(pose_msg);
        
        // Обработка всех коллбэков
        ros::spinOnce();

        // Sleep for the remainder of the loop
        loop_rate.sleep();
    }

    return 0;
}
