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

#include "iiwa_haptic/quaternion.hpp"

ros::Publisher iiwa_pose_pub;
geometry_msgs::PoseStamped previous_haptic_pose;
geometry_msgs::PoseStamped iiwa_pose;
geometry_msgs::PoseStamped previous_iiwa_pose;
bool buttonPressed = false;
bool prev_button_state = false;


void haptic_pose_callback(const geometry_msgs::PoseStamped& msg)
{
    printf("Haptic Device Position: x=%.2f, y=%.2f, z=%.2f\n",
             msg.pose.position.x * 4, msg.pose.position.y * 4, msg.pose.position.z * 4 + 0.5);

    printf("Haptic Device Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f\n",
             msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);

    printf("IIWA Position: x=%.2f, y=%.2f, z=%.2f\n",
             iiwa_pose.pose.position.x, iiwa_pose.pose.position.y, iiwa_pose.pose.position.z);

    printf("IIWA Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f\n",
             iiwa_pose.pose.orientation.x, iiwa_pose.pose.orientation.y, iiwa_pose.pose.orientation.z, iiwa_pose.pose.orientation.w);
    printf("\033[4A");
    if(!prev_button_state && buttonPressed)
    {
        previous_haptic_pose = msg;
        previous_iiwa_pose = iiwa_pose;
    }
    if (buttonPressed)
    {
        geometry_msgs::PoseStamped pose_msg;

        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = "iiwa_link_0";

        constexpr double pos_scaler = 20;

        pose_msg.pose.position.x = previous_iiwa_pose.pose.position.x + (msg.pose.position.x - previous_haptic_pose.pose.position.x) * pos_scaler;
        pose_msg.pose.position.y = previous_iiwa_pose.pose.position.y + (msg.pose.position.y - previous_haptic_pose.pose.position.y) * pos_scaler;
        pose_msg.pose.position.z = previous_iiwa_pose.pose.position.z + (msg.pose.position.z - previous_haptic_pose.pose.position.z) * pos_scaler;

        //constexpr double orientation_scaler = 10;

        geometry_msgs::Quaternion haptic_rotation = inverse(previous_haptic_pose.pose.orientation) * msg.pose.orientation; //* inverse(previous_haptic_pose.pose.orientation);
        //haptic_rotation.w *= 2;
        pose_msg.pose.orientation = previous_iiwa_pose.pose.orientation * haptic_rotation; //* power(haptic_rotation, 10);
        //pose

        iiwa_pose_pub.publish(pose_msg);
        //previous_haptic_pose.pose.position = msg.pose.position;
    }
    prev_button_state = buttonPressed;
}

void haptic_button_callback(const omni_msgs::OmniButtonEvent::ConstPtr& msg)
{
    buttonPressed = msg->grey_button; 
}

void iiwa_pose_callback(const iiwa_msgs::CartesianPose& msg_iiwa_pose)
{
    iiwa_pose = msg_iiwa_pose.poseStamped;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "haptic_to_iiwa");
    ros::NodeHandle nh;
    ros::Duration(0.5).sleep();

    // Подписка на топик с данными от Haptic устройства
    ros::Subscriber haptic_pose_sub = nh.subscribe("/phantom/pose", 0, haptic_pose_callback);

    // Подписка на топик с состоянием кнопки
    ros::Subscriber haptic_button_sub = nh.subscribe("/phantom/button", 0, haptic_button_callback);

    ros::Subscriber kuka_pose_sub = nh.subscribe("/iiwa/state/CartesianPose", 0, iiwa_pose_callback);

    // Publisher for iiwa cartesian pose commanding
    iiwa_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/iiwa/command/CartesianPose", 10);


    ros::Rate loop_rate(100); // Set the loop rate (Hz)

    while (ros::ok())
    {
        ros::spinOnce();

        // Sleep for the remainder of the loop
        loop_rate.sleep();
    }

    return 0;
}

//enx00e04c36112c