// *** include ***
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

// #include <trajectory_msgs/JointTrajectory.h>
// #include <trajectory_msgs/JointTrajectoryPoint.h>

// Service message
// #include <iiwa_msgs/SetSmartServoJointSpeedLimits.h>

std::vector<float> desired_pose = {0, 0, 0};

void waitForMotion(iiwa_ros::service::TimeToDestinationService& time_2_dist, double time_out = 2.0) {
    double time = time_2_dist.getTimeToDestination();
    ros::Time start_wait = ros::Time::now();
    while (time < 0.0 && (ros::Time::now() - start_wait) < ros::Duration(time_out)) {
        ros::Duration(0.5).sleep();
        time = time_2_dist.getTimeToDestination();
    }
    if (time > 0.0) {
        ros::Duration(time).sleep();
    }
}

void pose_callback(const std_msgs::Float32MultiArray& msg) {
    desired_pose[0] = msg.data[0];
    desired_pose[1] = msg.data[1];
    desired_pose[2] = msg.data[2];
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "inverce_kinematic");
    ros::NodeHandle nh;

    
    // ros spinner
    ros::AsyncSpinner spinner(1);
    spinner.start();
    // Wait a bit, so that you can be sure the subscribers are connected.
    ros::Duration(0.5).sleep();
    
    // *** declare ***
    // services
    iiwa_ros::service::ControlModeService control_mode;
    iiwa_ros::service::PathParametersService j_vel;
    iiwa_ros::service::PathParametersLinService c_vel;
    iiwa_ros::service::TimeToDestinationService time_to_dist;
    // commands
    iiwa_ros::command::CartesianPose cp_command;
    iiwa_ros::command::CartesianPoseLinear cpl_command;
    iiwa_ros::command::JointPosition jp_command;
    // states
    iiwa_ros::state::CartesianWrench cw_state;
    iiwa_ros::state::CartesianPose cp_state;
    iiwa_ros::state::JointVelocity jv_state;
    iiwa_ros::state::JointPosition jp_state;
    // cartesian position msg
    geometry_msgs::PoseStamped init_pos, new_pose;
    // cartesian velocity msg
    geometry_msgs::Twist cartesian_velocity;
    double vel = 0.1;
    double Jvel = 0.2;
    cartesian_velocity.linear.x = vel;
    cartesian_velocity.linear.y = vel;
    cartesian_velocity.linear.z = vel;
    cartesian_velocity.angular.x = vel;
    cartesian_velocity.angular.y = vel;
    cartesian_velocity.angular.z = vel;
    // Подписка на топик
    ros::Subscriber sub = nh.subscribe("/lefttop_point", 1000, pose_callback);
    ros::spinOnce();
    // *** initialize ***
    // services
    control_mode.init("iiwa");
    j_vel.init("iiwa");
    c_vel.init("iiwa");
    time_to_dist.init("iiwa");
    // commands
    cp_command.init("iiwa");
    cpl_command.init("iiwa");
    jp_command.init("iiwa");
    // states
    cw_state.init("iiwa");
    cp_state.init("iiwa");
    jv_state.init("iiwa");
    jp_state.init("iiwa");
    // set the cartesian and joints velocity limit
    j_vel.setSmartServoJointSpeedLimits(Jvel, Jvel);
    c_vel.setMaxCartesianVelocity(cartesian_velocity); 
    ros::Duration(0.5).sleep();  // wait to initialize ros topics
    
    std::vector<float> orient = {0.707165002823, 0.707041292473, -0.00230447391603, -0.00221763853181};
    auto cartesian_position = cp_state.getPose();
    init_pos = cartesian_position.poseStamped;
    while(true)
    {    
        // переместиться в исходное положение
        // Position
        // cartesian_position = cp_state.getPose();
        // if (fabs(cartesian_position.poseStamped.pose.position.x - (init_pos.pose.position.x+desired_pose[0])) > 0.002 && fabs(cartesian_position.poseStamped.pose.position.y - (init_pos.pose.position.y+desired_pose[1])) > 0.002 && fabs(cartesian_position.poseStamped.pose.position.z - (init_pos.pose.position.z+desired_pose[2])) > 0.002)
        init_pos.pose.position.x = desired_pose[0];
        init_pos.pose.position.y = desired_pose[1];
        init_pos.pose.position.z = desired_pose[2];
        // Orientation
        init_pos.pose.orientation.x = orient[0];
        init_pos.pose.orientation.y = orient[1];
        init_pos.pose.orientation.z = orient[2];
        init_pos.pose.orientation.w = orient[3];
        cp_command.setPose(init_pos);
        // std::cout << "Moving to position: (" << init_pos.pose.position.x << ", " << init_pos.pose.position.y << ", " << init_pos.pose.position.z << ")" << std::endl;
        std::cout<<std::to_string(init_pos.pose.position.x)<<", "<<std::to_string(init_pos.pose.position.y)<<", "<<std::to_string(init_pos.pose.position.z)<<std::endl;
        waitForMotion(time_to_dist);
        ros::Duration(0.5).sleep();
    }

    return 0;
}



// // для симулятора gazebo
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

// #include <trajectory_msgs/JointTrajectory.h>
// #include <trajectory_msgs/JointTrajectoryPoint.h>

// void keyboardCallback(const std_msgs::String::ConstPtr& msg)
// {
//     ROS_INFO("Received keyboard input: %s", msg->data.c_str());
// }

// trajectory_msgs::JointTrajectoryPoint createTrajectoryPoint(const std::vector<double>& positions, double time_from_start) {
//     trajectory_msgs::JointTrajectoryPoint point;
//     point.positions = positions;
//     point.time_from_start = ros::Duration(time_from_start);
//     return point;
// }

// std::vector<float> desired_pose = {0, 0, 0};

// void pose_callback(const std_msgs::Float32MultiArray& msg) {
//     desired_pose[0] = msg.data[0];
//     desired_pose[1] = msg.data[1];
//     desired_pose[2] = msg.data[2];
// }

// int main(int argc, char **argv) {
//     ros::init(argc, argv, "inverce_kinematic");
//     ros::NodeHandle nh;

//     // Создание издателя для отправки команд движения
//     ros::Publisher joint_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/iiwa/PositionJointInterface_trajectory_controller/command", 10);

//     // Создание подписчика для получения команд с клавиатуры
//     ros::Subscriber keyboard_sub = nh.subscribe("keyboard_listener", 10, keyboardCallback);

//     // ros spinner
//     ros::AsyncSpinner spinner(1);
//     spinner.start();
//     // Wait a bit, so that you can be sure the subscribers are connected.
//     ros::Duration(0.5).sleep();
    
//     // *** declare ***
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
//     geometry_msgs::PoseStamped init_pos, new_pose;
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
//     // Подписка на топик
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
//     // set the cartesian and joints velocity limit
//     j_vel.setSmartServoJointSpeedLimits(Jvel, Jvel);
//     c_vel.setMaxCartesianVelocity(cartesian_velocity); 
//     ros::Duration(0.5).sleep();  // wait to initialize ros topics
    
//     std::vector<float> orient = {0.707165002823, 0.707041292473, -0.00230447391603, -0.00221763853181};
//     auto cartesian_position = cp_state.getPose();
//     init_pos = cartesian_position.poseStamped;
//     while(true)
//     {    
//         // переместиться в исходное положение
//         // Position
//         // cartesian_position = cp_state.getPose();
//         // if (fabs(cartesian_position.poseStamped.pose.position.x - (init_pos.pose.position.x+desired_pose[0])) > 0.002 && fabs(cartesian_position.poseStamped.pose.position.y - (init_pos.pose.position.y+desired_pose[1])) > 0.002 && fabs(cartesian_position.poseStamped.pose.position.z - (init_pos.pose.position.z+desired_pose[2])) > 0.002)
//         init_pos.pose.position.x = desired_pose[0];
//         init_pos.pose.position.y = desired_pose[0];
//         init_pos.pose.position.z = desired_pose[0];
//         // Orientation
//         init_pos.pose.orientation.x = orient[0];
//         init_pos.pose.orientation.y = orient[1];
//         init_pos.pose.orientation.z = orient[2];
//         init_pos.pose.orientation.w = orient[3];
//         cp_command.setPose(init_pos);
//         // std::cout << "Moving to position: (" << init_pos.pose.position.x << ", " << init_pos.pose.position.y << ", " << init_pos.pose.position.z << ")" << std::endl;
//         std::cout<<std::to_string(init_pos.pose.position.x)<<", "<<std::to_string(init_pos.pose.position.y)<<", "<<std::to_string(init_pos.pose.position.z)<<std::endl;
//         waitForMotion(time_to_dist);
//         ros::Duration(0.5).sleep();
//     }

//     return 0;
// }