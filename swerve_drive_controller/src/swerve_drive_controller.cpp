#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <hardware_interface/joint_command_interface.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include "swerve_drive_controller/swerve_drive_controller.h"

namespace swerve_drive_controller {

SwerveDriveController::SwerveDriveController() : x(0.0), y(0.0), th(0.0), vx(0.0), vy(0.0), omega(0.0) {}

bool SwerveDriveController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh) {
    // Initialize parameters
    nh.param("wheel_base", wheel_base, 0.5);
    nh.param("track_width", track_width, 0.4);

    // Get joint handles for wheels
    front_left_wheel_joint = hw->getHandle("front_left_wheel_joint");
    front_right_wheel_joint = hw->getHandle("front_right_wheel_joint");
    rear_left_wheel_joint = hw->getHandle("rear_left_wheel_joint");
    rear_right_wheel_joint = hw->getHandle("rear_right_wheel_joint");

    // Get joint handles for steering
    front_left_steering_joint = hw->getHandle("front_left_steering_joint");
    front_right_steering_joint = hw->getHandle("front_right_steering_joint");
    rear_left_steering_joint = hw->getHandle("rear_left_steering_joint");
    rear_right_steering_joint = hw->getHandle("rear_right_steering_joint");

    // Initialize publishers and subscribers
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    cmd_vel_sub = nh.subscribe("cmd_vel", 10, &SwerveDriveController::cmdVelCallback, this);

    last_time = ros::Time::now();

    return true;
}

void SwerveDriveController::update(const ros::Time& time, const ros::Duration& period) {
    // Update odometry and publish
    publishOdometry(time);
}

void SwerveDriveController::starting(const ros::Time& time) {
    // Reset odometry
    x = 0.0;
    y = 0.0;
    th = 0.0;
    last_time = time;
}

void SwerveDriveController::stopping(const ros::Time& time) {}

void SwerveDriveController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    vx = msg->linear.x;
    vy = msg->linear.y;
    omega = msg->angular.z;

    // Calculate the wheel velocities and angles
    calculateWheelSpeeds(vx, vy, omega);
}

void SwerveDriveController::calculateWheelSpeeds(double vx, double vy, double omega) {
    // Calculate wheel speeds and angles
    double r = sqrt(pow(wheel_base, 2) + pow(track_width, 2));
    double a = vx - omega * (wheel_base / r);
    double b = vx + omega * (wheel_base / r);
    double c = vy - omega * (track_width / r);
    double d = vy + omega * (track_width / r);

    double front_left_speed = sqrt(pow(b, 2) + pow(c, 2));
    double front_right_speed = sqrt(pow(b, 2) + pow(d, 2));
    double rear_left_speed = sqrt(pow(a, 2) + pow(c, 2));
    double rear_right_speed = sqrt(pow(a, 2) + pow(d, 2));

    double front_left_angle = atan2(b, c);
    double front_right_angle = atan2(b, d);
    double rear_left_angle = atan2(a, c);
    double rear_right_angle = atan2(a, d);

    // Set the steering angles
    front_left_steering_joint.setCommand(front_left_angle);
    front_right_steering_joint.setCommand(front_right_angle);
    rear_left_steering_joint.setCommand(rear_left_angle);
    rear_right_steering_joint.setCommand(rear_right_angle);

    // Set the wheel speeds
    front_left_wheel_joint.setCommand(front_left_speed);
    front_right_wheel_joint.setCommand(front_right_speed);
    rear_left_wheel_joint.setCommand(rear_left_speed);
    rear_right_wheel_joint.setCommand(rear_right_speed);
}

void SwerveDriveController::publishOdometry(const ros::Time& current_time) {
    double dt = (current_time - last_time).toSec();

    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = omega * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(th);

    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = omega;

    odom_pub.publish(odom);

    last_time = current_time;
}

} // namespace swerve_drive_controller

PLUGINLIB_EXPORT_CLASS(swerve_drive_controller::SwerveDriveController, controller_interface::ControllerBase)
