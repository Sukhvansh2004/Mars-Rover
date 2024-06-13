#ifndef SWERVE_DRIVE_CONTROLLER_H
#define SWERVE_DRIVE_CONTROLLER_H

#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <hardware_interface/joint_command_interface.h>

namespace swerve_drive_controller {

class SwerveDriveController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
public:
    SwerveDriveController();

    bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh);
    void update(const ros::Time& time, const ros::Duration& period);
    void starting(const ros::Time& time);
    void stopping(const ros::Time& time);

private:
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void calculateWheelSpeeds(double vx, double vy, double omega);
    void publishOdometry(const ros::Time& current_time);

    ros::Publisher odom_pub;
    ros::Subscriber cmd_vel_sub;

    double wheel_base;
    double track_width;
    double x, y, th;
    double vx, vy, omega;
    ros::Time last_time;

    hardware_interface::JointHandle front_left_wheel_joint;
    hardware_interface::JointHandle front_right_wheel_joint;
    hardware_interface::JointHandle rear_left_wheel_joint;
    hardware_interface::JointHandle rear_right_wheel_joint;

    hardware_interface::JointHandle front_left_steering_joint;
    hardware_interface::JointHandle front_right_steering_joint;
    hardware_interface::JointHandle rear_left_steering_joint;
    hardware_interface::JointHandle rear_right_steering_joint;
};

} // namespace swerve_drive_controller

#endif // SWERVE_DRIVE_CONTROLLER_H
