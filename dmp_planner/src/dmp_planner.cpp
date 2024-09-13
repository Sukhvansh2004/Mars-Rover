#include <pluginlib/class_list_macros.h>
#include "dmp_planner.h"
#include <dmp/GetDMPPlan.h>
#include <dmp/SetActiveDMP.h>
#include <string>

PLUGINLIB_EXPORT_CLASS(dmp_global_planner::DMPGlobalPlanner, nav_core::BaseGlobalPlanner)

namespace dmp_global_planner {

    DMPGlobalPlanner::DMPGlobalPlanner() : nh_("~") {}

    DMPGlobalPlanner::DMPGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) : nh_("~") {
        initialize(name, costmap_ros);
    }

    void DMPGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
        if (!ros::isInitialized()) {
            ROS_FATAL("ROS has not been initialized, unable to create a planner");
            return;
        }

        ros::NodeHandle private_nh("~/" + name);
        set_active_dmp_client_ = nh_.serviceClient<dmp::SetActiveDMP>("set_active_dmp");
        get_dmp_plan_client_ = nh_.serviceClient<dmp::GetDMPPlan>("get_dmp_plan");

    }

    bool DMPGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                                    std::vector<geometry_msgs::PoseStamped>& plan) {
        plan.clear();
        std::string way;
        private_nh.param("~/dmp_waypoint", way, "");
        if(way == "") {
            ROS_ERROR("DMP Way not setup");
            return false
        }
        else ROS_INFO("DMP planning with " + way);

        std::vector<double> x_0 = {start.pose.position.x, start.pose.position.y};
        std::vector<double> goal_pos = {goal.pose.position.x, goal.pose.position.y};
        std::vector<double> x_dot_0 = {0.0, 0.0};
        double t_0 = 0;
        std::vector<double> goal_thresh = {0.2, 0.2};
        double seg_length = -1;
        double tau = 2; // Example value, adjust as needed
        double dt = 1.0;
        int integrate_iter = 5;

        dmp::GetDMPPlan::Request plan_req;
        dmp::GetDMPPlan::Response plan_res;

        plan_req.x_0 = x_0;
        plan_req.x_dot_0 = x_dot_0;
        plan_req.t_0 = t_0;
        plan_req.goal = goal_pos;
        plan_req.goal_thresh = goal_thresh;
        plan_req.seg_length = seg_length;
        plan_req.tau = tau;
        plan_req.dt = dt;
        plan_req.integrate_iter = integrate_iter;

        ROS_INFO("Requesting DMP plan...");

        if (!get_dmp_plan_client_.call(plan_req, plan_res)) {
            ROS_ERROR("Failed to call get_dmp_plan service");
            return false;
        }

        ROS_INFO("DMP planning done.");

        for (const auto& pt : plan_res.plan.points) {
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "map"; // Adjust frame_id as needed
            pose.header.stamp = ros::Time::now(); // Use appropriate timestamp
            pose.pose.position.x = pt.positions[0];
            pose.pose.position.y = pt.positions[1];
            plan.push_back(pose);
        }

        return true;
    }
}
