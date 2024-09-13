#ifndef DMP_GLOBAL_PLANNER_H
#define DMP_GLOBAL_PLANNER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <costmap_2d/costmap_2d_ros.h>
// #include <base_local_planner/costmap_model.h>
// #include <base_local_planner/world_model.h>
#include <nav_core/base_global_planner.h>

namespace dmp_global_planner {
    class DMPGlobalPlanner : public nav_core::BaseGlobalPlanner {
    public:
        DMPGlobalPlanner();
        DMPGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                      std::vector<geometry_msgs::PoseStamped>& plan);

    private:
        ros::NodeHandle nh_;
        ros::ServiceClient set_active_dmp_client_;
        ros::ServiceClient get_dmp_plan_client_;
    };
};

#endif