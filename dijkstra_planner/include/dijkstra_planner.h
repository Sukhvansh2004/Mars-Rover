#ifndef DIJKSTRA_PLANNER_H_
#define DIJKSTRA_PLANNER_H_

#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <pluginlib/class_list_macros.h>
#include <vector>
#include <string>
#include <queue>
#include <map>

namespace dijkstra_planner {

class DijkstraPlanner : public nav_core::BaseGlobalPlanner {
public:
    DijkstraPlanner();
    DijkstraPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

private:
    costmap_2d::Costmap2D* costmap_;
    bool initialized_;
    double resolution_;
    int width_, height_;
    std::vector<int> directions_;
};

} // namespace dijkstra_planner

#endif // DIJKSTRA_PLANNER_H_
