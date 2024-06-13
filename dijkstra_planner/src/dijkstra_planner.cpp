#include <dijkstra_planner.h>
#include <pluginlib/class_list_macros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <cmath>
#include <queue>
#include <map>

PLUGINLIB_EXPORT_CLASS(dijkstra_planner::DijkstraPlanner, nav_core::BaseGlobalPlanner)

namespace dijkstra_planner {

DijkstraPlanner::DijkstraPlanner() :
    costmap_(nullptr), initialized_(false) {}

DijkstraPlanner::DijkstraPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) :
    costmap_(nullptr), initialized_(false) {
    initialize(name, costmap_ros);
}

void DijkstraPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    if (!initialized_) {
        costmap_ = costmap_ros->getCostmap();
        resolution_ = costmap_->getResolution();
        width_ = costmap_->getSizeInCellsX();
        height_ = costmap_->getSizeInCellsY();
        directions_ = {-1, 0, 1, 0, 0, -1, 0, 1}; // 4 directions (left, up, right, down)
        initialized_ = true;
    }
}

bool DijkstraPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) {
    if (!initialized_) {
        ROS_ERROR("DijkstraPlanner has not been initialized, please call initialize() before using this planner");
        return false;
    }

    plan.clear();

    int start_x = start.pose.position.x / resolution_;
    int start_y = start.pose.position.y / resolution_;
    int goal_x = goal.pose.position.x / resolution_;
    int goal_y = goal.pose.position.y / resolution_;

    std::priority_queue<std::pair<double, std::pair<int, int>>, std::vector<std::pair<double, std::pair<int, int>>>, std::greater<std::pair<double, std::pair<int, int>>>> open_list;
    std::map<std::pair<int, int>, std::pair<int, int>> came_from;
    std::map<std::pair<int, int>, double> cost_so_far;

    open_list.emplace(0, std::make_pair(start_x, start_y));
    came_from[std::make_pair(start_x, start_y)] = std::make_pair(start_x, start_y);
    cost_so_far[std::make_pair(start_x, start_y)] = 0;

    while (!open_list.empty()) {
        auto current = open_list.top().second;
        open_list.pop();

        int curr_x = current.first;
        int curr_y = current.second;

        if (curr_x == goal_x && curr_y == goal_y) {
            break;
        }

        for (int i = 0; i < 4; ++i) {
            int next_x = curr_x + directions_[i * 2];
            int next_y = curr_y + directions_[i * 2 + 1];
            if (next_x < 0 || next_x >= width_ || next_y < 0 || next_y >= height_) {
                continue;
            }

            double new_cost = cost_so_far[current] + costmap_->getCost(next_x, next_y);
            auto next = std::make_pair(next_x, next_y);

            if (cost_so_far.find(next) == cost_so_far.end() || new_cost < cost_so_far[next]) {
                cost_so_far[next] = new_cost;
                double priority = new_cost;
                open_list.emplace(priority, next);
                came_from[next] = current;
            }
        }
    }

    std::pair<int, int> current = std::make_pair(goal_x, goal_y);
    while (!(current.first == start_x && current.second == start_y)) {
        double world_x = current.first * resolution_;
        double world_y = current.second * resolution_;
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = world_x;
        pose.pose.position.y = world_y;
        pose.pose.orientation.w = 1.0;
        plan.push_back(pose);
        current = came_from[current];
    }

    std::reverse(plan.begin(), plan.end());

    return true;
}

} // namespace dijkstra_planner
