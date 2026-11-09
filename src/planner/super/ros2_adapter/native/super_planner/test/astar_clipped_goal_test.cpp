// Exercise the real A* implementation on an empty local map (unknown-as-free).
#include <path_search/astar.h>
#include <ros_interface/ros2/ros2_interface.hpp>
#include <iostream>
#include <stdexcept>

class TestMap : public rog_map::ROGMapROS {
public:
    using rog_map::ROGMapROS::ROGMapROS;
    using rog_map::ROGMap::updateRobotState;
};

int main(int argc, char **argv) {
    if (argc != 2) return 2;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("astar_clipped_goal_test");
    auto map = std::make_shared<TestMap>(node, argv[1]);
    map->updateRobotState({super_utils::Vec3f(0, 0, 2), super_utils::Quatf::Identity()});
    auto ros = std::make_shared<ros_interface::Ros2Interface>(node);
    path_search::Astar astar(argv[1], ros, map);
    const auto start = super_utils::Vec3f(0, 0, 2);
    auto check = [&](const char *name, const super_utils::Vec3f &goal,
                     super_utils::RET_CODE expected) {
        rog_map::vec_Vec3f path;
        const auto result = astar.pointToPointPathSearch(start, goal,
            path_search::ON_INF_MAP | path_search::UNKNOWN_AS_FREE |
            path_search::DONT_USE_INF_NEIGHBOR, 20., path, 5.);
        std::cout << "CASE " << name << " code=" << super_utils::RET_CODE_STR[result]
                  << " points=" << path.size();
        if (!path.empty()) std::cout << " endpoint=" << path.back().transpose();
        std::cout << std::endl;
        if (result != expected || path.size() < 2)
            throw std::runtime_error(std::string(name) + ": unexpected search result");
        if (expected == super_utils::REACH_HORIZON && (path.back()-goal).norm() < 1.)
            throw std::runtime_error(std::string(name) + ": local path reaches distant goal");
        if (expected == super_utils::REACH_GOAL && (path.back()-goal).norm() > .4)
            throw std::runtime_error(std::string(name) + ": near goal not reached");
    };
    check("near-before", {0, 5, 2}, super_utils::REACH_GOAL);
    check("clipped-straight", {0, 235, 2}, super_utils::REACH_HORIZON);
    check("clipped-10-ab", {-37.57, 231.76, 6}, super_utils::REACH_HORIZON);
    check("horizon-before-goal", {19, 19, 2}, super_utils::REACH_HORIZON);
    check("near-after", {0, 5, 2}, super_utils::REACH_GOAL);
    rclcpp::shutdown();
}
