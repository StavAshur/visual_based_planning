#pragma once
#include <moveit/move_group_interface/move_group_interface.h>

class PathExecuter {
    moveit::planning_interface::MoveGroupInterface* move_group_;
public:
    PathExecuter(moveit::planning_interface::MoveGroupInterface* mg) : move_group_(mg) {}

    bool execute(const std::vector<std::vector<double>>& path) {
        // Stub: Convert to trajectory and execute
        return true;
    }
};