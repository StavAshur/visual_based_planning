#pragma once
#include <moveit/planning_scene/planning_scene.h>

class ValidityChecker {
    planning_scene::PlanningScenePtr planning_scene_;
public:
    ValidityChecker(planning_scene::PlanningScenePtr scene) : planning_scene_(scene) {}

    bool isValid(const std::vector<double>& joint_values) {
        moveit::core::RobotState& state = planning_scene_->getCurrentStateNonConst();
        state.setJointGroupPositions("manipulator", joint_values);
        state.update();
        return !planning_scene_->isStateColliding(state);
    }
};