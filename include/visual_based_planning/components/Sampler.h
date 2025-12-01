#pragma once
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <random>

class Sampler {
    moveit::core::RobotModelConstPtr robot_model_;
    moveit::core::RobotStatePtr robot_state_;
public:
    Sampler(moveit::core::RobotModelConstPtr model) : robot_model_(model) {
        robot_state_.reset(new moveit::core::RobotState(robot_model_));
    }

    std::vector<double> sampleUniform() {
        robot_state_->setToRandomPositions();
        std::vector<double> values;
        robot_state_->copyJointGroupPositions("manipulator", values); // Replace "manipulator" with your group
        return values;
    }
};