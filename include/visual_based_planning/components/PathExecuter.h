#pragma once

#include <vector>
#include <string>
#include <memory>

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/RobotTrajectory.h>

class PathExecuter {
private:
    std::string group_name_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    
    // Scaling factors for time parameterization
    double velocity_scaling_ = 0.025;
    double acceleration_scaling_ = 0.05;

public:
    /**
     * @brief Constructor
     * @param group_name The planning group (e.g. "manipulator")
     */
    PathExecuter(const std::string& group_name = "manipulator") 
        : group_name_(group_name)
    {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(group_name_);
    }

    /**
     * @brief Set velocity scaling factor (0.0 to 1.0)
     */
    void setVelocityScaling(double factor) {
        if (factor > 0.0 && factor <= 1.0) {
            velocity_scaling_ = factor;
            // Also set on move_group just in case, though we compute timestamps manually
            move_group_->setMaxVelocityScalingFactor(factor);
        } else {
            ROS_WARN("PathExecuter: Invalid velocity scaling factor %f. Must be (0, 1].", factor);
        }
    }

    /**
     * @brief Set acceleration scaling factor (0.0 to 1.0)
     */
    void setAccelerationScaling(double factor) {
        if (factor > 0.0 && factor <= 1.0) {
            acceleration_scaling_ = factor;
            move_group_->setMaxAccelerationScalingFactor(factor);
        } else {
            ROS_WARN("PathExecuter: Invalid acceleration scaling factor %f. Must be (0, 1].", factor);
        }
    }

    /**
     * @brief Executes a joint-space path directly.
     */
    bool executePath(const std::vector<std::vector<double>>& path) {
        if (path.empty()) {
            ROS_WARN("PathExecuter received empty path.");
            return false;
        }

        ROS_INFO_STREAM("PathExecuter: Preparing execution (Vel: " << velocity_scaling_ << ", Acc: " << acceleration_scaling_ << ")");

        // 1. Create a RobotTrajectory container
        robot_trajectory::RobotTrajectory rt(move_group_->getRobotModel(), group_name_);

        // 2. Convert raw joint vectors to Trajectory Waypoints
        const moveit::core::JointModelGroup* jmg = move_group_->getRobotModel()->getJointModelGroup(group_name_);
        
        for (const auto& joint_cfg : path) {
            moveit::core::RobotState state(move_group_->getRobotModel());
            state.setJointGroupPositions(jmg, joint_cfg);
            rt.addSuffixWayPoint(state, 0.0);
        }

        // 3. Time Parameterization (This is where speed is actually determined!)
        trajectory_processing::IterativeParabolicTimeParameterization iptp;
        
        bool success = iptp.computeTimeStamps(rt, velocity_scaling_, acceleration_scaling_);
        
        if (!success) {
            ROS_ERROR("PathExecuter: Failed to compute time stamps for trajectory.");
            return false;
        }

        // 4. Convert to ROS Message
        moveit_msgs::RobotTrajectory trajectory_msg;
        rt.getRobotTrajectoryMsg(trajectory_msg);

        // 5. Execute
        ROS_INFO("PathExecuter: Sending trajectory to controller...");
        moveit::planning_interface::MoveItErrorCode code = move_group_->execute(trajectory_msg);

        if (code == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ROS_INFO("PathExecuter: Execution successful.");
            return true;
        } else {
            ROS_ERROR_STREAM("PathExecuter: Execution failed with error code: " << code);
            return false;
        }
    }
};