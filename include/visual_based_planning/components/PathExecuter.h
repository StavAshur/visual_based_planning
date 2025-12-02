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

public:
    /**
     * @brief Constructor
     * @param group_name The planning group (e.g. "manipulator")
     */
    PathExecuter(const std::string& group_name = "manipulator") 
        : group_name_(group_name)
    {
        // Initialize MoveGroupInterface
        // Note: Requires an AsyncSpinner to be running in the main node for callbacks
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(group_name_);
        
        // Set default scaling factors (can be adjusted via setters)
        move_group_->setMaxVelocityScalingFactor(1.0);
        move_group_->setMaxAccelerationScalingFactor(1.0);
    }

    /**
     * @brief Executes a joint-space path directly.
     * The robot will traverse the specific joint configurations in 'path' in order.
     * Movement between points is linear in joint space (standard interpolation).
     * * @param path A vector of joint configurations (each is a vector<double>)
     * @return true if execution succeeds
     */
    bool executePath(const std::vector<std::vector<double>>& path) {
        if (path.empty()) {
            ROS_WARN("PathExecuter received empty path.");
            return false;
        }

        ROS_INFO_STREAM("PathExecuter: Preparing to execute path with " << path.size() << " waypoints.");

        // 1. Create a RobotTrajectory container
        // This structure holds the sequence of robot states
        robot_trajectory::RobotTrajectory rt(move_group_->getRobotModel(), group_name_);

        // 2. Convert raw joint vectors to Trajectory Waypoints
        const moveit::core::JointModelGroup* jmg = move_group_->getRobotModel()->getJointModelGroup(group_name_);
        
        for (const auto& joint_cfg : path) {
            // Create a state based on the robot model
            moveit::core::RobotState state(move_group_->getRobotModel());
            
            // Set positions (this handles mapping vector<double> to joint names)
            // It also handles cyclic joints (normalizing them to range), ensuring the
            // state stored in the trajectory is valid.
            state.setJointGroupPositions(jmg, joint_cfg);
            
            // Add to trajectory. 
            // dt=0.0 is a placeholder; timestamps will be computed in step 3.
            rt.addSuffixWayPoint(state, 0.0);
        }

        // 3. Time Parameterization
        // This calculates velocities and accelerations for each waypoint so the trajectory
        // is physically executable. Without this, the timestamps are 0 and execution fails.
        trajectory_processing::IterativeParabolicTimeParameterization iptp;
        
        // We can create a smoother trajectory by adjusting these factors if needed
        double velocity_scaling = 1.0;
        double acceleration_scaling = 1.0;
        
        bool success = iptp.computeTimeStamps(rt, velocity_scaling, acceleration_scaling);
        
        if (!success) {
            ROS_ERROR("PathExecuter: Failed to compute time stamps for trajectory.");
            return false;
        }

        // 4. Convert to ROS Message for execution
        moveit_msgs::RobotTrajectory trajectory_msg;
        rt.getRobotTrajectoryMsg(trajectory_msg);

        // 5. Execute
        // This is a blocking call (waits for execution to finish).
        // If you need non-blocking, use move_group_->asyncExecute(trajectory_msg);
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