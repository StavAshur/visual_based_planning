#pragma once

#include <vector>
#include <random>
#include <memory>
#include <cmath>

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/Point.h>

// Dependencies
#include "../data_structures/VisibilityOracle.h"
#include "../common/Types.h"
#include "ValidityChecker.h"
#include "VisualIK.h"


namespace visual_planner {

class Sampler {
private:
    moveit::core::RobotModelConstPtr robot_model_;
    moveit::core::RobotStatePtr robot_state_;
    std::mt19937 rng_;
    
    std::shared_ptr<VisualIK> vis_ik_;
    std::shared_ptr<VisibilityOracle> vis_oracle_;
    std::shared_ptr<ValidityChecker> validity_checker_;

    double visibility_threshold_ = 0.8;
    BoundingBox workspace_bounds_;

    std::string group_name_ = "tool0";

public:
    Sampler(moveit::core::RobotModelConstPtr model) 
        : robot_model_(model), rng_(std::random_device{}()) 
    {
        robot_state_.reset(new moveit::core::RobotState(robot_model_));
        robot_state_->setToDefaultValues();
        workspace_bounds_ = {-2.0, 2.0, -2.0, 2.0, -0.5, 3.5};
    }

    void setVisualComponents(std::shared_ptr<VisualIK> ik, std::shared_ptr<VisibilityOracle> oracle) {
        vis_ik_ = ik;
        vis_oracle_ = oracle;
    }

    void setVisibilityThreshold(double threshold) {
        visibility_threshold_ = threshold;
    }

    void setValidityChecker(std::shared_ptr<ValidityChecker> checker) {
        validity_checker_ = checker;
    }

    void setWorkspaceBounds(const BoundingBox& bounds) {
        workspace_bounds_ = bounds;
    }

    void setGroupName(const std::string& group) {
        group_name_ = group; 
    }

    std::vector<double> sampleUniform() {
        robot_state_->setToRandomPositions();
        std::vector<double> values;
        robot_state_->copyJointGroupPositions(group_name_, values); 
        return values;
    }

    /**
     * @brief Samples num_points valid 3D points from the workspace.
     * Populates the provided vector to avoid copying.
     */
    void sampleValidPoints(int num_points, std::vector<Eigen::Vector3d>& out_points) {

        std::uniform_real_distribution<double> dist_x(workspace_bounds_.x_min, workspace_bounds_.x_max);
        std::uniform_real_distribution<double> dist_y(workspace_bounds_.y_min, workspace_bounds_.y_max);
        std::uniform_real_distribution<double> dist_z(workspace_bounds_.z_min, workspace_bounds_.z_max);

        out_points.reserve(out_points.size() + num_points);
        int valid_count = 0;
        int attempts = 0;
        int max_attempts = num_points * 100; // Safety break

        while (valid_count < num_points && attempts < max_attempts) {
            double x = dist_x(rng_);
            double y = dist_y(rng_);
            double z = dist_z(rng_);

            if (!validity_checker_->isPointInObstacle(x, y, z)) {
                out_points.emplace_back(x, y, z);
                valid_count++;
            }
            attempts++;
        }
    }

    // --- Helpers ---

    Eigen::Matrix3d computeLookAtRotation(const Eigen::Vector3d& position, const Eigen::Vector3d& target) {
        Eigen::Vector3d z_axis = (target - position).normalized();
        Eigen::Vector3d world_up = Eigen::Vector3d::UnitZ();
        if (std::abs(z_axis.dot(world_up)) > 0.99) world_up = Eigen::Vector3d::UnitX();
        Eigen::Vector3d x_axis = world_up.cross(z_axis).normalized();
        Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();

        Eigen::Matrix3d rotation;
        rotation.col(0) = x_axis; rotation.col(1) = y_axis; rotation.col(2) = z_axis;
        return rotation;
    }

    // --- Advanced Sampling 1: Ball Target ---

    bool sampleVisibilityAwareState(double starting_radius, const Ball& target_ball, std::vector<double>& q_out, int max_iterations = 1000) {
        if (!vis_ik_ || !vis_oracle_) return false;

        std::uniform_real_distribution<double> dist(-1.0, 1.0);
        double current_radius = starting_radius;
        int max_levels = 10;

        for (int level = 0; level < max_levels; ++level) {
            for (int k = 0; k < max_iterations; ++k) {
                // Sample Point
                Eigen::Vector3d offset;
                do {
                    offset = Eigen::Vector3d(dist(rng_), dist(rng_), dist(rng_));
                } while (offset.squaredNorm() > 1.0);
                Eigen::Vector3d p_sample = target_ball.center + offset * current_radius;

                // Check Visibility
                // Using Source Variant of checkBallBeamVisibility
                double vis_score = vis_oracle_->checkBallBeamVisibility(p_sample, target_ball.center, target_ball.radius);

                if (vis_score > visibility_threshold_) {
                    // Visual IK (Ordinary IK logic)
                    // 1. Compute Orientation
                    Eigen::Matrix3d rot = computeLookAtRotation(p_sample, target_ball.center);
                    
                    // 2. Construct Full Pose
                    Eigen::Isometry3d target_pose = Eigen::Isometry3d::Identity();
                    target_pose.translation() = p_sample;
                    target_pose.linear() = rot;
                    
                    // 3. Solve Ordinary IK (No local search)
                    std::vector<double> seed = sampleUniform();
                    if (vis_ik_->solveIK(target_pose, seed, q_out)) {
                        return true;
                    }
                }
            }
            current_radius *= 2.0;
        }
        return false;
    }

    // --- Advanced Sampling 2: Point Set Target ---

    bool sampleVisibilityAwareState(double starting_radius, const std::vector<Eigen::Vector3d>& targets, std::vector<double>& q_out, int max_iterations = 1000) {
        if (!vis_ik_ || !vis_oracle_ || targets.empty()) return false;

        // 1. Compute Centroid for sampling center
        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        for (const auto& t : targets) centroid += t;
        centroid /= targets.size();

        std::uniform_real_distribution<double> dist(-1.0, 1.0);
        double current_radius = starting_radius;
        int max_levels = 10;

        for (int level = 0; level < max_levels; ++level) {
            for (int k = 0; k < max_iterations; ++k) {
                // Sample Point around centroid
                Eigen::Vector3d offset;
                do {
                    offset = Eigen::Vector3d(dist(rng_), dist(rng_), dist(rng_));
                } while (offset.squaredNorm() > 1.0);
                Eigen::Vector3d p_sample = centroid + offset * current_radius;

                // Construct LookAt Pose
                Eigen::Matrix3d rot = computeLookAtRotation(p_sample, centroid);
                Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
                pose.translation() = p_sample;
                pose.linear() = rot;

                // Check Visibility of the Point Set using new Oracle method
                double vis_score = vis_oracle_->checkVisibleFraction(pose, targets);

                if (vis_score > visibility_threshold_) {
                    // Ordinary IK
                    std::vector<double> seed = sampleUniform();
                    if (vis_ik_->solveIK(pose, seed, q_out)) {
                        return true;
                    }
                }
            }
            current_radius *= 2.0;
        }
        return false;
    }
};

} // namespace visual_planner