#pragma once
#include <vector>
#include <cmath>
#include <algorithm> // for std::ceil
#include <memory>

#include <ros/ros.h>
#include <angles/angles.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>

enum class EdgeCheckMode {
    LINEAR,
    BINARY_SEARCH
};

class ValidityChecker {
private:
    planning_scene::PlanningScenePtr planning_scene_;
    double resolution_; // Resolution in radians

public:
    ValidityChecker(planning_scene::PlanningScenePtr scene, double resolution = 0.05) 
        : planning_scene_(scene), resolution_(resolution) {}

    /**
     * @brief Checks if a single configuration is valid.
     */
    bool isValid(const std::vector<double>& joint_values) {
        moveit::core::RobotState& state = planning_scene_->getCurrentStateNonConst();
        
        const moveit::core::JointModelGroup* jmg = state.getJointModelGroup("manipulator");
        if (!jmg) {
             ROS_ERROR_ONCE("ValidityChecker: JointModelGroup 'manipulator' not found.");
             return false;
        }
        
        state.setJointGroupPositions(jmg, joint_values);
        
        collision_detection::CollisionRequest req;
        collision_detection::CollisionResult res;
        planning_scene_->checkCollision(req, res, state);
        
        return !res.collision;
    }

    /**
     * @brief Interpolates between two configurations handling cyclic joints.
     */
    std::vector<double> interpolate(const std::vector<double>& start, const std::vector<double>& end, double t) const {
        std::vector<double> result(start.size());
        for (size_t i = 0; i < start.size(); ++i) {
            double diff = angles::shortest_angular_distance(start[i], end[i]);
            double val = start[i] + (diff * t);
            result[i] = angles::normalize_angle(val);
        }
        return result;
    }

    /**
     * @brief Calculates Euclidean distance (handling cycles)
     */
    double distance(const std::vector<double>& a, const std::vector<double>& b) const {
        double dist_sq = 0.0;
        for (size_t i = 0; i < a.size(); ++i) {
            double diff = angles::shortest_angular_distance(a[i], b[i]);
            dist_sq += diff * diff;
        }
        return std::sqrt(dist_sq);
    }

    /**
     * @brief Validates the path segment between two configurations.
     */
    bool validateEdge(const std::vector<double>& start, const std::vector<double>& end, EdgeCheckMode mode = EdgeCheckMode::BINARY_SEARCH) {
        if (!isValid(start) || !isValid(end)) return false;

        double dist = distance(start, end);
        if (dist < 1e-6) return true;

        if (mode == EdgeCheckMode::LINEAR) {
            return validateEdgeLinear(start, end, dist);
        } else {
            return validateEdgeRecursive(start, end, dist);
        }
    }

    /**
     * @brief Extends from 'start' towards 'goal' and returns the furthest valid state reached.
     * Used for RRT steering.
     * - If the whole segment is valid, it returns 'goal'.
     * - If a collision is found, it returns the last valid state before collision.
     * @param max_step Optional limit on how far to extend.
     */
    std::vector<double> extend(const std::vector<double>& start, const std::vector<double>& goal, double max_step = -1.0) {
        double d = distance(start, goal);
        if (d < 1e-6) return start;

        // Determine actual target based on max_step
        std::vector<double> target_state = goal;
        double actual_dist = d;

        if (max_step > 0.0 && d > max_step) {
            double t = max_step / d;
            target_state = interpolate(start, goal, t);
            actual_dist = max_step;
        }

        // Calculate number of checks based on resolution
        int steps = std::ceil(actual_dist / resolution_);
        if (steps == 0) return start;

        std::vector<double> last_valid = start;

        // Linear check loop
        for (int i = 1; i <= steps; ++i) {
            double t = (double)i / (double)steps;
            std::vector<double> intermediate = interpolate(start, target_state, t);

            if (isValid(intermediate)) {
                last_valid = intermediate;
            } else {
                // Collision hit! Return the last valid state we saw.
                return last_valid;
            }
        }

        // If loop completes, we reached the target_state safely
        return target_state;
    }

private:
    bool validateEdgeLinear(const std::vector<double>& start, const std::vector<double>& end, double dist) {
        int steps = std::ceil(dist / resolution_);
        for (int i = 1; i < steps; ++i) {
            double t = (double)i / (double)steps;
            std::vector<double> intermediate = interpolate(start, end, t);
            if (!isValid(intermediate)) return false;
        }
        return true;
    }

    bool validateEdgeRecursive(const std::vector<double>& start, const std::vector<double>& end, double current_dist) {
        if (current_dist <= resolution_) return true;

        std::vector<double> mid = interpolate(start, end, 0.5);
        if (!isValid(mid)) return false;

        return validateEdgeRecursive(start, mid, current_dist * 0.5) && 
               validateEdgeRecursive(mid, end, current_dist * 0.5);
    }
};