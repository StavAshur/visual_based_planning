#pragma once
#include <vector>
#include <cmath>
#include <algorithm> // for std::ceil
#include <memory>

#include <ros/ros.h>
#include <angles/angles.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>
#include "../common/Types.h"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>

namespace bg = boost::geometry;
typedef bg::model::point<double, 3, bg::cs::cartesian> Point3D;
typedef bg::model::box<Point3D> Box3D;

class ValidityChecker {
private:
    planning_scene::PlanningScenePtr planning_scene_;

    std::vector<Box3D> obstacles_;

    double resolution_; // Resolution in radians

    std::string group_name_= "manipulator";


public:
    ValidityChecker(planning_scene::PlanningScenePtr scene, double resolution = 0.05) 
        : planning_scene_(scene), resolution_(resolution) {}


    /**
     * @brief Update the scene snapshot.
     */
    void setPlanningScene(const planning_scene::PlanningScenePtr& scene) {
        planning_scene_ = scene;
    }

    void clearObstacles() {
        obstacles_.clear();
    }

    void addObstacle(double min_x, double min_y, double min_z, double max_x, double max_y, double max_z) {
        Box3D b(Point3D(min_x, min_y, min_z), Point3D(max_x, max_y, max_z));
        obstacles_.push_back(b);
    }

    /**
     * @brief Checks if a single configuration is valid.
     */
    bool isValid(const std::vector<double>& joint_values) {
        if (!planning_scene_) {
            ROS_ERROR_THROTTLE(1.0, "ValidityChecker: PlanningScene is null!");
            return false;
        }

        moveit::core::RobotState& state = planning_scene_->getCurrentStateNonConst();
        const moveit::core::JointModelGroup* jmg = state.getJointModelGroup("manipulator");
        if (!jmg) return false;
        
        state.setJointGroupPositions(jmg, joint_values);
        state.update(); 
        
        // --- UPDATED COLLISION REQUEST ---
        collision_detection::CollisionRequest req;
        req.contacts = true; // Request contact details
        req.max_contacts = 10; // Get multiple contacts
        req.verbose = false; // Set to true for spammy logs if needed

        collision_detection::CollisionResult res;
        planning_scene_->checkCollision(req, res, state);
        
        if (res.collision) {
            ROS_WARN_THROTTLE(1.0, "State is INVALID!");
            for (const auto& contact : res.contacts) {
                // contact.first is a pair of strings (body_name_1, body_name_2)
                ROS_WARN("Collision between: '%s' and '%s'", 
                         contact.first.first.c_str(), contact.first.second.c_str());
            }
        }
        
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
    bool validateEdge(const std::vector<double>& start, const std::vector<double>& end, visual_planner::EdgeCheckMode mode = visual_planner::EdgeCheckMode::BINARY_SEARCH) {
        if (!isValid(start) || !isValid(end)) return false;

        double dist = distance(start, end);
        if (dist < 1e-6) return true;

        if (mode == visual_planner::EdgeCheckMode::LINEAR) {
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

    void setResolution(double new_res){
        resolution_ = new_res;
    }

    void setGroupName(const std::string& group) {
        group_name_ = group; 
    }

    /**
     * @brief Checks if a 3D point is inside any known obstacle.
     */
    bool isPointInObstacle(double x, double y, double z) const {
        Point3D p(x, y, z);
        for (const auto& box : obstacles_) {
            if (bg::within(p, box)) {
                return true;
            }
        }
        return false;
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