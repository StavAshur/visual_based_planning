#pragma once

// Standard Library
#include <vector>
#include <cmath>
#include <memory>
#include <algorithm>

// ROS
#include <ros/ros.h>
#include <angles/angles.h>

// MoveIt
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>

// Internal Components
#include "data_structures/Graph.h"
#include "data_structures/NearestNeighbor.h"
#include "data_structures/VisibilityOracle.h"
#include "data_structures/VisibilityIntegrity.h"
#include "components/Sampler.h"
#include "components/ValidityChecker.h"
#include "components/VisualIK.h"
#include "components/PathSmoother.h"

namespace visual_planner {

class VisualPlanner {
private:
    // --- Data Structures ---
    GraphManager graph_;
    NearestNeighbor nn_;
    
    // --- Components ---
    std::shared_ptr<Sampler> sampler_;
    std::shared_ptr<ValidityChecker> validity_checker_;
    std::shared_ptr<VisualIK> vis_ik_;
    std::shared_ptr<PathSmoother> path_smoother_;
    
    // --- Visibility Components ---
    std::shared_ptr<VisibilityOracle> vis_oracle_;
    std::shared_ptr<VisibilityIntegrity> vis_integrity_;

public:
    // Constructor: Injects the MoveIt environment
    VisualPlanner(const planning_scene::PlanningScenePtr& scene, double resolution = 0.05) 
    {
        // 1. Initialize Validity Checker first (Dependencies use it)
        validity_checker_ = std::make_shared<ValidityChecker>(scene, resolution);
        
        // 2. Initialize Sampler
        sampler_ = std::make_shared<Sampler>(scene->getRobotModel());
        
        // 3. Initialize VisualIK
        vis_ik_ = std::make_shared<VisualIK>(
            scene->getRobotModel(), 
            validity_checker_, 
            "manipulator", 
            "tool0" 
        );

        // 4. Initialize Smoother
        path_smoother_ = std::make_shared<PathSmoother>(validity_checker_);

        // 5. Initialize Visibility Structures
        vis_oracle_ = std::make_shared<VisibilityOracle>();
        vis_integrity_ = std::make_shared<VisibilityIntegrity>();
    }

    // ========================================================================
    // 1. Core Primitives (Proxies to ValidityChecker)
    // ========================================================================

    std::vector<double> interpolate(const std::vector<double>& start, const std::vector<double>& end, double t) {
        return validity_checker_->interpolate(start, end, t);
    }

    double distance(const std::vector<double>& a, const std::vector<double>& b) {
        return validity_checker_->distance(a, b);
    }

    bool validateEdge(const std::vector<double>& start, const std::vector<double>& end, EdgeCheckMode mode = EdgeCheckMode::BINARY_SEARCH) {
        return validity_checker_->validateEdge(start, end, mode);
    }

    /**
     * @brief Extends from 'start' towards 'goal' and returns the furthest valid state reached.
     */
    std::vector<double> extend(const std::vector<double>& start, const std::vector<double>& goal, double max_step = -1.0) {
        return validity_checker_->extend(start, goal, max_step);
    }

    // ========================================================================
    // 2. Planner Operations (API for Service)
    // ========================================================================

    void reset() {
        nn_.clear();
        // graph_.clear(); 
    }

    /**
     * @brief Add a start or goal configuration to the roadmap
     */
    VertexDesc addState(const std::vector<double>& q) {
        if (!validity_checker_->isValid(q)) {
            ROS_WARN("Attempted to add invalid state to planner.");
            return -1; 
        }
        VertexDesc v = graph_.addVertex(q);
        nn_.addPoint(q, v);
        return v;
    }

    /**
     * @brief The sampling step for PRM/RRT
     */
    std::vector<double> sampleValidState() {
        std::vector<double> q;
        int max_attempts = 100;
        for(int i=0; i<max_attempts; ++i) {
            q = sampler_->sampleUniform();
            if (validity_checker_->isValid(q)) {
                return q;
            }
        }
        return {}; // Failed to sample
    }

    // --- Accessors ---
    
    GraphManager& getGraph() { return graph_; }
    NearestNeighbor& getNN() { return nn_; }
    VisualIK& getVisualIK() { return *vis_ik_; }
    PathSmoother& getSmoother() { return *path_smoother_; }
    ValidityChecker& getValidityChecker() { return *validity_checker_; }

    // --- Planning Stubs ---
    bool planVisRRT() {
        ROS_INFO("VisRRT placeholder called");
        return true;
    }

    bool planVisPRM() {
        ROS_INFO("VisPRM placeholder called");
        return true;
    }
};

} // namespace visual_planner