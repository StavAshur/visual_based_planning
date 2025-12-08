#pragma once

// Standard Library
#include <vector>
#include <cmath>
#include <memory>
#include <algorithm>
#include <random>

// ROS
#include <ros/ros.h>
#include <angles/angles.h>
#include <geometry_msgs/Point.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

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

#include "common/Types.h"
namespace visual_planner {

// --- Parameter Structs ---

struct RRTParams {
    double goal_bias = std::pow(0.5, 7);         
    double max_extension = 0.1;      
    int max_iterations = 5000;       
};

struct PRMParams {
    int num_neighbors = 10;          
    int num_samples = 1000;          
    EdgeCheckMode edge_validation_method = EdgeCheckMode::BINARY_SEARCH;
};

class VisualPlanner {
private:
    // --- Configuration & State ---
    double resolution_;
    double visibility_threshold_;
    bool shortcutting_;
    bool use_visual_ik_;
    Ball target_mes_; // Moved to top-level state
    RRTParams rrt_params_;
    PRMParams prm_params_;

    // Planner State Variables
    std::vector<double> start_joint_values_;
    std::vector<std::vector<double>> result_path_;
    

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

    planning_scene::PlanningScenePtr planning_scene_; // Keep reference to scene

    // Robot State Tracking
    moveit::core::RobotStatePtr robot_state_;
    std::string group_name_;
    std::string ee_link_name_;
    
    std::mt19937 rng_;

public:
    // Constructor
    VisualPlanner(const planning_scene::PlanningScenePtr& scene, 
                  const std::vector<geometry_msgs::Point>& targets = {}, 
                  double resolution = 0.05) 
        : planning_scene_(scene),
          resolution_(resolution), 
          visibility_threshold_(0.8),
          group_name_("manipulator"), 
          ee_link_name_("tool0"),
          rng_(std::random_device{}()),
          shortcutting_(true),
          use_visual_ik_(false)
    {
        // 1. Initialize Validity Checker first
        validity_checker_ = std::make_shared<ValidityChecker>(scene, resolution);
        
        // 2. Initialize VisualIK
        vis_ik_ = std::make_shared<VisualIK>(
            scene->getRobotModel(), 
            validity_checker_, 
            group_name_, 
            ee_link_name_ 
        );

        // 3. Initialize Visibility Oracle
        vis_oracle_ = std::make_shared<VisibilityOracle>();
        vis_integrity_ = std::make_shared<VisibilityIntegrity>();

        // 4. Initialize Sampler
        sampler_ = std::make_shared<Sampler>(scene->getRobotModel());
        sampler_->setVisualComponents(vis_ik_, vis_oracle_);
        sampler_->setVisibilityThreshold(visibility_threshold_);

        // 5. Initialize Smoother
        path_smoother_ = std::make_shared<PathSmoother>(validity_checker_);
        
        // 6. Internal Robot State & FK Setup
        robot_state_ = std::make_shared<moveit::core::RobotState>(scene->getRobotModel());
        robot_state_->setToDefaultValues();

        vis_oracle_->setFKSolver([this](const std::vector<double>& joints){
            robot_state_->setJointGroupPositions(group_name_, joints);
            robot_state_->update();
            return robot_state_->getGlobalLinkTransform(ee_link_name_);
        });

        // 7. Compute Target Enclosing Sphere
        computeTargetMES(targets);

        // 8. Populate Visibility Oracle with Obstacles
        updateObstaclesFromScene();
    }

    /**
     * @brief Extracts collision objects from the PlanningScene and adds them to VisibilityOracle.
     */
    void updateObstaclesFromScene() {
        if (!planning_scene_) return;

        // Get list of collision objects in the world
        // Note: This gets attached bodies and world objects
        std::vector<std::string> object_ids = planning_scene_->getWorld()->getObjectIds();
        
        ROS_INFO("VisualPlanner: Extracting %lu obstacles from PlanningScene...", object_ids.size());

        for (const std::string& id : object_ids) {
            // Get the object (CollisionObject)
            const collision_detection::World::ObjectConstPtr& obj = planning_scene_->getWorld()->getObject(id);
            if (!obj) continue;

            // Iterate through shapes in the object (could be mesh, box, etc.)
            for (size_t i = 0; i < obj->shapes_.size(); ++i) {
                // Compute AABB for the shape
                // We need to transform the shape pose to world frame
                Eigen::Isometry3d shape_pose = obj->shape_poses_[i]; 
                
                double x_min, y_min, z_min, x_max, y_max, z_max;
                
                if (obj->shapes_[i]->type == shapes::BOX) {
                    const shapes::Box* box = static_cast<const shapes::Box*>(obj->shapes_[i].get());
                    // Half extents
                    double hx = box->size[0]/2.0;
                    double hy = box->size[1]/2.0;
                    double hz = box->size[2]/2.0;
                    
                    // Transform the 8 corners and find min/max
                    // (Simplified: assuming axis aligned for now, but really should rotate)
                    // If we assume the objects are somewhat axis aligned or we take a loose bound:
                    Eigen::Vector3d center = shape_pose.translation();

                    x_min = center.x() - hx; x_max = center.x() + hx;
                    y_min = center.y() - hy; y_max = center.y() + hy;
                    z_min = center.z() - hz; z_max = center.z() + hz;
                    
                    vis_oracle_->addObstacle(x_min, y_min, z_min, x_max, y_max, z_max, i);
                } 
                else if (obj->shapes_[i]->type == shapes::SPHERE) {
                    const shapes::Sphere* sphere = static_cast<const shapes::Sphere*>(obj->shapes_[i].get());
                    Eigen::Vector3d center = shape_pose.translation();
                    double r = sphere->radius;
                    vis_oracle_->addObstacle(center.x()-r, center.y()-r, center.z()-r, 
                                             center.x()+r, center.y()+r, center.z()+r, i);
                }
                // Add MESH support if needed later
            }
        }
    }

    void setPlanningScene(const planning_scene::PlanningScenePtr& scene) {
        planning_scene_ = scene;
        updateObstaclesFromScene();
        validity_checker_->setPlanningScene(scene);
    }

    void setShortcutting(bool enable) { shortcutting_ = enable; }

    // --- Configuration Getters/Setters ---
    void setRRTParams(const RRTParams& params) { rrt_params_ = params; }
    void setPRMParams(const PRMParams& params) { prm_params_ = params; }
    
    void setVisibilityToolParams(const VisibilityOracle::VisibilityToolParams& params) {
        vis_oracle_->setVisibilityToolParams(params); 
    }

    void setVisibilityThreshold(double t) {
        visibility_threshold_ = t;
        if (sampler_) sampler_->setVisibilityThreshold(t);
    }

    void setUseVisualIK(bool use) { use_visual_ik_ = use; }
    bool getUseVisualIK() const { return use_visual_ik_; }

    void setStartJoints(const std::vector<double>& start) { start_joint_values_ = start; }
    const std::vector<double>& getStartJoints() const { return start_joint_values_; }

    const std::vector<std::vector<double>>& getResultPath() const { return result_path_; }


    // NEW SETTERS needed for full config loading
    void setResolution(double res) { 
        resolution_ = res;
        validity_checker_->setResolution(res);
    }

    void setGroupName(const std::string& group) {
        group_name_ = group; 
        vis_ik_->setGroupName(group);
        sampler_->setGroupName(group);
        validity_checker_->setGroupName(group);
    }
    
    void setEELinkName(const std::string& ee_link) {
        ee_link_name_ = ee_link;
        vis_ik_->setEELinkName(ee_link);
    }
    
    std::string getGroupName() const { return group_name_; }
    std::string getEELinkName() const { return ee_link_name_; }


    // ========================================================================
    // 1. Core Primitives
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

    std::vector<double> extend(const std::vector<double>& start, const std::vector<double>& goal, double max_step = -1.0) {
        double step = (max_step < 0) ? rrt_params_.max_extension : max_step;
        return validity_checker_->extend(start, goal, step);
    }

    // ========================================================================
    // 2. Planner Operations
    // ========================================================================

    void reset() {
        nn_.clear();
        // graph_.clear(); 
    }

    void computeTargetMES(const std::vector<geometry_msgs::Point>& targets) {
        if (targets.empty()) {
            target_mes_ = {Eigen::Vector3d::Zero(), 0.0};
        } else {
            Eigen::Vector3d centroid(0, 0, 0);
            for (const auto& p : targets) {
                centroid += Eigen::Vector3d(p.x, p.y, p.z);
            }
            centroid /= targets.size();

            double max_dist_sq = 0.0;
            for (const auto& p : targets) {
                Eigen::Vector3d pt(p.x, p.y, p.z);
                double d_sq = (pt - centroid).squaredNorm();
                if (d_sq > max_dist_sq) {
                    max_dist_sq = d_sq;
                }
            }
            target_mes_.center = centroid;
            target_mes_.radius = std::sqrt(max_dist_sq);
        }
        ROS_INFO("Target MES has center = (%f,%f,%f) and radius = %f",
                target_mes_.center.x(), target_mes_.center.y(), target_mes_.center.z(), target_mes_.radius);
    }

    VertexDesc addState(const std::vector<double>& q) {
        if (!validity_checker_->isValid(q)) {
            return -1; 
        }
        VertexDesc v = graph_.addVertex(q);
        nn_.addPoint(q, v);
        return v;
    }

    // --- Accessors ---
    
    GraphManager& getGraph() { return graph_; }
    NearestNeighbor& getNN() { return nn_; }
    VisualIK& getVisualIK() { return *vis_ik_; }
    PathSmoother& getSmoother() { return *path_smoother_; }
    ValidityChecker& getValidityChecker() { return *validity_checker_; }
    Ball getTargetMES() const { return target_mes_; }

    // ========================================================================
    // 3. RRT Implementation
    // ========================================================================

    bool planVisRRT() 
    {
        // 0. Initialize
        reset();
        
        // Use member variable start_joint_values_
        if (start_joint_values_.empty()) {
            ROS_ERROR("Start joint values not set!");
            return false;
        }

        VertexDesc root_id = addState(start_joint_values_);
        if (root_id == -1) {
            ROS_ERROR("Start state is invalid!");
            return false;
        }

        // Calculate initial sampling radius
        robot_state_->setJointGroupPositions(group_name_, start_joint_values_);
        robot_state_->update();
        Eigen::Vector3d start_ee = robot_state_->getGlobalLinkTransform(ee_link_name_).translation();
        
        double sampling_radius = (target_mes_.center - start_ee).norm() * 2.0;
        
        std::uniform_real_distribution<double> dist_01(0.0, 1.0);

        ROS_INFO("Starting VisRRT...");

        // 1. Main Loop
        for (int i = 0; i < rrt_params_.max_iterations; ++i) {
            
            ROS_INFO("Starting iteration %d", i);

            std::vector<double> q_rand;
            bool valid_sample = false;

            // --- Step 1: Sampling ---
            // if (dist_01(rng_) > rrt_params_.goal_bias) {
            //     // Uniform
            //     q_rand = sampler_->sampleUniform();
            //     valid_sample = true;
            // } 
            // else {
            //     // Visibility-Biased (Delegated to Sampler)
            //     if (sampler_->sampleVisibilityAwareState(sampling_radius, target_mes_, q_rand)) {
            //         valid_sample = true;
            //     } else {
            //         q_rand = sampler_->sampleUniform();
            //         valid_sample = true;
            //     }
            // }

            q_rand = sampler_->sampleUniform();
            valid_sample = true;
            if (!valid_sample) continue;

            // --- Step 2: Nearest Neighbor ---
            std::vector<VertexDesc> nearest = nn_.kNearest(q_rand, 1);
            if (nearest.empty()) continue;
            VertexDesc q_near_id = nearest[0];
            std::vector<double> q_near = graph_.getVertexConfig(q_near_id);

            // --- Step 3: Extend ---
            std::vector<double> q_new = extend(q_near, q_rand);

            // --- Step 4: Add to Tree ---
            if (distance(q_near, q_new) < 1e-4) continue; 
            
            VertexDesc q_new_id = addState(q_new);
            if (q_new_id == -1) continue;

            graph_.addEdge(q_near_id, q_new_id, distance(q_near, q_new));

            // --- Step 5: Check Goal / Visual IK Connection ---
            
            // Check A: Direct Visibility
            ROS_INFO("Direct visibility check:");
            double current_vis = vis_oracle_->checkBallBeamVisibility(q_new, target_mes_.center, target_mes_.radius);
            
            if (current_vis > visibility_threshold_) {
                ROS_INFO("VisRRT: Found solution via direct visibility.");
                std::vector<VertexDesc> path_idx = graph_.shortestPath(root_id, q_new_id);
                result_path_.clear();
                for (auto v : path_idx) result_path_.push_back(graph_.getVertexConfig(v));
                return true;
            }

            // Check B: VisualIK Variant
            if (use_visual_ik_) {
                // Get current position
                robot_state_->setJointGroupPositions(group_name_, q_new);
                robot_state_->update();
                Eigen::Vector3d current_pos = robot_state_->getGlobalLinkTransform(ee_link_name_).translation();
                
                ROS_INFO("Visibility check for VisualIK:");
                // Check if this *position* is good
                double potential_vis = vis_oracle_->checkBallBeamVisibility(
                    current_pos, 
                    target_mes_.center,
                    target_mes_.radius
                );
                ROS_INFO("Visibility score was %f.", potential_vis);

                if (potential_vis > visibility_threshold_) {
                    // Try to snap
                    Eigen::Matrix3d look_at_rot = sampler_->computeLookAtRotation(current_pos, target_mes_.center);

                    std::vector<double> q_snapped;

                    if (vis_ik_->solveVisualIK(q_new, target_mes_, look_at_rot, q_snapped)) {
                        if (validateEdge(q_new, q_snapped)) {
                            VertexDesc goal_id = addState(q_snapped);
                            if (goal_id != -1) {
                                graph_.addEdge(q_new_id, goal_id, distance(q_new, q_snapped));
                                
                                ROS_INFO("VisRRT: Found solution via VisualIK snap.");
                                std::vector<VertexDesc> path_idx = graph_.shortestPath(root_id, goal_id);
                                result_path_.clear();
                                // for (auto v : path_idx) result_path_.push_back(graph_.getVertexConfig(v));
                                finalizePath(root_id, goal_id);
                                return true;
                            }
                        }
                    }
                }
            }
        }
        
        ROS_WARN("VisRRT: Max iterations reached without solution.");
        return false;
    }

    bool planVisPRM() {
        ROS_INFO("VisPRM placeholder called");
        return true;
    }

    // Helper to extract and smooth path
    void finalizePath(VertexDesc start, VertexDesc goal) {
        std::vector<VertexDesc> path_idx = graph_.shortestPath(start, goal);
        std::vector<std::vector<double>> raw_path;
        raw_path.reserve(path_idx.size());
        
        for (auto v : path_idx) {
            raw_path.push_back(graph_.getVertexConfig(v));
        }

        if (shortcutting_) {
            ROS_INFO("Smoothing path with shortcuts...");
            result_path_ = path_smoother_->smoothPath(raw_path);
        } else {
            result_path_ = raw_path;
        }
    }

};

} // namespace visual_planner