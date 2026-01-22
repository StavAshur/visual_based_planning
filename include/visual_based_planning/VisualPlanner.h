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


class VisualPlanner {
private:
    // --- Configuration & State ---
    double resolution_;
    double visibility_threshold_;
    BoundingBox workspace_bounds_;
    bool shortcutting_;
    bool use_visual_ik_;
    bool use_visibility_integrity_;
    bool is_initialized_ = false;
    VisibilityIntegrityParams visibility_integrity_params_;
    int time_cap_;

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
          use_visual_ik_(false),
          use_visibility_integrity_(false),
          time_cap_(120)
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
        

        // 4. Initialize Sampler
        sampler_ = std::make_shared<Sampler>(scene->getRobotModel());
        sampler_->setVisualComponents(vis_ik_, vis_oracle_);
        sampler_->setVisibilityThreshold(visibility_threshold_);
        sampler_->setValidityChecker(validity_checker_);

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

        const moveit::core::RobotState& current_state = scene->getCurrentState();
        current_state.copyJointGroupPositions(group_name_, start_joint_values_);

        workspace_bounds_ = {-2.0, 2.0, -2.0, 2.0, -0.5, 3.5};
        sampler_->setWorkspaceBounds(workspace_bounds_);
        validity_checker_->setWorkspaceBounds(workspace_bounds_);

        // 7. Compute Target Enclosing Sphere
        computeTargetMES(targets);

        // 8. Initialize visibility integrity tool
        vis_integrity_ = std::make_shared<VisibilityIntegrity>();
        vis_integrity_->setSampler(sampler_);
        vis_integrity_->setVisibilityOracle(vis_oracle_);
        vis_integrity_->setWorkspaceBounds(workspace_bounds_);

        // 9. Populate Visibility Oracle with Obstacles and build visibility integrity tool
        initialize_visibility();

    }

    /**
     * @brief Extracts collision objects from the PlanningScene and adds them to VisibilityOracle.
     */
    void initialize_visibility() {
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
                    
                    vis_oracle_->addObstacle(x_min, y_min, z_min, x_max, y_max, z_max);
                    validity_checker_->addObstacle(x_min, y_min, z_min, x_max, y_max, z_max);
                    sampler_->addObstacle(x_min, y_min, z_min, x_max, y_max, z_max);
                } 
                else if (obj->shapes_[i]->type == shapes::SPHERE) {
                    const shapes::Sphere* sphere = static_cast<const shapes::Sphere*>(obj->shapes_[i].get());
                    Eigen::Vector3d center = shape_pose.translation();
                    double r = sphere->radius;
                    vis_oracle_->addObstacle(center.x()-r, center.y()-r, center.z()-r, 
                                             center.x()+r, center.y()+r, center.z()+r);
                    validity_checker_->addObstacle(center.x()-r, center.y()-r, center.z()-r,
                                                   center.x()+r, center.y()+r, center.z()+r);
                }
                // Add MESH support if needed later
            }
        }
        
        if(use_visibility_integrity_) {
            setVisibilityIntegrityParams(visibility_integrity_params_);
            vis_integrity_->build();
        }

    }

    void setWorkspaceBounds(const BoundingBox& bounds) {
        workspace_bounds_ = bounds;
        sampler_->setWorkspaceBounds(bounds);
        validity_checker_->setWorkspaceBounds(bounds);
        vis_integrity_->setWorkspaceBounds(bounds);
    }

    void setPlanningScene(const planning_scene::PlanningScenePtr& scene) {
        planning_scene_ = scene;
        initialize_visibility();
        validity_checker_->setPlanningScene(scene);
        nn_.setPlanningScene(scene, group_name_);
        is_initialized_ = true;

    }

    void setShortcutting(bool enable) { shortcutting_ = enable; }

    // --- Configuration Getters/Setters ---
    void setRRTParams(const RRTParams& params) {
        rrt_params_ = params; 
    }

    void setPRMParams(const PRMParams& params) { prm_params_ = params; }
    
    void setVisibilityToolParams(const VisibilityToolParams& params) {
        vis_oracle_->setVisibilityToolParams(params);
        vis_ik_->setVisibilityToolParams(params);
    }

    void setVisibilityThreshold(double t) {
        visibility_threshold_ = t;
        if (sampler_) sampler_->setVisibilityThreshold(t);
    }

    void setUseVisualIK(bool use) {
        use_visual_ik_ = use; 
        }
    bool getUseVisualIK() const { return use_visual_ik_; }


    void setUseVisibilityIntegrity(bool enable) {
        use_visibility_integrity_ = enable;
    }

    void setVisibilityIntegrityParams(const VisibilityIntegrityParams& params) {
        visibility_integrity_params_ = params;
        vis_integrity_->setParams(params);
    }

    void setStartJoints(const std::vector<double>& start) {
            start_joint_values_ = start; 
    }
    const std::vector<double>& getStartJoints() const { return start_joint_values_; }


    void setTimeCap(int time_cap){time_cap_ = time_cap;}

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

    bool isInitialized() const { return is_initialized_; }

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
        ROS_WARN("Clearing roadmap graph and NN tree.");
        nn_.clear();
        graph_.clear(); 
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

    VertexDesc addState(const std::vector<double>& q, bool compute_ee_pose = true) {
        Eigen::Isometry3d ee_pose;
        if (compute_ee_pose){
            ee_pose = solveFK(q);
        }
        else
            ee_pose = Eigen::Isometry3d::Identity();
        VertexDesc v = graph_.addVertex(q, ee_pose);
        nn_.addPoint(q, v);
        return v;
    }

    // --- Accessors ---
    
    GraphManager& getGraph() { return graph_; }
    NearestNeighbor& getNN() { return nn_; }
    VisualIK& getVisualIK() { return *vis_ik_; }
    PathSmoother& getSmoother() { return *path_smoother_; }
    ValidityChecker& getValidityChecker() { return *validity_checker_; }
    Sampler& getSampler() {return *sampler_;}

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

        if (!validity_checker_->isValid(start_joint_values_)) {
            ROS_ERROR("Start state is invalid!");
            return false;
        }

        VertexDesc root_id = addState(start_joint_values_);

        // Calculate initial sampling radius
        Eigen::Vector3d start_ee = solveFK(start_joint_values_).translation();
        
        double sampling_radius = (target_mes_.center - start_ee).norm() * 0.5;
        
        std::uniform_real_distribution<double> dist_01(0.0, 1.0);

        ROS_INFO("Starting VisRRT...");
        ros::WallTime start_time = ros::WallTime::now();
        // 1. Main Loop
        for (int i = 0; i < rrt_params_.max_iterations; ++i) {
            
            double elapsed = (ros::WallTime::now() - start_time).toSec();
            if (elapsed > time_cap_) {
                ROS_WARN("VisRRT: Time cap of %d s reached (elapsed: %.2f s). Aborting.", time_cap_, elapsed);
                return false;
            }

            if (i % 100 == 0)
                ROS_INFO("Starting iteration %d", i);

            std::vector<double> q_rand;
            bool valid_sample = false;

            // --- Step 1: Sampling ---
            if (dist_01(rng_) > rrt_params_.goal_bias) {
                // Uniform
                q_rand = sampler_->sampleUniform();
                valid_sample = true;
            } 
            else {
                if (use_visibility_integrity_) {
                    if (sampleVisibilityGoal(q_rand)) {
                        valid_sample = true;
                    }
                }
            }

            if (!valid_sample) {
                q_rand = sampler_->sampleUniform();
            }

            // --- Step 2: Nearest Neighbor ---
            std::vector<VertexDesc> nearest = nn_.kNearest(q_rand, 1);
            if (nearest.empty()) continue;
            VertexDesc q_near_id = nearest[0];
            std::vector<double> q_near = graph_.getVertexConfig(q_near_id);

            // --- Step 3: Extend ---
            std::vector<double> q_new = extend(q_near, q_rand);

            // --- Step 4: Add to Tree ---
            if (distance(q_near, q_new) < 1e-4) continue; 
            
            VertexDesc q_new_id = addState(q_new, false);
            if (q_new_id == -1) continue;

            graph_.addEdge(q_near_id, q_new_id, distance(q_near, q_new));

            // --- Step 5: Check Goal / Visual IK Connection ---
            
            // Check A: Direct Visibility
            double current_vis = vis_oracle_->checkBallBeamVisibility(q_new, target_mes_.center, target_mes_.radius);
            
            if (current_vis > visibility_threshold_) {
                ROS_WARN("VisRRT: Found solution via direct visibility.");
                std::vector<VertexDesc> path_idx = graph_.shortestPath(root_id, q_new_id);
                result_path_.clear();
                finalizePath(root_id, q_new_id);
                return true;
            }

            // Check B: VisualIK Variant
            if (use_visual_ik_) {
                // Get current position
                Eigen::Vector3d current_pos = solveFK(q_new).translation();

                // Check if this *position* is good
                double potential_vis = vis_oracle_->checkBallBeamVisibility(
                    current_pos, 
                    target_mes_.center,
                    target_mes_.radius
                );

                if (potential_vis > visibility_threshold_) {
                    // ROS_INFO("VisRRT: Using VisIK to extend to goal");
                    // Try to snap
                    Eigen::Matrix3d look_at_rot = sampler_->computeLookAtRotation(current_pos, target_mes_.center);

                    std::vector<double> q_snapped;

                    if (vis_ik_->solveVisualIK(q_new, target_mes_, look_at_rot, q_snapped)) {
                        if (validateEdge(q_new, q_snapped)) {
                            VertexDesc goal_id = addState(q_snapped, false);
                            if (goal_id != -1) {
                                graph_.addEdge(q_new_id, goal_id, distance(q_new, q_snapped));
                                
                                ROS_WARN("VisRRT: Found solution via VisualIK snap.");
                                std::vector<VertexDesc> path_idx = graph_.shortestPath(root_id, goal_id);
                                result_path_.clear();
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

    // ========================================================================
    // 4. PRM Implementation
    // ========================================================================

    bool planVisPRM(std::vector<double> start_joint_values) {
        
        if (start_joint_values.empty()) { ROS_ERROR("Start joint values not set!"); return false; }
        
        // 1. Add Start Configuration
        VertexDesc root_id = addState(start_joint_values);     
        
        size_t start_num_neighbors = (graph_.getNumVertices() > 100) ? prm_params_.num_neighbors : 
                                                                        prm_params_.num_neighbors * 3 ;
        
        VertexDesc res = connectToGraph(start_joint_values, start_num_neighbors, root_id);

        std::string success = "SUCCESS";
        std::string failure = "FAILURE";
    
        ROS_WARN("%s connecting start configuration to roadmap", (res != -1) ? success.c_str() : failure.c_str());

        if (root_id == -1) { ROS_ERROR("Start state is invalid!"); return false; }

        robot_state_->setJointGroupPositions(group_name_, start_joint_values);
        robot_state_->update();
        
        std::vector<VertexDesc> goal_ids;

        ROS_INFO("Starting VisPRM...");
        ros::WallTime start_time = ros::WallTime::now();

        ROS_INFO("Scanning existing roadmap for target visibility...");
        
        // Assuming GraphManager exposes the number of vertices. 
        // If not, use appropriate iterator from your Graph.h
        size_t num_vertices = graph_.getNumVertices(); 
        
        for (size_t i = 0; i < num_vertices; ++i) {
            std::vector<double> q_node = graph_.getVertexConfig(i);

            // Actual Visibility Oracle Check
            // Check if this node actually sees the target
            double vis_score = vis_oracle_->checkBallBeamVisibility(graph_.getVertexPose(i), target_mes_.center, target_mes_.radius);
            
            if (vis_score > visibility_threshold_) {
                ROS_INFO("VisPRM: Found existing roadmap node %lu with sufficient visibility (Score: %.2f).", i, vis_score);

                goal_ids.push_back(i);

                // Attempt to find path from Start -> This Node
                std::vector<VertexDesc> path_idx = graph_.shortestPath(root_id, i);
                if (!path_idx.empty()) {
                    ROS_INFO("VisPRM: Path to existing visible node found!");
                    finalizePath(root_id, i);
                    return true;
                }
            }
        }

        int goal_count = 0;
        int max_iterations = 100; // Outer loop limit to avoid infinite run
        for (int iter = 0; iter < max_iterations; ++iter) {
            // ROS_INFO("Number of connected components in roadmap is: %d", graph_.countConnectedComponents());

            double elapsed = (ros::WallTime::now() - start_time).toSec();
            if (elapsed > time_cap_) {
                ROS_WARN("VisPRM: Time cap of %d s reached (elapsed: %.2f s). Aborting.", time_cap_, elapsed);
                return false;
            }

            // 2.1 Sample Goal Configuration
            if ((goal_count < prm_params_.max_goals) || ( graph_.getNumVertices() >= prm_params_.max_size)) {
                ROS_INFO("Looking for more goal configurations...");

                bool found_goal = false;
                std::vector<double> q_goal;

                if (use_visibility_integrity_ ) {
                    if (sampleVisibilityGoal(q_goal)) {
                        found_goal = true;
                        ROS_INFO("Found a goal configuration");
                    }
                }

                if (found_goal) {

                    bool added = false;
                    VertexDesc g_id;
                    if (!validity_checker_->isValid(q_goal))
                        ROS_ERROR("THE GOAL STATE IS INVALID - THIS SHOULD NOT HAPPEN");
                    // Connect to k-NN

                    g_id = connectToGraph(q_goal, prm_params_.num_neighbors);
                    if (g_id != -1) {
                        goal_ids.push_back(g_id);
                        goal_count++;
                        ROS_INFO("Connected a goal configuration to graph");
                    }


                }
            }
            ROS_INFO("Looking for path in roadmap...");

            // 2.2 Check for Path
            for (auto g_id : goal_ids) {
                // Check if start is connected to this goal
                // We use BFS or Dijkstra. Dijkstra is already available.
                std::vector<VertexDesc> path_idx = graph_.shortestPath(root_id, g_id);
                if (!path_idx.empty()) {
                    ROS_WARN("VisPRM: Found solution via visibility integrity!");
                    finalizePath(root_id, g_id);
                    return true;
                }
            }

            // 2.3 Expand Roadmap
            if (!use_visibility_integrity_ || (graph_.getNumVertices() < prm_params_.max_size)) {
                ROS_INFO("VisPRM: Expanding roadmap (Iter %d)...", iter);
                for (int i = 0; i < prm_params_.num_samples; ++i) {
                    std::vector<double> q_rand = sampler_->sampleUniform();
                    if (validity_checker_->isValid(q_rand)) {
                        
                        VertexDesc v_id;

                        if ((graph_.getNumVertices() < prm_params_.max_size)){
                            v_id = addState(q_rand);
                            connectToGraph(q_rand, prm_params_.num_neighbors, v_id);
                        }
                        else {
                            v_id = connectToGraph(q_rand, prm_params_.num_neighbors);
                        }
                            
                        bool perform_vis_check = true;

                        // Oracle Check & Path Termination
                        if (perform_vis_check && v_id != -1) {
                            double vis_score = vis_oracle_->checkBallBeamVisibility(graph_.getVertexPose(v_id), target_mes_.center, target_mes_.radius);
                            
                            if (vis_score > visibility_threshold_) {
                                ROS_INFO("VisPRM: New expanded node %lu sees target (Score: %.2f)", v_id, vis_score);
                                
                                // Check if this new visible node connects back to the start
                                std::vector<VertexDesc> path_idx = graph_.shortestPath(root_id, v_id);
                                if (!path_idx.empty()) {
                                    ROS_WARN("VisPRM: Found solution via roadmap expansion!");
                                    finalizePath(root_id, v_id);
                                    return true;
                                }
                            }
                        }
                    }
                }
            }
        }

        ROS_WARN("VisPRM: Max iterations reached without solution.");
        return false;
    }


    Eigen::Isometry3d solveFK(const std::vector<double>& joints){
        robot_state_->setJointGroupPositions(group_name_, joints);
        robot_state_->update();
        return robot_state_->getGlobalLinkTransform(ee_link_name_);
    }


    VertexDesc connectToGraph(std::vector<double>& q_connect, int k ,VertexDesc v_id = -1) {
        bool connected = false;
        VertexDesc res;
        std::vector<VertexDesc> neighbors = nn_.kNearest(q_connect, k);
        for (auto n_id : neighbors) {
            if (n_id == v_id) continue;

            std::vector<double> q_neighbor = graph_.getVertexConfig(n_id); 

            if (validateEdge(q_connect, q_neighbor, prm_params_.edge_validation_method)) {

                if (v_id == -1) {
                    res = addState(q_connect);
                    graph_.addEdge(res, n_id, distance(q_connect, q_neighbor));
                }
                else{
                    graph_.addEdge(v_id, n_id, distance(q_connect, q_neighbor));
                }
                
                connected = true;
            }
        }
        
        if (v_id != -1)
            return res;
        else {
            res = connected ? res : -1;
            return res;
        }
    }


    bool sampleVisibilityGoal(std::vector<double>& res_sample, int attempts = 100) {

        Eigen::Vector3d sample_pos;

        for (int i = 0; i < attempts; i++) {
            
            if (vis_integrity_->SampleFromVisibilityRegion(target_mes_, sample_pos, visibility_threshold_)) {

                Eigen::Matrix3d sample_ori = sampler_->computeLookAtRotation(sample_pos, target_mes_.center);

                Eigen::Isometry3d sample_pose;
                sample_pose.translation() = sample_pos;
                sample_pose.linear() = sample_ori;
                
                std::vector<double> ik_seed = sampler_->sampleUniform();

                if (vis_ik_->solveIK(sample_pose, ik_seed, res_sample)){
                    return true;
                }
            }
        }

        return false;
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

        if (raw_path.size() >= 2) {
            for (size_t i = 0; i < raw_path.size() - 1; ++i) {
                if (!validity_checker_->validateEdge(raw_path[i], raw_path[i+1])) {
                    ROS_ERROR("VisualPlanner: Final path validation FAILED at edge %lu -> %lu. The path contains collision!", i, i+1);
                    // Depending on policy, we might want to clear result_path_ or just warn.
                    // For now, we warn but proceed to smoothing if enabled (which might fix or fail).
                }
            }
        }

    }

};

} // namespace visual_planner