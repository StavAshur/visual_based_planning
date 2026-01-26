#include <ros/ros.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

// Include your headers
#include "visual_based_planning/VisualPlanner.h"
#include "visual_based_planning/PlanVisibilityPath.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include "../include/visual_based_planning/common/Types.h"

class VisualPlanningNode {
private:
    ros::NodeHandle nh_;  // Global Namespace ("/")
    ros::NodeHandle pnh_; // Private Namespace ("~")
    ros::ServiceServer service_;
    planning_scene_monitor::PlanningSceneMonitorPtr psm_;
    std::shared_ptr<visual_planner::VisualPlanner> planner_;

    // State tracking to detect changes
    bool params_changed_;
    bool current_use_vis_ik_;
    bool current_use_vi_;
    bool current_shortcutting_;
    std::string current_mode_;

public:
    VisualPlanningNode(planning_scene_monitor::PlanningSceneMonitorPtr psm) 
        : psm_(psm), pnh_("~"), params_changed_(false)
    {
        ROS_WARN("Initializing Planning Service");
        planning_scene_monitor::LockedPlanningSceneRO ls(psm_);
        planning_scene::PlanningScenePtr scene_ptr = ls->diff();

        planner_ = std::make_shared<visual_planner::VisualPlanner>(scene_ptr);

        // Load static config (Bounds, Resolution, etc.)
        loadStaticConfig();
        
        // Initialize current state trackers with defaults (or what was just loaded)
        // We assume defaults match the VisualPlanner constructor defaults
        current_use_vis_ik_ = false;
        current_use_vi_ = false;
        current_shortcutting_ = true;

        service_ = nh_.advertiseService("plan_visibility_path", &VisualPlanningNode::planCallback, this);
        ROS_WARN("Visual Planning Service Ready");
    }

    void loadStaticConfig() {
        pnh_.param<std::string>("planner/mode", current_mode_, "VisRRT");

        double resolution;
        pnh_.param<double>("planner/resolution", resolution, 0.05);
        planner_->setResolution(resolution);

        visual_planner::BoundingBox bounds;
        pnh_.param("planner/workspace_bounds/x_min", bounds.x_min, -2.0);
        pnh_.param("planner/workspace_bounds/x_max", bounds.x_max, 2.0);
        pnh_.param("planner/workspace_bounds/y_min", bounds.y_min, -2.0);
        pnh_.param("planner/workspace_bounds/y_max", bounds.y_max, 2.0);
        pnh_.param("planner/workspace_bounds/z_min", bounds.z_min, -0.5);
        pnh_.param("planner/workspace_bounds/z_max", bounds.z_max, 3.5);
        planner_->setWorkspaceBounds(bounds);

        double visibility_threshold;
        pnh_.param<double>("planner/visibility_threshold", visibility_threshold, 0.75);
        planner_->setVisibilityThreshold(visibility_threshold);

        bool use_visual_ik;
        pnh_.param("planner/use_visual_ik", use_visual_ik, true);
        planner_->setUseVisualIK(use_visual_ik);

        std::string group_name;
        pnh_.param<std::string>("planner/group_name", group_name, "manipulator");
        planner_->setGroupName(group_name);

        std::string ee_link;
        pnh_.param<std::string>("planner/ee_link_name", ee_link, "tool0");
        planner_->setEELinkName(ee_link);


        visual_planner::RRTParams rrt; 
        pnh_.param("planner/rrt/goal_bias", rrt.goal_bias, 0.1);
        pnh_.param("planner/rrt/max_extension", rrt.max_extension, 0.5);
        pnh_.param("planner/rrt/max_iterations", rrt.max_iterations, 10000);
        planner_->setRRTParams(rrt);

        visual_planner::PRMParams prm;
        pnh_.param("planner/prm/num_neighbors", prm.num_neighbors, 10);
        pnh_.param("planner/prm/num_samples", prm.num_samples, 100);
        int edge_method_int = 1; 
        pnh_.param("planner/prm/edge_validation_method", edge_method_int, 1);
        prm.edge_validation_method = static_cast<visual_planner::EdgeCheckMode>(edge_method_int);
        pnh_.param("planner/prm/max_size", prm.max_size, 10000);
        pnh_.param("planner/prm/max_goals", prm.max_goals, 10);

        planner_->setPRMParams(prm);

        visual_planner::VisibilityToolParams vt_params; // Initialized with defaults from Types.h
        pnh_.param("planner/visibility_tool_params/beam_angle", vt_params.beam_angle, vt_params.beam_angle);
        pnh_.param("planner/visibility_tool_params/beam_length", vt_params.beam_length, vt_params.beam_length);
        
        // Pass to planner
        planner_->setVisibilityToolParams(vt_params);

        // Load Time Cap
        int time_cap;
        pnh_.param("planner/time_cap", time_cap, 120); // Default to 120 if missing
        planner_->setTimeCap(time_cap);

        bool use_visibility_integrity;
        pnh_.param("planner/use_visibility_integrity", use_visibility_integrity, true);
        planner_->setUseVisibilityIntegrity(use_visibility_integrity);

        visual_planner::VisibilityIntegrityParams vi_params; // Initialized with defaults from Types.h
        pnh_.param("planner/visibility_integrity_params/num_samples", vi_params.num_samples, vi_params.num_samples);
        pnh_.param("planner/visibility_integrity_params/vi_threshold", vi_params.vi_threshold, vi_params.vi_threshold);
        pnh_.param("planner/visibility_integrity_params/k_neighbors", vi_params.k_neighbors, vi_params.k_neighbors);
        pnh_.param("planner/visibility_integrity_params/limit_diameter_factor", vi_params.limit_diameter_factor, vi_params.limit_diameter_factor);
        pnh_.param("planner/visibility_integrity_params/face_samples", vi_params.face_samples, vi_params.face_samples);
        
        // Pass to planner
        planner_->setVisibilityIntegrityParams(vi_params);
    }

    // Helper to reload parameters and detect changes
    void updatePlannerParams() {
        bool changed_this_cycle = false;

        // 1. Visual IK (Global)
        bool new_vis_ik = false;
        if (!nh_.getParam("/planner/use_visual_ik", new_vis_ik)) {
            // If param missing, default to true
            new_vis_ik = false;
        }

        if (new_vis_ik != current_use_vis_ik_) {
            planner_->setUseVisualIK(new_vis_ik);
            current_use_vis_ik_ = new_vis_ik;
            changed_this_cycle = true;
            ROS_INFO("Param Changed: use_visual_ik -> %s", new_vis_ik ? "TRUE" : "FALSE");
        }

        // 2. Visibility Integrity (Global)
        bool new_vi = false;
        if (nh_.getParam("/planner/use_visibility_integrity", new_vi)) {
            // Found specific param
        } else if (nh_.getParam("/planner/visibility_integrity/enabled", new_vi)) {
            // Found nested param
        }

        if (new_vi != current_use_vi_) {
            planner_->setUseVisibilityIntegrity(new_vi);
            current_use_vi_ = new_vi;
            changed_this_cycle = true;
            ROS_INFO("Param Changed: use_visibility_integrity -> %s", new_vi ? "TRUE" : "FALSE");
        }
        
        // 3. Shortcutting (Global)
        bool new_shortcutting = true;
        nh_.getParam("/planner/enable_shortcutting", new_shortcutting);

        if (new_shortcutting != current_shortcutting_) {
            planner_->setShortcutting(new_shortcutting);
            current_shortcutting_ = new_shortcutting;
            changed_this_cycle = true;
            ROS_INFO("Param Changed: enable_shortcutting -> %s", new_shortcutting ? "TRUE" : "FALSE");
        }

        // 4. Mode (Global)
        std::string new_mode;
        if (pnh_.getParam("planner/mode", new_mode)) {
            if (new_mode != current_mode_) {
                current_mode_ = new_mode;
                changed_this_cycle = true;
                ROS_INFO("Param Changed: planner/mode -> %s", current_mode_.c_str());
            }
        }

        // Update the member flag if any change occurred
        if (changed_this_cycle) {
            params_changed_ = true;
        }
    }

    bool planCallback(visual_based_planning::PlanVisibilityPath::Request &req,
                      visual_based_planning::PlanVisibilityPath::Response &res) {
        
        // 1. Check for param updates
        updatePlannerParams();

        planning_scene_monitor::LockedPlanningSceneRO ls(psm_);

        // 2. Modified IF statement: Update scene if not initialized OR params changed
        if (!planner_->isInitialized() || params_changed_) {
            
            ROS_WARN("Planner Re-Initialization Triggered (Init: %d, ParamsChanged: %d)", 
                     planner_->isInitialized(), params_changed_);

            planner_->reset();

            // Get fresh snapshot
            planning_scene::PlanningScenePtr fresh_scene = ls->diff();
            
            // This usually rebuilds the visibility oracle/obstacles
            planner_->setPlanningScene(fresh_scene);

            // Reset the flag now that we've handled the change
            params_changed_ = false;
        }

        // 3. Pass Targets
        planner_->computeTargetMES(req.task.target_points);

        if (!req.task.start_joints.empty()) {
            planner_->setStartJoints(req.task.start_joints);
        } else {
            std::vector<double> current_joints;
            std::string group = planner_->getGroupName(); 
            ls->getCurrentState().copyJointGroupPositions(group, current_joints);
            planner_->setStartJoints(current_joints);
        }

        // 4. Determine Mode (Global Param)
        std::string mode = current_mode_; // Default to what we read from config/param server
        
        // If client overrides it in the request, use that
        if (!req.planner_type.empty()) {
             mode = req.planner_type;
        }

        ROS_WARN("Executing Planner with Mode: %s", mode.c_str());

        bool success = false;
        if (mode.find("PRM") != std::string::npos) {
            // visual_planner::Sampler& sampler = planner_->getSampler();
            // visual_planner::ValidityChecker& validity_checker = planner_->getValidityChecker() ;
            // std::vector<double> start;
            // bool found_random_start = false;

            // while (!found_random_start) {
            //     start = sampler.sampleUniform();
            //     found_random_start = validity_checker.isValid(start);
            // } 

            // success = planner_->planVisPRM(start);
            success = planner_->planVisPRM();
        } else {
            success = planner_->planVisRRT();
        }

        res.success = success;
        
        if (success) {
            const auto& raw_path = planner_->getResultPath();
            std::string group = planner_->getGroupName();
            const moveit::core::JointModelGroup* jmg = ls->getRobotModel()->getJointModelGroup(group);
            res.trajectory.joint_names = jmg->getActiveJointModelNames();
            for (const auto& conf : raw_path) {
                trajectory_msgs::JointTrajectoryPoint point;
                point.positions = conf;
                res.trajectory.points.push_back(point);
            }
        } else {
            ROS_WARN("Planner failed to find a solution.");
        }
        return true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "visual_planning_node");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(2); 
    spinner.start();
    
    robot_model_loader::RobotModelLoaderPtr robot_model_loader(
        new robot_model_loader::RobotModelLoader("robot_description"));

    planning_scene_monitor::PlanningSceneMonitorPtr psm(
        new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));
    
    psm->startSceneMonitor();
    psm->startWorldGeometryMonitor();
    psm->startStateMonitor();

    VisualPlanningNode node(psm);

    ros::waitForShutdown();
    
    return 0;
}