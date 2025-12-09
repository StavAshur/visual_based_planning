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
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_; // Private NodeHandle for params
    ros::ServiceServer service_;
    planning_scene_monitor::PlanningSceneMonitorPtr psm_;
    std::shared_ptr<visual_planner::VisualPlanner> planner_;

public:
    VisualPlanningNode(planning_scene_monitor::PlanningSceneMonitorPtr psm) 
        : psm_(psm), pnh_("~") 
    {
        ROS_WARN("Initializing Planning Service");
        // 1. Lock the scene to safely get the initial state
        planning_scene_monitor::LockedPlanningSceneRO ls(psm_);

        // 2. Get a copy of the scene for the planner
        planning_scene::PlanningScenePtr scene_ptr = ls->diff();

        ROS_WARN("Got the scene, creating the planner...");

        // 3. Initialize Planner with default resolution (will be updated from config)
        planner_ = std::make_shared<visual_planner::VisualPlanner>(scene_ptr);

        ROS_WARN("Created the planner, loading config...");

        // 4. Load Configuration from Parameter Server (YAML)
        loadConfig();

        // 5. Advertise Service
        service_ = nh_.advertiseService("plan_visibility_path", &VisualPlanningNode::planCallback, this);
        ROS_WARN("Visual Planning Service Ready");
    }

    void loadConfig() {
        // 1. General Settings
        // param(key, output_var, default_val) handles missing fields gracefully
        
        bool use_vis_ik;
        pnh_.param<bool>("planner/use_visual_ik", use_vis_ik, false); 
        planner_->setUseVisualIK(use_vis_ik);

        double vis_thresh;
        pnh_.param<double>("planner/visibility_threshold", vis_thresh, 0.8);
        planner_->setVisibilityThreshold(vis_thresh);

        bool use_vi;
        pnh_.param<bool>("planner/use_visibility_integrity", use_vi, false);
        planner_->setUseVisibilityIntegrity(use_vi);

        // Load VI Params if enabled (or always, doesn't hurt)
        visual_planner::VisibilityIntegrityParams vi_params;
        pnh_.param("planner/visibility_integrity/vi_threshold", vi_params.vi_threshold, 0.7);
        pnh_.param("planner/visibility_integrity/k_neighbors", vi_params.k_neighbors, 5);
        pnh_.param("planner/visibility_integrity/limit_diameter_factor", vi_params.limit_diameter_factor, 2.0);
        planner_->setVisibilityIntegrityParams(vi_params);

        double resolution;
        // Default 0.05 if not in YAML
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
        ROS_INFO("Workspace Bounds: X[%.1f, %.1f] Y[%.1f, %.1f] Z[%.1f, %.1f]", 
                 bounds.x_min, bounds.x_max, bounds.y_min, bounds.y_max, bounds.z_min, bounds.z_max);

        // Robot Configuration
        std::string group_name;
        // Load group_name from param server, default to "manipulator"
        pnh_.param<std::string>("planner/group_name", group_name, "manipulator");
        planner_->setGroupName(group_name);

        std::string ee_link;
        // Load ee_link_name from param server, default to "tool0"
        pnh_.param<std::string>("planner/ee_link_name", ee_link, "tool0");
        planner_->setEELinkName(ee_link);

        bool shortcutting;
        pnh_.param<bool>("planner/enable_shortcutting", shortcutting, true);
        planner_->setShortcutting(shortcutting);


        // 2. Visibility Tool Params
        VisibilityOracle::VisibilityToolParams visibility_tool_params; 
        pnh_.param("planner/visibility_tool_params/beam_length", visibility_tool_params.beam_length, visibility_tool_params.beam_length);
        pnh_.param("planner/visibility_tool_params/beam_angle", visibility_tool_params.beam_angle, visibility_tool_params.beam_angle);

        planner_->setVisibilityToolParams(visibility_tool_params);

        // 2. RRT Params
        visual_planner::RRTParams rrt; 
        pnh_.param("planner/rrt/goal_bias", rrt.goal_bias, rrt.goal_bias);
        pnh_.param("planner/rrt/max_extension", rrt.max_extension, rrt.max_extension);
        pnh_.param("planner/rrt/max_iterations", rrt.max_iterations, rrt.max_iterations);
        
        planner_->setRRTParams(rrt);

        // 3. PRM Params
        visual_planner::PRMParams prm; // Initialized with defaults
        
        pnh_.param("planner/prm/num_neighbors", prm.num_neighbors, prm.num_neighbors);
        pnh_.param("planner/prm/num_samples", prm.num_samples, prm.num_samples);
        
        // Handle Enum conversion for edge validation
        int edge_method_int = static_cast<int>(prm.edge_validation_method);
        pnh_.param("planner/prm/edge_validation_method", edge_method_int, edge_method_int);
        prm.edge_validation_method = static_cast<visual_planner::EdgeCheckMode>(edge_method_int);

        planner_->setPRMParams(prm);
        
        ROS_INFO("Planner configuration loaded. Group: %s, EE: %s, Res: %.3f", 
                 group_name.c_str(), ee_link.c_str(), resolution);
    }

    bool planCallback(visual_based_planning::PlanVisibilityPath::Request &req,
                      visual_based_planning::PlanVisibilityPath::Response &res) {
        
        ROS_WARN("Received planning request.");
        
        // 1. UPDATE SCENE: Get a fresh snapshot from the monitor
        planning_scene_monitor::LockedPlanningSceneRO ls(psm_);
        planning_scene::PlanningScenePtr fresh_scene = ls->diff();
        
        // Pass this fresh scene to the planner to update obstacles
        planner_->setPlanningScene(fresh_scene);

        // 2. Pass Targets to Planner
        planner_->computeTargetMES(req.task.target_points);

        // 3. Pass Start State
        if (!req.task.start_joints.empty()) {
            planner_->setStartJoints(req.task.start_joints);
        } else {
            std::vector<double> current_joints;
            // Use configured group name if possible, else default
            std::string group = "manipulator"; 
            pnh_.param<std::string>("planner/group_name", group, "manipulator");
            
            ls->getCurrentState().copyJointGroupPositions(group, current_joints);
            planner_->setStartJoints(current_joints);
        }

        // 4. Determine Planner Mode
        // Priority: Service Request Override > YAML Configuration > Default "VisRRT"
        std::string mode;
        pnh_.param<std::string>("planner/mode", mode, "VisRRT");
        
        if (!req.planner_type.empty()) {
             mode = req.planner_type;
        }

        // 5. Execute Planning
        bool success = false;
        if (mode.find("PRM") != std::string::npos) {
             success = planner_->planVisPRM();
        } else {
             success = planner_->planVisRRT();
        }

        // 6. Pack Response
        res.success = success;
        
        if (success) {
            const auto& raw_path = planner_->getResultPath();
            ROS_WARN("Path found with %lu states.", raw_path.size());
            
            std::string group = "manipulator"; 
            pnh_.param<std::string>("planner/group_name", group, "manipulator");
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

    // 1. START ASYNC SPINNER IMMEDIATELY
    // MoveIt requires this to process TF and other background updates during initialization
    ros::AsyncSpinner spinner(2); // Use 2 threads
    spinner.start();
    
    // 2. Load MoveIt Planning Scene Monitor
    // Make sure "robot_description" is reachable. 
    // If your node is in a namespace, you might need "/robot_description"
    robot_model_loader::RobotModelLoaderPtr robot_model_loader(
        new robot_model_loader::RobotModelLoader("robot_description"));

    planning_scene_monitor::PlanningSceneMonitorPtr psm(
        new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));
    
    // 3. Start Monitors
    psm->startSceneMonitor();
    psm->startWorldGeometryMonitor();
    psm->startStateMonitor();

    // 4. Instantiate the Node Class
    VisualPlanningNode node(psm);

    // 5. Wait for shutdown (Replace ros::spin() because AsyncSpinner is running)
    ros::waitForShutdown();
    
    return 0;
}

