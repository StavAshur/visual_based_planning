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
        // 1. Lock the scene to safely get the initial state
        planning_scene_monitor::LockedPlanningSceneRO ls(psm_);
        
        // 2. Get a copy of the scene for the planner
        planning_scene::PlanningScenePtr scene_ptr = ls->diff();

        // 3. Initialize Planner with default resolution (will be updated from config)
        planner_ = std::make_shared<visual_planner::VisualPlanner>(scene_ptr);

        // 4. Load Configuration from Parameter Server (YAML)
        loadConfig();

        // 5. Advertise Service
        service_ = nh_.advertiseService("plan_visibility_path", &VisualPlanningNode::planCallback, this);
        ROS_INFO("Visual Planning Service Ready");
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

        // 2. RRT Params
        // We create a local struct which is initialized with the defaults defined in VisualPlanner.h
        visual_planner::RRTParams rrt; 
        
        // We try to overwrite the defaults with values from YAML. 
        // If YAML key is missing, rrt.goal_bias remains its default value.
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
        
        ROS_INFO("Planner configuration loaded.");
    }

    bool planCallback(visual_based_planning::PlanVisibilityPath::Request &req,
                      visual_based_planning::PlanVisibilityPath::Response &res) {
        
        ROS_INFO("Received planning request.");
        
        // 1. Pass Targets to Planner
        planner_->computeTargetMES(req.task.target_points);

        // 2. Pass Start State
        if (!req.task.start_joints.empty()) {
            planner_->setStartJoints(req.task.start_joints);
        } else {
            planning_scene_monitor::LockedPlanningSceneRO ls(psm_);
            std::vector<double> current_joints;
            // Use configured group name if possible, else default
            std::string group = "manipulator"; 
            pnh_.param<std::string>("planner/group_name", group, "manipulator");
            
            ls->getCurrentState().copyJointGroupPositions(group, current_joints);
            planner_->setStartJoints(current_joints);
        }

        // 3. Determine Planner Mode
        // Priority: Service Request Override > YAML Configuration > Default "VisRRT"
        std::string mode;
        pnh_.param<std::string>("planner/mode", mode, "VisRRT");
        
        if (!req.planner_type.empty()) {
             mode = req.planner_type;
        }

        // 4. Execute Planning
        bool success = false;
        if (mode.find("PRM") != std::string::npos) {
             success = planner_->planVisPRM();
        } else {
             success = planner_->planVisRRT();
        }

        // 5. Pack Response
        res.success = success;
        
        if (success) {
            const auto& raw_path = planner_->getResultPath();
            ROS_INFO("Path found with %lu states.", raw_path.size());
            
            planning_scene_monitor::LockedPlanningSceneRO ls(psm_);
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
    
    // Load MoveIt Planning Scene Monitor
    robot_model_loader::RobotModelLoaderPtr robot_model_loader(new robot_model_loader::RobotModelLoader("robot_description"));
    planning_scene_monitor::PlanningSceneMonitorPtr psm(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));
    
    // Start Monitors
    psm->startSceneMonitor();
    psm->startWorldGeometryMonitor();
    psm->startStateMonitor();

    // Instantiate the Node Class
    VisualPlanningNode node(psm);

    ros::spin();
    return 0;
}