#include <ros/ros.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

// Include your headers
#include "visual_based_planning/VisualPlanner.h"
#include "visual_based_planning/PlanVisibilityPath.h"

class VisualPlanningNode {
private:
    ros::NodeHandle nh_;
    ros::ServiceServer service_;
    planning_scene_monitor::PlanningSceneMonitorPtr psm_;
    std::shared_ptr<visual_planner::VisualPlanner> planner_;

public:
    VisualPlanningNode(planning_scene_monitor::PlanningSceneMonitorPtr psm) : psm_(psm) {
        // 1. Lock the scene
        planning_scene_monitor::LockedPlanningSceneRO ls(psm_);
        
        // 2. Fix: Use ->diff() directly. 
        // ls acts like a smart pointer, so -> works automatically.
        planning_scene::PlanningScenePtr scene_ptr = ls->diff();

        // 3. Initialize Planner
        planner_ = std::make_shared<visual_planner::VisualPlanner>(scene_ptr);

        // 4. Advertise Service
        service_ = nh_.advertiseService("plan_visibility_path", &VisualPlanningNode::planCallback, this);
        ROS_INFO("Visual Planning Service Ready");
    }

    bool planCallback(visual_based_planning::PlanVisibilityPath::Request &req,
                      visual_based_planning::PlanVisibilityPath::Response &res) {
        
        ROS_INFO("Received planning request");
        
        // Reset planner state (clear graphs from previous run)
        planner_->reset();

        // Pass task info to planner (You'll need to implement a setTask function in VisualPlanner later)
        // planner_->setTask(req.task);

        if (req.planner_type == "VisRRT") {
            res.success = planner_->planVisRRT();
        } else {
            res.success = planner_->planVisPRM();
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